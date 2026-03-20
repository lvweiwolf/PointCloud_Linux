#include <LasFile/PointCloudTool.h>
#include <LasFile/PointCloudPageLod.h>
#include <LasFile/PointCloudPropertyTool.h>
#include <LasFile/PointCloudToolkit.h>
#include <LasFile/PointProcessThread.h>

#include <BusinessNode/PCNodeType.h>
#include <BusinessNode/BnsPointCloudNode.h>

#include <Tool/FileToolkit.h>
#include <Tool/LibToolkit.h>
#include <Tool/XmlDocument.h>

#include <osg/ComputeBoundsVisitor>
#include <osg/MatrixTransform>
#include <osg/Geode>

#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_pointcloud.h>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#define USE_MULTI_THREAD
#define USE_READ_BATCH_POINT


#define MAX_MEMORY_POINT_NUM 40000000
#define PIECE_SAVE_NUM 500000
#define OctteeSize 10
#define PieceMinSize 125.0
#define PieceOneSize 250.00
#define PieceTwoSize 1000.00
#define PieceLasSize 1000.0

static const int LAS_TINY_RIGHT_MOVE = 8;
static const int RESERVE = 15625; // 扩容后点能达到默认50万点
static const int LAS_TINY_TARGET_NUM_VERT = 10000;
static const int nSpliteSize = 2000000; // 单位线程处理点数量

static const TCHAR* IVE_FILE_NAME = L"FileName";	// 文件名
static const TCHAR* PAGED_LOD_ID = L"PageLodID";	// PAGEDLODID
static const TCHAR* PIECE_POINTS_NUM = L"PointNum"; // 每片的点数量

std::map<unsigned, unsigned> CPointCloudTool::ReadTypeMapping()
{
	std::map<unsigned, unsigned> typeMapping;
	CString strAppPath = CLibToolkit::GetAppModuleDir(nullptr);
	CString strCfgPath = strAppPath + L"/PointTypeMap.xml";

	if (!CFileToolkit::FileExist(strCfgPath))
		return typeMapping;

	toolkit::CXmlDocument doc;
	if (!doc.LoadFile(strCfgPath, toolkit::fmtXMLUTF8))
		return typeMapping;

	toolkit::CXmlElement* pRoot = doc.GetElementRoot();
	if (nullptr == pRoot)
		return typeMapping;

	auto pChilds = doc.GetElementRoot()->GetChildElements();
	if (nullptr == pChilds)
		return typeMapping;
	size_t nCount = pChilds->GetCount();
	for (size_t i = 0; i < nCount; ++i)
	{
		auto pEle = pChilds->GetAt(i);
		if (nullptr == pEle)
			continue;
		unsigned nType = CStringToolkit::StrToInt(pEle->GetAttrValue(L"Type"));
		unsigned nTo = CStringToolkit::StrToInt(pEle->GetAttrValue(L"To"));
		typeMapping[nType] = nTo;
	}
	return typeMapping;
}


CPointCloudTool::CPointCloudTool(const CString& strLasFilePath,
								 pc::data::CModelNodePtr pProjectNode,
								 const CString& strOutPath)
	: _strLasFilePath(strLasFilePath),
	  _pProjectNode(pProjectNode),
	  _strOutPath(strOutPath),
	  _bShuZiLvDiLas(false),
	  _bBoowayLas(false),
	  _dOffSetZ(0.0),
	  _writeFileThread(nullptr)
{
	_nStartX = 0;
	_nStartY = 0;
	_nStartZ = 0;
	_nLasIdex = 0;
	_dLasCenterHight = 0.0;
	_dHight = 0.0;
	_dMin_Z = 0.0;
}

CPointCloudTool::~CPointCloudTool() {}

void CPointCloudTool::LoadLas()
{
	CBnsProjectNode bnsProject = _pProjectNode;
	if (bnsProject.IsNull())
		return;

	_typeMapping = ReadTypeMapping();
	FileOpen();
	GetScanTime();

	// 启动写文件线程
	_writeFileThread = new CPointCloudWriteThread();
	_writeFileThread->start();

	pc::data::CModelNodePtr pPageLod = Las2Osgb();
	if (nullptr == pPageLod)
		return;

	CString strNameId = CFileToolkit::GetFileBaseName(_strLasFilePath);

	std::string fragId = CStringToolkit::CStringToUTF8(strNameId);
	// 创建自定义属性容器
	// osg::ref_ptr<osg::UserDataContainer> udc = new osg::DefaultUserDataContainer;
	// udc->addUserObject(new osg::StringValueObject("LasFileName", fragId)); // 字符串属性
	// 绑定到 PagedLOD 节点
	// pPageLod->setUserDataContainer(udc.get());

	// if (pPageLod != NULL)
	// vecFileToNode.emplace_back(std::make_pair(strFileName, pPageLod));

	SavePieceInfo2Xml(_dAllLasRadius, _centerPoint, _dHight, _dMin_Z);

	bnsProject.InsertPointCloudNode(pPageLod);

	bnsProject.setBasePos(_centerPoint);

	// 等待写文件结束，销毁线程
	_writeFileThread->WaitComplete();
	SAFE_DELETE(_writeFileThread);
}

void CPointCloudTool::FileOpen()
{
	std::string finame = CStringToolkit::convertUTF16toUTF8(_strLasFilePath);

	_lasreadopener.set_file_name(finame.c_str());
	_pLasreader = _lasreadopener.open();
	// 560747 导入点云：选择的文件可以是不存在的文件，且点击确定后软件崩溃 附图V1.6.0.31
	if (nullptr == _pLasreader)
		return;
	// 判断是否读取数字绿地las文件
	const CHAR* strLidar = "LiDAR360";
	const CHAR* strLidar1 = "LiGeoreference";
	const CHAR* strSoftware = _pLasreader->header.generating_software;
	if (0 == strcmp(strSoftware, strLidar) || 0 == strcmp(strSoftware, strLidar1))
		_bShuZiLvDiLas = true;
	const CHAR* strBooway = "BOOWAY";
	if (0 == strcmp(strSoftware, strBooway))
		_bBoowayLas = true;
}

bool CPointCloudTool::GetScanTime()
{
	CBnsProjectNode bnsProject = _pProjectNode;
	if (_pLasreader == nullptr || bnsProject.IsNull())
		return false;

	char buffer[100];
	int nYear = _pLasreader->header.file_creation_year;
	int nDay = _pLasreader->header.file_creation_day;
	if (nYear == 0 && nDay == 0)
	{
		std::string filePath = CStringToolkit::CStringToUTF8(_strLasFilePath);
		struct stat fileStat;
		if (stat(filePath.c_str(), &fileStat) == 0)
		{
			time_t t = fileStat.st_mtime; // 最后修改时间
			struct tm* tm_info = localtime(&t);
			if (tm_info)
			{
				strftime(buffer, sizeof(buffer), "%Y-%m-%d", tm_info);
			}
		}
	}
	else
	{
		// 使用标准库 mktime 将年和天数转换为具体日期
		struct tm tm_date = {};
		tm_date.tm_year = nYear - 1900;
		tm_date.tm_mon = 0;		// 一月
		tm_date.tm_mday = nDay; // 一年中的第几天 (1-366)
		tm_date.tm_hour = 0;
		tm_date.tm_min = 0;
		tm_date.tm_sec = 0;
		mktime(&tm_date); // 标准化，填充 tm_mon, tm_mday 等

		strftime(buffer, sizeof(buffer), "%Y-%m-%d", &tm_date);
	}

	std::string strT = std::string(buffer);
	CString strTime = CStringToolkit::UTF8ToCString(strT);
	bnsProject.SetScanTime(strTime);
	return true;
}

bool CPointCloudTool::GetBoundingBox(size_t& nPointCount,
									 double& dMaxX,
									 double& dMaxY,
									 double& dMaxZ,
									 double& dMinX,
									 double& dMinY,
									 double& dMinZ)
{
	if (_pLasreader == nullptr)
		return false;

	nPointCount = _pLasreader->header.number_of_point_records;
	dMaxX = _pLasreader->header.max_x;
	dMaxY = _pLasreader->header.max_y;
	dMaxZ = _pLasreader->header.max_z;
	dMinX = _pLasreader->header.min_x;
	dMinY = _pLasreader->header.min_y;
	dMinZ = _pLasreader->header.min_z;

	// 确保点云Z大于0
	if (dMinZ < 0.0)
	{
		_dOffSetZ = fabs(dMinZ);
		dMaxZ = dMaxZ + _dOffSetZ;
		dMinZ = 0.0; // 等价于dMinZ = dMinZ + _dOffSetZ
	}
	return true;
}


std::vector<std::pair<CString, osg::ref_ptr<CPointCloudpagedLod>>> CPointCloudTool::CutLasFile(
	const osg::Vec3d* pOffSet /* = nullptr*/)
{
}


pc::data::CModelNodePtr CPointCloudTool::Las2Osgb()
{
	double dLasMax_x = 0;
	double dLasMax_y = 0;
	double dLasMax_z = 0;
	double dLasMin_x = 0;
	double dLasMin_y = 0;
	double dLasMin_z = 0;
	size_t nPointCount = 0;
	GetBoundingBox(nPointCount, dLasMax_x, dLasMax_y, dLasMax_z, dLasMin_x, dLasMin_y, dLasMin_z);
	if (nPointCount == 0)
		return nullptr;
	if (isinf(dLasMax_x) || isinf(dLasMax_y) || isinf(dLasMax_z) || isinf(dLasMin_x) ||
		isinf(dLasMin_y) || isinf(dLasMin_z))
		return nullptr;

	double offsetX = (dLasMin_x + dLasMax_x) / 2;
	double offsetY = (dLasMin_y + dLasMax_y) / 2;
	// 同获取块最小值向下取整处理
	int nOffsetX = floor(offsetX);
	int nOffsetY = floor(offsetY);
	double dX = dLasMax_x - dLasMin_x;
	double dY = dLasMax_y - dLasMin_y;

	_centerPoint = osg::Vec3d(nOffsetX, nOffsetY, 0.0);
	_dAllLasRadius = dX > dY ? dX : dY;
	_dAllLasRadius = ceil(_dAllLasRadius / PieceTwoSize) * PieceTwoSize;
	_dLasCenterHight = (dLasMin_z + dLasMax_z) / 2;

	double dHight = dLasMax_z - dLasMin_z;
	double dMin_Z = dLasMin_z;

	double Max_x = GetAreaValue(dLasMax_x, _centerPoint.x() + PieceMinSize, true);
	double Max_y = GetAreaValue(dLasMax_y, _centerPoint.y() + PieceMinSize, true);
	double Max_z = GetAreaValue(dLasMax_z, _centerPoint.z() + PieceMinSize, true);
	double Min_x = GetAreaValue(dLasMin_x, _centerPoint.x(), false);
	double Min_y = GetAreaValue(dLasMin_y, _centerPoint.y(), false);
	double Min_z = GetAreaValue(dLasMin_z, _centerPoint.z(), false);


	_nStartX = floor(Min_x);
	_nStartY = floor(Min_y);
	_nStartZ = floor(Min_z);
	CreateLevelInfo(Max_x, Min_x, Max_y, Min_y, Max_z, Min_z);


	// 当前解析las信息根节点
	d3s::share_ptr<LasOsg::PieceInfo> currentPieceRoot = CreateLasPieceInfoPtr(_strLasFilePath);
	osg::Vec3d minPoint(dLasMin_x - _centerPoint.x(), dLasMin_y - _centerPoint.y(), dLasMin_z);
	osg::Vec3d maxPoint(dLasMax_x - _centerPoint.x(), dLasMax_y - _centerPoint.y(), dLasMax_z);
	currentPieceRoot->minPoint = minPoint;
	currentPieceRoot->maxPoint = maxPoint;
	CreateLasAllPiece(currentPieceRoot, Max_x, Max_y, Max_z, Min_x, Min_y, Min_z);

	ProcessPointToPiece();

	return BuildAllPageLod(currentPieceRoot);
}

double CPointCloudTool::GetAreaValue(double dValue, double dCentreValue, bool bMax)
{
	double dNewValue = 0.0;
	if (bMax)
	{
		if (dCentreValue >= dValue)
		{
			dNewValue = dCentreValue;
			while (dNewValue > dValue)
			{
				dNewValue -= PieceMinSize;
			}
			return (dNewValue + PieceMinSize);
		}
		else
		{
			dNewValue = dCentreValue;
			while (dValue > dNewValue)
			{
				dNewValue += PieceMinSize;
			}
			return dNewValue;
		}
	}
	else
	{
		if (dCentreValue <= dValue)
		{
			dNewValue = dCentreValue;
			while (dValue > dNewValue)
			{
				dNewValue += PieceMinSize;
			}
			return (dNewValue - PieceMinSize);
		}
		else
		{
			dNewValue = dCentreValue;
			while (dNewValue > dValue)
			{
				dNewValue -= PieceMinSize;
			}
			return dNewValue;
		}
	}
	return dNewValue;
}

void CPointCloudTool::CreateLevelInfo(double Max_x,
									  double Min_x,
									  double Max_y,
									  double Min_y,
									  double Max_z,
									  double Min_z)
{
	// 根据xyz划分，最小级为250，其次为500，再为1000, 粗略层级2000
	_vecLevelInfo.clear();
	for (int nIdx = 0; nIdx < nLevel; ++nIdx)
	{
		float fPieceSize =
			nIdx == 0 ? PieceMinSize : _vecLevelInfo.at(nIdx - 1)._fPieceSize * 2; // 计算划分大小
		SLevelInfo tmpLevelInfo;
		tmpLevelInfo._fPieceSize = fPieceSize;
		tmpLevelInfo._nX500Size = ceil((Max_x - Min_x) / fPieceSize);
		tmpLevelInfo._nY500Size = ceil((Max_y - Min_y) / fPieceSize);
		tmpLevelInfo._nZ500Size = ceil((Max_z - Min_z) / fPieceSize);
		int nAll500size =
			tmpLevelInfo._nX500Size * tmpLevelInfo._nY500Size * tmpLevelInfo._nZ500Size;
		tmpLevelInfo._PieceInfoVec.resize(nAll500size);
		tmpLevelInfo._mtx = std::make_shared<tbb::mutex>();

		_vecLevelInfo.emplace_back(tmpLevelInfo);
	}
	_vecLevelInfo.at(EPageLodLevel::eZeroLevelPageLod)._fPageLodMinPixel =
		PAGEDLOG_0_MIN_PIXEL_SIZE;
	_vecLevelInfo.at(EPageLodLevel::eZeroLevelPageLod)._fPageLodMaxPixel = FLT_MAX;
	_vecLevelInfo.at(EPageLodLevel::eOneLevelPageLod)._fPageLodMinPixel = PAGEDLOG_1_MIN_PIXEL_SIZE;
	_vecLevelInfo.at(EPageLodLevel::eOneLevelPageLod)._fPageLodMaxPixel = FLT_MAX;
	_vecLevelInfo.at(EPageLodLevel::eTwoLevelPageLod)._fPageLodMinPixel = PAGEDLOG_2_MIN_PIXEL_SIZE;
	_vecLevelInfo.at(EPageLodLevel::eTwoLevelPageLod)._fPageLodMaxPixel = FLT_MAX;
	_vecLevelInfo.at(EPageLodLevel::eWideLevelPageLod)._fPageLodMinPixel =
		PAGEDLOG_3_MIN_PIXEL_SIZE;
	_vecLevelInfo.at(EPageLodLevel::eWideLevelPageLod)._fPageLodMaxPixel = FLT_MAX;
}

d3s::share_ptr<LasOsg::PieceInfo> CPointCloudTool::CreateLasPieceInfoPtr(const CString& strFilename)
{
	// 使用Las的文件名称作为第一层节点名称，按Las文件名称分类
	CString strBaseName = CFileToolkit::GetFileBaseName(strFilename);
	// 540890 检查文件夹是否存在，避免同名覆盖
	CString strTmpName = strBaseName;
	for (int nIndex = 1; CFileToolkit::DirectoryExist(_strOutPath + strTmpName); ++nIndex)
	{
		strTmpName = strBaseName + L" - " + CStringToolkit::IntToStr(nIndex);
	}
	strBaseName = strTmpName;
	d3s::share_ptr<LasOsg::PieceInfo> thisPiece = new LasOsg::PieceInfo();
	thisPiece->strRelativePath = strBaseName + L"/";
	thisPiece->strOsgRePath = strBaseName + L"/Tile_+000_+000/";
	thisPiece->strLasName = strBaseName;
	thisPiece->strName = L"Tile_+000_+000";
	thisPiece->strOgbPath = L"Tile_+000_+000";
	thisPiece->nScale = 1.0 * 2;
	thisPiece->nWidth = PieceLasSize * 2;
	thisPiece->nLevel = EPageLodLevel::eTitlePageLod;
	thisPiece->nX = _nStartX;
	thisPiece->nY = _nStartY;
	thisPiece->nZ = _nStartZ;
	thisPiece->nHeight = PieceMinSize;
	CloudPtr newCloud = CloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>());
	thisPiece->cloudPt = newCloud;
	thisPiece->nIdex = 0;

	CString strOsgPath = _strOutPath + thisPiece->strOsgRePath;
	if (!CFileToolkit::DirectoryExist(strOsgPath))
	{
		CFileToolkit::CreateDirectory(strOsgPath);
	}

	return thisPiece;
}

void CPointCloudTool::CreateLasAllPiece(d3s::share_ptr<LasOsg::PieceInfo> thisPiece,
										double Max_x,
										double Max_y,
										double Max_z,
										double Min_x,
										double Min_y,
										double Min_z)
{
	if (nullptr == thisPiece || thisPiece->nLevel == 0)
		return;
	// 默认以2层级2000,1层级1000，0层级500划分，起始3层级为4000
	int nX2000 = ceil((Max_x - Min_x) / (thisPiece->nWidth / 2));
	int nY2000 = ceil((Max_y - Min_y) / (thisPiece->nWidth / 2));
	int nZ2000 = ceil((Max_z - Min_z) / (thisPiece->nWidth / 2));
	for (int x = 0; x < nX2000; ++x)
	{
		for (int y = 0; y < nY2000; ++y)
		{
			for (int z = 0; z < nZ2000; ++z)
			{
				int nMinKeyX = floor(Min_x) + x * thisPiece->nWidth / 2;
				int nMinKeyY = floor(Min_y) + y * thisPiece->nWidth / 2;
				int nMinKeyZ = floor(Min_z) + z * thisPiece->nWidth / 2;
				int nMaxKeyX = nMinKeyX + thisPiece->nWidth / 2;
				int nMaxKeyY = nMinKeyY + thisPiece->nWidth / 2;
				int nMaxKeyZ = nMinKeyZ + thisPiece->nWidth / 2;
				nMaxKeyX = Max_x > nMaxKeyX ? nMaxKeyX : Max_x;
				nMaxKeyY = Max_y > nMaxKeyY ? nMaxKeyY : Max_y;
				nMaxKeyZ = Max_z > nMaxKeyZ ? nMaxKeyZ : Max_z;
				// 使用此分块的最小X和Y值作为此分块的名称
				CString strPieceName = L"";
				strPieceName.Format(L"%d#%d#%d", nMinKeyX, nMinKeyY, nMinKeyZ);

				// 获取当前分快下用于保存此点位信息的小分块
				d3s::share_ptr<LasOsg::PieceInfo> pNewPiece = nullptr;
				CString strPiecePath = thisPiece->strRelativePath + strPieceName + L"/";

				pNewPiece = new LasOsg::PieceInfo();
				pNewPiece->strLasName = thisPiece->strLasName;
				pNewPiece->strName = strPieceName;
				pNewPiece->strRelativePath = strPiecePath;
				pNewPiece->nX = nMinKeyX;
				pNewPiece->nY = nMinKeyY;
				pNewPiece->nZ = nMinKeyZ;
				pNewPiece->nWidth = thisPiece->nWidth / 2;
				osg::Vec3d minPoint(nMinKeyX - _centerPoint.x(),
									nMinKeyY - _centerPoint.y(),
									nMinKeyZ);
				osg::Vec3d maxPoint(minPoint.x() + pNewPiece->nWidth,
									minPoint.y() + pNewPiece->nWidth,
									minPoint.z() + pNewPiece->nWidth);
				pNewPiece->minPoint = minPoint;
				pNewPiece->maxPoint = maxPoint;
				pNewPiece->nScale = thisPiece->nScale / 2;
				pNewPiece->pFatherPiece = thisPiece.get();
				pNewPiece->nLevel = thisPiece->nLevel - 1;
				pNewPiece->nHeight = PieceMinSize;
				pNewPiece->nIdex = 1;
				pNewPiece->strOsgRePath = thisPiece->strOsgRePath;
				CloudPtr newCloud = CloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>());
				pNewPiece->cloudPt = newCloud;
				CString strExName = L"";
				strExName.Format(L"%s#%d",
								 CStringToolkit::CStringToUTF8(pNewPiece->strName).c_str(),
								 pNewPiece->nLevel);
				pNewPiece->strOgbPath = strExName;
				thisPiece->ChildPieceVec.push_back(pNewPiece);

				// 将切片写入对应的层级信息中
				{
					int nKeyX = (nMinKeyX + nMaxKeyX) / 2;
					int nKeyY = (nMinKeyY + nMaxKeyY) / 2;
					int nKeyZ = (nMinKeyZ + nMaxKeyZ) / 2;
					int nX500Size = ceil((nKeyX - _nStartX) / (float)pNewPiece->nWidth);
					int nY500Size = ceil((nKeyY - _nStartY) / (float)pNewPiece->nWidth);
					int nZ500Size = ceil((nKeyZ - _nStartZ) / (float)pNewPiece->nWidth);
					nX500Size = nX500Size > 0 ? nX500Size : 1;
					nY500Size = nY500Size > 0 ? nY500Size : 1;
					nZ500Size = nZ500Size > 0 ? nZ500Size : 1;

					// 先X再Y,x作行，y作列
					SLevelInfo& curLevelInfo = _vecLevelInfo.at(pNewPiece->nLevel);
					size_t nsize = (size_t)((nZ500Size - 1) * (curLevelInfo._nX500Size *
															   curLevelInfo._nY500Size) +
											(nY500Size - 1) * curLevelInfo._nX500Size + nX500Size);
					if (nsize > curLevelInfo._PieceInfoVec.size())
					{
						// ASSERT(FALSE && L"切片信息关联层级发生错误！");
						return;
					}
					curLevelInfo._PieceInfoVec.at(nsize - 1) = pNewPiece;
				}

				// 递归创建所有层级信息
				CreateLasAllPiece(pNewPiece,
								  nMaxKeyX,
								  nMaxKeyY,
								  nMaxKeyZ,
								  nMinKeyX,
								  nMinKeyY,
								  nMinKeyZ);
			}
		}
	}
}

// 点到切片处理线程
class CPointToPieceProcessThread : public OpenThreads::Thread
{
public:
	CPointToPieceProcessThread(CPointCloudTool* pLasFileProcess)
		: _curArrPoint(nullptr), _pLasFileProcess(pLasFileProcess)
	{
		// 初始化互斥锁和条件变量
		pthread_mutex_init(&_mutex, nullptr);
		pthread_cond_init(&_condEvent, nullptr);
		pthread_mutex_init(&_stopMutex, nullptr);
		pthread_cond_init(&_condStopEvent, nullptr);

		// 初始化标志
		_eventFlag = false;
		_stopEventFlag = false;

		_bRun = true;
	};

	~CPointToPieceProcessThread()
	{
		WaitComplete();

		// 销毁互斥锁和条件变量
		pthread_mutex_destroy(&_mutex);
		pthread_cond_destroy(&_condEvent);
		pthread_mutex_destroy(&_stopMutex);
		pthread_cond_destroy(&_condStopEvent);
	};

	virtual void run(void) override
	{
		while (_bRun)
		{
			// 设置停止事件（通知主线程已暂停）
			pthread_mutex_lock(&_stopMutex);
			_stopEventFlag = true;
			pthread_cond_signal(&_condStopEvent);
			pthread_mutex_unlock(&_stopMutex);

			// 等待事件（等待主线程设置新数据）
			pthread_mutex_lock(&_mutex);
			while (!_eventFlag && _bRun)
			{
				pthread_cond_wait(&_condEvent, &_mutex);
			}

			// 检查是否需要退出
			if (!_bRun)
			{
				pthread_mutex_unlock(&_mutex);
				break; // 退出线程
			}

			// 重置事件标志
			_eventFlag = false;
			pthread_mutex_unlock(&_mutex);

			// 处理数据
			ProcessPointData();
		}
	};

	void ProcessPointData()
	{
		// 计算当前数组启用的多线程数
		I64 nTmpPointCount = _nPointCount % MAX_MEMORY_POINT_NUM;
		nTmpPointCount = (0 == nTmpPointCount) ? MAX_MEMORY_POINT_NUM : nTmpPointCount;
		const int nTbbCount = std::ceil(nTmpPointCount / (float)nSpliteSize);
		// CTbbPointPiece parallel(_pLasFileProcess, _curArrPoint, nTmpPointCount);
		// CTBBParallel::For(0, nTbbCount, parallel);

		for (int i = 0; i < nTbbCount; i++)
		{
			size_t nStartPointIndex = nSpliteSize * i;
			size_t nSpliteCount = nSpliteSize;
			if (nStartPointIndex + nSpliteCount > nTmpPointCount)
			{
				nSpliteCount = nTmpPointCount - nStartPointIndex;
			}
			for (size_t nIndex = 0; nIndex < nSpliteCount; ++nIndex)
			{
				size_t nArrIndex = nStartPointIndex + nIndex;
				_pLasFileProcess->SavePoinToPieceInfo(_curArrPoint[nArrIndex], nArrIndex);
			}
		}

		// 清空数组
		memset(_curArrPoint, 0, sizeof(pcl::PointXYZRGBA) * MAX_MEMORY_POINT_NUM);

		// 重置事件标志
	}

	void SetArrPoint(pcl::PointXYZRGBA* pArrPoint, I64 nPointCount)
	{
		// 等待停止事件（等待线程暂停）
		pthread_mutex_lock(&_stopMutex);
		while (!_stopEventFlag && _bRun)
		{
			pthread_cond_wait(&_condStopEvent, &_stopMutex);
		}
		_stopEventFlag = false; // 重置停止事件标志
		pthread_mutex_unlock(&_stopMutex);

		if (!_bRun)
		{
			return; // 如果线程正在关闭，直接返回
		}

		_curArrPoint = pArrPoint;
		_nPointCount = nPointCount;

		// 设置事件，唤醒线程
		pthread_mutex_lock(&_mutex);
		_eventFlag = true;
		pthread_cond_signal(&_condEvent);
		pthread_mutex_unlock(&_mutex);
	}

	void WaitComplete()
	{
		if (!_bRun)
		{
			return; // 已经停止
		}

		// 等待停止事件（等待线程暂停）
		pthread_mutex_lock(&_stopMutex);
		while (!_stopEventFlag && _bRun)
		{
			pthread_cond_wait(&_condStopEvent, &_stopMutex);
		}
		_stopEventFlag = false; // 重置停止事件标志
		pthread_mutex_unlock(&_stopMutex);

		// 设置停止标志
		_bRun = false;

		// 设置事件，唤醒线程
		pthread_mutex_lock(&_mutex);
		_eventFlag = true;
		pthread_cond_signal(&_condEvent);
		pthread_mutex_unlock(&_mutex);

		// 等待线程结束
		join();
	};

private:
	pcl::PointXYZRGBA* _curArrPoint;
	I64 _nPointCount; // 当前数组已读取点数量
	CPointCloudTool* _pLasFileProcess;

	// Linux 平台同步原语
	pthread_mutex_t _mutex;		   // 互斥锁，保护 _eventFlag
	pthread_cond_t _condEvent;	   // 条件变量，对应 _hEvent
	pthread_mutex_t _stopMutex;	   // 互斥锁，保护 _stopEventFlag
	pthread_cond_t _condStopEvent; // 条件变量，对应 _hStopEvent

	bool _eventFlag;	 // 事件标志
	bool _stopEventFlag; // 停止事件标志
	bool _bRun;			 // 线程是否继续运行
};


void CPointCloudTool::ProcessPointToPiece()
{
	// 遍历处理所有点
	auto start = std::chrono::steady_clock::now();

#ifndef USE_MULTI_THREAD
	// 遍历点云
	CPointToPieceProcessThread* pProcessThread = new CPointToPieceProcessThread(this);
	pProcessThread->start();

	I64 nPointCount = 0;
	I64 nArrPointIndex = 0;

	// 创建两片内存区域读取las文件信息
	pcl::PointXYZRGBA* arrPoint1 = new pcl::PointXYZRGBA[MAX_MEMORY_POINT_NUM];
	pcl::PointXYZRGBA* arrPoint2 = new pcl::PointXYZRGBA[MAX_MEMORY_POINT_NUM];
	pcl::PointXYZRGBA* arrPoint = arrPoint1;
	pcl::PointXYZRGBA point;

	while (ReadPoint(point, _centerPoint.x(), _centerPoint.y(), _centerPoint.z(), true))
	{
		nArrPointIndex = nPointCount % MAX_MEMORY_POINT_NUM; // 数组中的索引值

		if (nPointCount != 0 && nArrPointIndex == 0) // 第一次不进入
		{
			pProcessThread->SetArrPoint(arrPoint, nPointCount);

			// 点数组指针交换继续读取
			if (arrPoint == arrPoint1)
			{
				arrPoint = arrPoint2;
			}
			else
			{
				arrPoint = arrPoint1;
			}
		}
		arrPoint[nArrPointIndex] = point;
		++nPointCount;
	}

	// 数组的末尾处理
	pProcessThread->SetArrPoint(arrPoint, nPointCount);

	pProcessThread->WaitComplete();
	SAFE_DELETE(pProcessThread);

	// 释放数组
	delete[] arrPoint1;
	delete[] arrPoint2;
#else
	CSavePieceProcessThread* pProcessThread = new CSavePieceProcessThread(this);
	pProcessThread->start();

#ifndef USE_READ_BATCH_POINT
	I64 nPointCount = 0;
	I64 nArrPointIndex = 0;

	// 创建两片内存区域读取las文件信息
	pcl::PointXYZRGBA* arrPoint1 = new pcl::PointXYZRGBA[MAX_MEMORY_POINT_NUM];
	pcl::PointXYZRGBA* arrPoint2 = new pcl::PointXYZRGBA[MAX_MEMORY_POINT_NUM];
	pcl::PointXYZRGBA* arrPoint = arrPoint1;
	pcl::PointXYZRGBA point;

	while (ReadPoint(point, _centerPoint.x(), _centerPoint.y(), _centerPoint.z(), true))
	{
		nArrPointIndex = nPointCount % MAX_MEMORY_POINT_NUM; // 数组中的索引值

		if (nPointCount != 0 && nArrPointIndex == 0) // 第一次不进入
		{
			pProcessThread->SetArrPoint(arrPoint, nPointCount);

			// 点数组指针交换继续读取
			if (arrPoint == arrPoint1)
			{
				arrPoint = arrPoint2;
			}
			else
			{
				arrPoint = arrPoint1;
			}
		}
		arrPoint[nArrPointIndex] = point;
		++nPointCount;
	}

	// 数组的末尾处理
	pProcessThread->SetArrPoint(arrPoint, nPointCount);
#else
	I64 nPointCount = 0;
	const double centerX = _centerPoint.x();
	const double centerY = _centerPoint.y();
	const double centerZ = _centerPoint.z();

	// 双缓冲区
	pcl::PointXYZRGBA* arrPoint1 = new pcl::PointXYZRGBA[MAX_MEMORY_POINT_NUM];
	pcl::PointXYZRGBA* arrPoint2 = new pcl::PointXYZRGBA[MAX_MEMORY_POINT_NUM];
	pcl::PointXYZRGBA* arrWrite = arrPoint1; // 待写入的缓冲
	pcl::PointXYZRGBA* arrRead = arrPoint2;	 // 待读取填充的缓冲

	// 批量读取配置
	const size_t batchSize = 1000000; // 每批读取 100 万点
	size_t bufferOffset = 0;

	while (true)
	{
		// 批量读取点
		size_t readCount = ReadPoints(arrRead + bufferOffset,
									  MAX_MEMORY_POINT_NUM - bufferOffset,
									  centerX,
									  centerY,
									  centerZ,
									  true);
		bufferOffset += readCount;

		// 缓冲区满或读取完毕，提交处理
		if (bufferOffset >= MAX_MEMORY_POINT_NUM || readCount == 0)
		{
			if (bufferOffset > 0)
			{
				pProcessThread->SetArrPoint(arrRead, nPointCount + bufferOffset);
				nPointCount += bufferOffset;
				bufferOffset = 0;

				// 交换缓冲区
				std::swap(arrRead, arrWrite);
			}

			if (readCount == 0)
				break;
		}
	}
#endif
	pProcessThread->WaitComplete();
	SAFE_DELETE(pProcessThread);

	delete[] arrPoint1;
	delete[] arrPoint2;
#endif

	auto end = std::chrono::steady_clock::now();
	auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "PointToPieceProcess() cost: " << elapsed_ms.count() << " ms" << std::endl;
}

bool CPointCloudTool::ReadPoint(pcl::PointXYZRGBA& point,
								const double& dCenterPointX,
								const double& dCenterPointY,
								const double& dCenterPointZ,
								const bool& bReadClassify)
{
	if (_pLasreader == nullptr)
		return false;
	if (!_pLasreader->read_point())
		return false;

	LASpoint& lasPoint = _pLasreader->point;

	int r, g, b;
	if (_bShuZiLvDiLas)
	{
		r = lasPoint.get_R();
		g = lasPoint.get_G();
		b = lasPoint.get_B();
	}
	else
	{
		r = lasPoint.rgb[0] >> LAS_TINY_RIGHT_MOVE;
		g = lasPoint.rgb[1] >> LAS_TINY_RIGHT_MOVE;
		b = lasPoint.rgb[2] >> LAS_TINY_RIGHT_MOVE;
	}

	int a = 255;
	double X = lasPoint.get_x();
	double Y = lasPoint.get_y();
	// 确保点云Z大于0
	double Z = lasPoint.get_z() + _dOffSetZ;

	point.x = X - dCenterPointX;
	point.y = Y - dCenterPointY;
	point.z = Z;
	point.r = r;
	point.g = g;
	point.b = b;
	point.a = a;
	// 558136 类别设置：新增的类别超过3种，第3种及以上的类别无法单独显示在视图中 附图V1.6.0.19
	if (bReadClassify)
	{
		if (_bBoowayLas)
			point.a = lasPoint.get_user_data();
		else
		{
			point.a = lasPoint.get_classification();
			if (_typeMapping.end() != _typeMapping.find(point.a))
				point.a = _typeMapping[point.a];
		}
	}
	return true;
}

size_t CPointCloudTool::ReadPoints(pcl::PointXYZRGBA* points,
								   size_t maxCount,
								   const double& dCenterPointX,
								   const double& dCenterPointY,
								   const double& dCenterPointZ,
								   const bool& bReadClassify)
{
	if (_pLasreader == nullptr || points == nullptr || maxCount == 0)
		return 0;

	size_t readCount = 0;
	const double dOffSetZ = _dOffSetZ;

	// 预取分类映射，避免每次查找
	const bool bBoowayLas = _bBoowayLas;
	const bool bShuZiLvDiLas = _bShuZiLvDiLas;
	const auto& typeMapping = _typeMapping;

	while (readCount < maxCount && _pLasreader->read_point())
	{
		LASpoint& lasPoint = _pLasreader->point;
		pcl::PointXYZRGBA& point = points[readCount];

		// RGB 处理
		if (bShuZiLvDiLas)
		{
			point.r = lasPoint.get_R();
			point.g = lasPoint.get_G();
			point.b = lasPoint.get_B();
		}
		else
		{
			point.r = lasPoint.rgb[0] >> LAS_TINY_RIGHT_MOVE;
			point.g = lasPoint.rgb[1] >> LAS_TINY_RIGHT_MOVE;
			point.b = lasPoint.rgb[2] >> LAS_TINY_RIGHT_MOVE;
		}

		// 坐标处理
		point.x = lasPoint.get_x() - dCenterPointX;
		point.y = lasPoint.get_y() - dCenterPointY;
		point.z = lasPoint.get_z() + dOffSetZ;

		// 分类处理
		point.a = 255;
		if (bReadClassify)
		{
			if (bBoowayLas)
			{
				point.a = lasPoint.get_user_data();
			}
			else
			{
				point.a = lasPoint.get_classification();
				auto it = typeMapping.find(point.a);
				if (it != typeMapping.end())
					point.a = it->second;
			}
		}

		++readCount;
	}

	return readCount;
}

void CPointCloudTool::SavePoinToPieceInfo(pcl::PointXYZRGBA& pointXYZRGBA, const I64& curPointIndex)
{
	// 根据当前点索引计算放入层级
	int nLevelIndex = 0; // 默认放入0级精细层
	if (curPointIndex % 100 == 0)
	{
		nLevelIndex = EPageLodLevel::eWideLevelPageLod;
	}
	else if ((curPointIndex % 25 == 0) && (curPointIndex % 100 != 0))
	{
		nLevelIndex = EPageLodLevel::eTwoLevelPageLod;
	}
	else if ((curPointIndex % 5 == 0) && (curPointIndex % 25 != 0) && (curPointIndex % 100 != 0))
	{
		nLevelIndex = EPageLodLevel::eOneLevelPageLod;
	}

	SLevelInfo& tmpLevelInfo = _vecLevelInfo.at(nLevelIndex);

	double dX = pointXYZRGBA.x + _centerPoint.x() - _nStartX;
	double dY = pointXYZRGBA.y + _centerPoint.y() - _nStartY;
	double dZ = pointXYZRGBA.z - _nStartZ;
	float fPieceSize = tmpLevelInfo._fPieceSize;
	int nX500Size = ceil(dX / fPieceSize);
	int nY500Size = ceil(dY / fPieceSize);
	int nZ500Size = ceil(dZ / fPieceSize);

	// 过滤异常点
	if (nX500Size < EPSILON || nY500Size < EPSILON || nZ500Size < EPSILON)
	{
		return;
	}
	nX500Size = nX500Size == 0 ? 1 : nX500Size;
	nY500Size = nY500Size == 0 ? 1 : nY500Size;
	nZ500Size = nZ500Size == 0 ? 1 : nZ500Size;
	// x作行，y作列
	size_t nsize = (size_t)((nZ500Size - 1) * (tmpLevelInfo._nX500Size * tmpLevelInfo._nY500Size) +
							tmpLevelInfo._nX500Size * (nY500Size - 1) + nX500Size);
	if (nsize < 1 || nsize > tmpLevelInfo._PieceInfoVec.size())
	{
		return;
	}

	d3s::share_ptr<LasOsg::PieceInfo> pPiece = tmpLevelInfo._PieceInfoVec.at(nsize - 1);
	if (nullptr == pPiece)
	{
		// ASSERT(FALSE && L"层级切片获取出错!");
		return;
	}
	else
	{
		tbb::mutex::scoped_lock lock(pPiece->mtx);
		pPiece->cloudPt->push_back(pointXYZRGBA);

		// 第三层级点不执行每满50万构建一个pagedlod节点
		if (pPiece->cloudPt->size() == PIECE_SAVE_NUM &&
			nLevelIndex != EPageLodLevel::eWideLevelPageLod)
		{
			WritePCDFile(pPiece.get());

			pPiece->cloudPt = CloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>());
			pPiece->cloudPt->reserve(RESERVE);
		}
	}
}

void CPointCloudTool::WritePCDFile(d3s::share_ptr<LasOsg::PieceInfo> pPiece)
{
	if (nullptr == pPiece || nullptr == pPiece->cloudPt || pPiece->cloudPt->empty())
		return;

	// 预转换路径字符串（只转换一次）
	std::string strOgbPath = CStringToolkit::CStringToUTF8(pPiece->strOgbPath);
	std::string strOsgRePath = CStringToolkit::CStringToUTF8(pPiece->strOsgRePath);
	std::string strOutPath = CStringToolkit::CStringToUTF8(_strOutPath);

	// 直接用 snprintf 构建 PCD 文件名
	const int nIndex = pPiece->nIdex++;
	char pcdName[512];
	snprintf(pcdName, sizeof(pcdName), "%d#%s#%d.pcd", _nLasIdex, strOgbPath.c_str(), nIndex);

	// 拼接完整路径
	std::string finame;
	finame.reserve(strOutPath.size() + strOsgRePath.size() + strlen(pcdName) + 1);
	finame = strOutPath + strOsgRePath + pcdName;

#ifndef USE_MULTI_THREAD
	pcl::io::savePCDFileBinary(finame, *pPiece->cloudPt);
#else
	_writeFileThread->AddTask(finame, pPiece->cloudPt);
#endif

	pPiece->allPCDFilePath.emplace_back(std::move(finame));
}

pc::data::CModelNodePtr CPointCloudTool::BuildAllPageLod(d3s::share_ptr<LasOsg::PieceInfo> pPiece)
{
	auto start = std::chrono::steady_clock::now();

	// 构建0-2层的pagelod信息
	std::vector<d3s::share_ptr<LasOsg::PieceInfo>> tmpPieceInfoVec;
	tmpPieceInfoVec.reserve(_vecLevelInfo.front()._PieceInfoVec.size() * 2);
	int nSize = (int)_vecLevelInfo.size() - 1;

	for (int nIndex = 0; nIndex < nSize; ++nIndex)
	{
		tmpPieceInfoVec.insert(tmpPieceInfoVec.end(),
							   _vecLevelInfo[nIndex]._PieceInfoVec.begin(),
							   _vecLevelInfo[nIndex]._PieceInfoVec.end());
	}

	// 确保pcd文件已写入完毕
	_writeFileThread->WaitIdle();

	size_t pieceInfoSize = tmpPieceInfoVec.size();

#ifndef USE_MULTI_THREAD
	for (size_t i = 0; i < pieceInfoSize; i++)
	{
		auto pPieceInfo = tmpPieceInfoVec.at(i).get();
		if (nullptr == pPieceInfo)
			continue;

		BuildPageLodInfo(pPieceInfo);
	}
#else
	tbb::parallel_for(tbb::blocked_range<size_t>(0, pieceInfoSize),
					  [&](const tbb::blocked_range<size_t>& r) {
						  for (size_t i = r.begin(); i != r.end(); ++i)
						  {
							  auto piceInfo = tmpPieceInfoVec.at(i).get();

							  if (nullptr == piceInfo)
								  continue;

							  BuildPageLodInfo(piceInfo);
						  }
					  });
#endif

	d3s::CLog::Info("PCL转OSG节点完成，开始构建粗糙层级信息");

	// 构建粗略层(3)级的PageLod
	pc::data::CModelNodePtr pWidePageLod =
		new pc::data::CModelNode(EProjectNodeType::eBnsPointCloudLodNode);

	// 将各层级pagelod合并
	std::vector<d3s::share_ptr<LasOsg::PieceInfo>>& vecPieceInfo =
		_vecLevelInfo.at(eWideLevelPageLod)._PieceInfoVec;

	// int nIndex = 0;
	for (auto iterPieceInfo : vecPieceInfo)
	{
		for (auto iterPagedLod : iterPieceInfo->childpagedLodPrtvec)
		{
			if (nullptr == iterPagedLod)
				continue;
			// pWidePageLod->addChild(iterPagedLod);
			pWidePageLod->InsertNode(iterPagedLod, -1);
			// pWidePageLod->setRange(nIndex++, 0, FLT_MAX);
		}
		for (auto iterChildPieceInfo : iterPieceInfo->ChildPieceVec)
		{
			BuildPageLodTree(iterChildPieceInfo.get());
		}
	}

	// 构建粗略层(3)级的PageLod
	BuildWidePageLod(pPiece, pWidePageLod);

	d3s::CLog::Info(L"粗糙层级构建完毕");

	auto end = std::chrono::steady_clock::now();
	auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "BuildAllPageLod() cost: " << elapsed_ms.count() << " ms" << std::endl;

	return pWidePageLod;
}



void CPointCloudTool::BuildPageLodInfo(d3s::share_ptr<LasOsg::PieceInfo> pPiece)
{
	if (nullptr == pPiece || pPiece->pFatherPiece == nullptr)
		return;

	if (0 == pPiece->cloudPt->size() && 0 == pPiece->allPCDFilePath.size())
		return;

	// 加载所有的点云信息
	{
		pPiece->cloudPt->reserve(pPiece->cloudPt->size() +
								 pPiece->allPCDFilePath.size() * PIECE_SAVE_NUM);
		CloudPtr tmpCloudPt(new pcl::PointCloud<pcl::PointXYZRGBA>());
		tmpCloudPt->reserve(PIECE_SAVE_NUM);
		for (auto iterFileName : pPiece->allPCDFilePath)
		{
			pcl::io::loadPCDFile(iterFileName, *tmpCloudPt);
			pPiece->cloudPt->insert(pPiece->cloudPt->end(), tmpCloudPt->begin(), tmpCloudPt->end());
		}
	}

	pcl::octree::OctreePointCloud<pcl::PointXYZRGBA>::Ptr pOcPointCloud(
		new pcl::octree::OctreePointCloud<pcl::PointXYZRGBA>(pPiece->nWidth / 2.0));
	pOcPointCloud->setInputCloud(pPiece->cloudPt);
	pOcPointCloud->defineBoundingBox();
	pOcPointCloud->addPointsFromInputCloud();

	const SLevelInfo& levelInfo = _vecLevelInfo.at(pPiece->nLevel);

	pPiece->nIdex = 0;
	for (auto iterOcPC = pOcPointCloud->leaf_begin(); iterOcPC != pOcPointCloud->leaf_end();
		 ++iterOcPC)
	{
		int nIndex = pPiece->nIdex;
		pPiece->nIdex = pPiece->nIdex + 1;
		CString strOgb;
		strOgb.Format(L"%d#%s#%d.ive",
					  _nLasIdex,
					  CStringToolkit::CStringToUTF8(pPiece->strOgbPath).c_str(),
					  nIndex);
		std::string sName = CStringToolkit::CStringToUTF8(strOgb);
		double centerX = (double)(pPiece->nX + pPiece->nWidth / 2 - _centerPoint.x());
		double centerY = (double)(pPiece->nY + pPiece->nWidth / 2 - _centerPoint.y());
		double centerZ = (double)(pPiece->nZ + pPiece->nWidth / 2 - _centerPoint.z());

		pc::data::CModelNodePtr pNewLodNode =
			new pc::data::CModelNode(EProjectNodeType::eBnsPointCloudLodNode);
		CBnsPointCloudNode pPageLod(pNewLodNode);
		// pPageLod->setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
		// pPageLod->setRange(0, levelInfo._fPageLodMinPixel, levelInfo._fPageLodMaxPixel);
		pPageLod.setCenter(osg::Vec3(centerX, centerY, centerZ));
		// 设置模型矩阵uniform（同中心点，Z值为0）
		// osg::ref_ptr<osg::StateSet> pStateSet = pPageLod->getOrCreateStateSet();
		// osg::ref_ptr<osg::Uniform> pModelMatrixUniform =
		// pStateSet->getOrCreateUniform(pc::data::S_STRING_MODEL_MATRIX_UNIFORM,
		// osg::Uniform::Type::FLOAT_MAT4);
		// pModelMatrixUniform->set((osg::Matrixf)osg::Matrix::translate(osg::Vec3(centerX, centerY,
		// 0.0f)));
		osg::BoundingBox boundingBox(pPiece->minPoint, pPiece->maxPoint);
		pPageLod.SetModelBoundingBox(boundingBox);
		pPageLod.setRadius(pPiece->nWidth);

		CString strFileWork = _strOutPath + pPiece->strOsgRePath;
		// std::string sFileWork = CStringToolkit::CStringToUTF8(strFileWork);
		pPageLod.setDatabasePath(strFileWork);
		// CString strId = CLibToolkit::CreateGuid();
		// pPageLod->SetIDKey(strId);
		CString strOgbPath = pPiece->strOsgRePath + strOgb;
		CString strActFile = _strOutPath + strOgbPath;
		std::string finame = CStringToolkit::CStringToUTF8(strActFile);
		// pageLod自身为0层级FileName
		pPageLod.setFileName(strOgb);
		// pPageLod->SetNewFileName(sName);
		// 添加到写文件线程,会加一个偏移矩阵
		osg::Vec3d pieceCentet = boundingBox.center();
		pieceCentet.z() = 0; // 高度不偏移


		CPointCloudTool::WriteTask writeTask;
		// writeTask.eFileType = CLasWriteFileThread::eOSGFile;
		writeTask.cloudPrt = pPiece->cloudPt;
		writeTask.strFilePath = finame;
		writeTask.center = pieceCentet;
		writeTask.pPageLod = pNewLodNode;
		writeTask.pointIndices = boost::shared_ptr<std::vector<int>>(new std::vector<int>);
		iterOcPC.getLeafContainer().getPointIndices(*writeTask.pointIndices);
		int nPointCnt = writeTask.pointIndices->size();
		pPageLod.SetPointsNum(nPointCnt);
		ExcuteWriteOsgTask(writeTask);
		{
			tbb::mutex::scoped_lock lock(pPiece->mtx);
			pPiece->pFatherPiece->childpagedLodPrtvec.push_back(pPageLod);
		}


		pPiece->pPieceInfoPagedLod = pNewLodNode;
		WriteMoudleInfo(boundingBox,
						strOgb,
						pNewLodNode->GetId(),
						nPointCnt,
						(EPageLodLevel)(pPiece->nLevel));
	}

	// 释放内存
	pPiece->cloudPt = nullptr;

	// 释放文件
	for (auto iterFileName : pPiece->allPCDFilePath)
	{
		CFileToolkit::DeleteToRecycle(CStringToolkit::UTF8ToCString(iterFileName.c_str()));
	}
}


void CPointCloudTool::ExcuteWriteOsgTask(const CPointCloudTool::WriteTask& writeTask)
{
	osg::ref_ptr<osg::MatrixTransform> pNewOsgNode = PointCloud2Osgb(writeTask);
	osg::ComputeBoundsVisitor computerBound;
	pNewOsgNode->accept(computerBound);
	CBnsPointCloudNode bnsPcNode(writeTask.pPageLod);
	bnsPcNode.SetRealBoundingBox(computerBound.getBoundingBox());
	CString strFilePath = CStringToolkit::UTF8ToCString(writeTask.strFilePath.c_str());
	int nCount = CStringToolkit::CountWord(strFilePath, L'#');
	int nLevel = 3;
	if (nCount == 6)
	{
		nLevel = CStringToolkit::StrToInt(CStringToolkit::ReadWord(strFilePath, 5, L'#'));
	}

	// 处理节点展现时，包围球不精确，导致周边大量镂空区域被加载的问题
	bnsPcNode.setRadius(bnsPcNode.GetModelBoundingBox().radius());
	bnsPcNode.setCenter(bnsPcNode.GetModelBoundingBox().center());

	// 模型光照效果
	pNewOsgNode->getOrCreateStateSet()->setMode(GL_LIGHTING,
												osg::StateAttribute::OFF |
													osg::StateAttribute::OVERRIDE);
	CPointCloudToolkit::WriteNode(pNewOsgNode, writeTask.strFilePath, CPointCloudToolkit::eAllData);
}


osg::ref_ptr<osg::MatrixTransform> CPointCloudTool::PointCloud2Osgb(
	const CPointCloudTool::WriteTask& writeTask)
{
	// 转换生成octree
	pcl::octree::OctreePointCloud<pcl::PointXYZRGBA>::Ptr pOcPointCloud(
		new pcl::octree::OctreePointCloud<pcl::PointXYZRGBA>(OctteeSize));
	pOcPointCloud->setInputCloud(writeTask.cloudPrt, writeTask.pointIndices);
	pOcPointCloud->addPointsFromInputCloud();

	osg::ref_ptr<osg::Group> pGroup = new osg::Group;
	// 遍历octree
	for (auto iterOcPC = pOcPointCloud->leaf_begin(); iterOcPC != pOcPointCloud->leaf_end();
		 ++iterOcPC)
	{
		osg::ref_ptr<osg::Geode> pGeode =
			new osg::Geode(); // 使用geode节点对应一个geometry确保包围盒剔除
		osg::ref_ptr<osg::Geometry> pGeometry = new osg::Geometry();

		osg::ref_ptr<osg::Vec3Array> pVertexArray = new osg::Vec3Array();
		osg::ref_ptr<osg::Vec4ubArray> pColorArray = new osg::Vec4ubArray();
		osg::ref_ptr<osg::Vec2Array> pTexCoordArray = new osg::Vec2Array();

		pVertexArray->reserve(RESERVE);
		pColorArray->reserve(RESERVE);
		pTexCoordArray->reserve(RESERVE);

		std::vector<int> pointIndices;
		iterOcPC.getLeafContainer().getPointIndices(pointIndices);
		for (auto iterIndex : pointIndices)
		{
			pcl::PointXYZRGBA p = writeTask.cloudPrt->at(iterIndex);

			uint32_t r = p.r;
			uint32_t g = p.g;
			uint32_t b = p.b;
			uint32_t a = p.a;

			double X = p.x - writeTask.center.x();
			double Y = p.y - writeTask.center.y();
			double Z = p.z - writeTask.center.z();

			pVertexArray->push_back(osg::Vec3(X, Y, Z));
			pColorArray->push_back(osg::Vec4ub(r, g, b, a));

			osg::Vec2 pro = CPointCloudPropertyTool::GetDefaultProperty();
			CPointCloudPropertyTool::SetSegmentProperty(pro, a);
			pTexCoordArray->push_back(pro);
		}

		pGeometry->setUseDisplayList(true);
		pGeometry->setUseVertexBufferObjects(true);
		pGeometry->setVertexArray(pVertexArray);
		pGeometry->setColorArray(pColorArray, osg::Array::BIND_PER_VERTEX);
		pGeometry->setTexCoordArray(0, pTexCoordArray);

		pGeometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, pVertexArray->size()));
		pGeode->addDrawable(pGeometry);
		pGroup->addChild(pGeode);
	}
	osg::ref_ptr<osg::MatrixTransform> transformModel = new osg::MatrixTransform;

	transformModel->setMatrix(osg::Matrix::translate(writeTask.center));
	transformModel->addChild(pGroup);
	return transformModel;
}


void CPointCloudTool::WriteMoudleInfo(const osg::BoundingBox& boundingBox,
									  CString strFileName,
									  CString strPageLodID,
									  size_t nPtCnt,
									  EPageLodLevel eLevel)
{
	MoudelInfo minmodelInfo;
	minmodelInfo.strModelLevel = EPageLodLevel2Text(eLevel);
	minmodelInfo.strPageLodID = strPageLodID;
	minmodelInfo.strFileName = strFileName;
	minmodelInfo.dMaxX = boundingBox.xMax();
	minmodelInfo.dMaxY = boundingBox.yMax();
	minmodelInfo.dMaxZ = boundingBox.zMax();
	minmodelInfo.dMinx = boundingBox.xMin();
	minmodelInfo.dMinY = boundingBox.yMin();
	minmodelInfo.dMinZ = boundingBox.zMin();
	minmodelInfo.nPointCnt = nPtCnt;
	{
		// toolkit::CCriticalSectionSync cs(_vecLevelInfo.at(eLevel)._CSC);
		tbb::mutex::scoped_lock lock(*(_vecLevelInfo.at(eLevel)._mtx));
		
		_vecLevelInfo.at(eLevel)._LevelMoudelInfoVec.emplace_back(minmodelInfo);
	}
}


void CPointCloudTool::BuildPageLodTree(d3s::share_ptr<LasOsg::PieceInfo> pPieceInfo)
{
	// 空的切片信息直接跳过
	if (nullptr == pPieceInfo || nullptr == pPieceInfo->pPieceInfoPagedLod ||
		pPieceInfo->nLevel == EPageLodLevel::eZeroLevelPageLod)
	{
		return;
	}

	CBnsPointCloudNode pCurPieceInfoPagedLod = pPieceInfo->pPieceInfoPagedLod;
	// unsigned int nFileCount = pPieceInfo->pPieceInfoPagedLod->getNumFileNames();
	// if (1 != nFileCount)	//当前结构要求一个pagedlod对应一个文件
	//{
	//	//ASSERT(FALSE && L"节点结构存在错误!");
	// }
	// 读取文件名称后移除，将文件关联在所有子节点最后
	// std::string strFileName = pCurPieceInfoPagedLod->getFileName(0);
	// float fMinRange = pCurPieceInfoPagedLod->getMinRange(0);
	// float fMaxRange = pCurPieceInfoPagedLod->getMaxRange(0);
	// pCurPieceInfoPagedLod->removeChildren(0);	//清除filename列表
	// pCurPieceInfoPagedLod->osg::LOD::removeChildren(0);	//清除rangelist列表
	int nIndex = 0;
	for (auto iterPagedLod : pPieceInfo->childpagedLodPrtvec)
	{
		if (nullptr == iterPagedLod)
			continue;
		pCurPieceInfoPagedLod->InsertNode(iterPagedLod, -1);
		// pCurPieceInfoPagedLod->setRange(nIndex++, 0, FLT_MAX);
	}
	unsigned int nChildCount = pPieceInfo->childpagedLodPrtvec.size();
	// pCurPieceInfoPagedLod->setFileName(nChildCount, strFileName);
	// pCurPieceInfoPagedLod->SetNewFileName(strFileName);
	// pCurPieceInfoPagedLod->setRange(nChildCount, fMinRange, fMaxRange);

	for (auto iterChildPieceInfo : pPieceInfo->ChildPieceVec)
	{
		BuildPageLodTree(iterChildPieceInfo.get());
	}
}


void CPointCloudTool::BuildWidePageLod(d3s::share_ptr<LasOsg::PieceInfo> pPiece,
									   pc::data::CModelNodePtr pLodNode)
{
	CBnsPointCloudNode pLasPagedLod = pLodNode;
	if (nullptr == pPiece || pLasPagedLod.IsNull())
		return;

	// 不套pageLod
	CString strOgb;
	strOgb.Format(L"%s#%d.ive",
				  CStringToolkit::CStringToUTF8(pPiece->strOgbPath).c_str(),
				  pPiece->nIdex);
	std::string sName = CStringToolkit::CStringToUTF8(strOgb);

	int nPointCount = 0;
	osg::ref_ptr<osg::Node> pNewOsgNode = WidePointCloud2Osgb(nPointCount);
	// 模型光照效果
	pNewOsgNode->getOrCreateStateSet()->setMode(GL_LIGHTING,
												osg::StateAttribute::OFF |
													osg::StateAttribute::OVERRIDE);

	osg::BoundingBox modelBoundingBox(pPiece->minPoint, pPiece->maxPoint);
	pLasPagedLod.SetModelBoundingBox(modelBoundingBox);
	pLasPagedLod.SetRealBoundingBox(modelBoundingBox);
	// pLasPagedLod->setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
	pLasPagedLod.setRadius(_dAllLasRadius);
	pLasPagedLod.setCenter(osg::Vec3(0, 0, _dLasCenterHight));
	// pLasPagedLod.setCenter(_centerPoint);
	pLasPagedLod.SetPointsNum(nPointCount);
	pLasPagedLod.SetPcLevel(pPiece->nLevel);

	CString strOgbPath = pPiece->strOsgRePath + strOgb;
	CString strActFile = _strOutPath + strOgbPath;
	CString strFileWork = _strOutPath + pPiece->strOsgRePath;
	std::string sFileWork = CStringToolkit::CStringToUTF8(strFileWork);
	std::string finame = CStringToolkit::CStringToUTF8(strActFile);
	CPointCloudToolkit::WriteNode(pNewOsgNode, finame, CPointCloudToolkit::eAllData);
	// 计数pagelod节点
	unsigned int nSize = 0;
	std::vector<d3s::share_ptr<LasOsg::PieceInfo>>& vecPieceInfo =
		_vecLevelInfo.at(eWideLevelPageLod)._PieceInfoVec;
	for (auto iterPiece : vecPieceInfo)
	{
		if (nullptr == iterPiece)
			continue;
		nSize += iterPiece->childpagedLodPrtvec.size();
	}
	// pLasPagedLod->setRange(nSize, 0, FLT_MAX);
	pLasPagedLod.setFileName(strOgb);
	// pLasPagedLod->SetNewFileName(sName);
	pLasPagedLod.setDatabasePath(strFileWork);
	CString strId = CLibToolkit::CreateGuid();
	// pLasPagedLod->SetIDKey(strId);
	// AddShader(pLasPagedLod);
	WriteMoudleInfo(modelBoundingBox, strOgb, strId, 0, EPageLodLevel::eWideLevelPageLod);
}


osg::ref_ptr<osg::Node> CPointCloudTool::WidePointCloud2Osgb(int& nPointCount)
{
	std::vector<d3s::share_ptr<LasOsg::PieceInfo>>& vecPieceInfo =
		_vecLevelInfo.at(eWideLevelPageLod)._PieceInfoVec;
	osg::ref_ptr<osg::Group> pGroup = new osg::Group();
	for (auto iterPieceInfo : vecPieceInfo)
	{
		if (nullptr == iterPieceInfo)
			continue;

		CloudPtr newCloud = iterPieceInfo->cloudPt;
		nPointCount += newCloud->size();

		// 转换生成octree
		pcl::octree::OctreePointCloud<pcl::PointXYZRGBA>::Ptr pOcPointCloud(
			new pcl::octree::OctreePointCloud<pcl::PointXYZRGBA>(100));
		pOcPointCloud->setInputCloud(newCloud);
		pOcPointCloud->addPointsFromInputCloud();

		// 遍历octree
		for (auto iterOcPC = pOcPointCloud->leaf_begin(); iterOcPC != pOcPointCloud->leaf_end();
			 ++iterOcPC)
		{
			osg::ref_ptr<osg::Geode> pGeode = new osg::Geode();
			osg::ref_ptr<osg::Geometry> pGeometry = new osg::Geometry();
			osg::ref_ptr<osg::Vec3Array> pVertexArray = new osg::Vec3Array();
			osg::ref_ptr<osg::Vec4ubArray> pColorArray = new osg::Vec4ubArray();
			osg::ref_ptr<osg::Vec2Array> pTexCoordArray = new osg::Vec2Array();

			pVertexArray->reserve(RESERVE);
			pColorArray->reserve(RESERVE);
			pTexCoordArray->reserve(RESERVE);

			std::vector<int> pointIndices;
			iterOcPC.getLeafContainer().getPointIndices(pointIndices);
			for (auto iterIndex : pointIndices)
			{
				pcl::PointXYZRGBA p = newCloud->at(iterIndex);

				uint32_t r = p.r;
				uint32_t g = p.g;
				uint32_t b = p.b;
				uint32_t a = p.a;

				double X = p.x;
				double Y = p.y;
				double Z = p.z;

				pVertexArray->push_back(osg::Vec3(X, Y, Z));
				pColorArray->push_back(osg::Vec4ub(r, g, b, a));
				osg::Vec2 property = CPointCloudPropertyTool::GetDefaultProperty();
				CPointCloudPropertyTool::SetSegmentProperty(property, a);
				pTexCoordArray->push_back(property);
			}

			pGeometry->setUseDisplayList(true);
			pGeometry->setUseVertexBufferObjects(true);
			pGeometry->setVertexArray(pVertexArray);
			pGeometry->setColorArray(pColorArray, osg::Array::BIND_PER_VERTEX);
			pGeometry->setTexCoordArray(0, pTexCoordArray);

			pGeometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, pVertexArray->size()));
			pGeode->addDrawable(pGeometry);
			pGroup->addChild(pGeode);
		}
	}
	osg::ref_ptr<osg::MatrixTransform> transformModel = new osg::MatrixTransform;
	osg::Matrix matrix;
	transformModel->setMatrix(matrix);
	transformModel->addChild(pGroup);

	return transformModel;
}


void CPointCloudTool::SavePieceInfo2Xml(double dLength,
										const osg::Vec3d& center,
										double dHight,
										double dMin_Z)
{
	const CString LOAD_LAS_LASLEVEL = L"Las";
	const CString STRING_BASE_POSITION_X_ATTRIBUTE_NAME = L"BasePointX"; // 基准点X
	const CString STRING_BASE_POSITION_Y_ATTRIBUTE_NAME = L"BasePointY"; // 基准点Y
	const CString STRING_BASE_POSITION_Z_ATTRIBUTE_NAME = L"BasePointZ"; // 基准点Z
	const CString ALL_LAS_RADIUS = L"AllLasRadius";
	const CString ALL_LAS_HIGHT = L"AllLasHight";

	/* 1. 路径*/
	CString szFilePath = _strOutPath + L"LasLevelInfo.xml";

	toolkit::CXmlDocument doc;
	toolkit::CXmlElement* rootEle = nullptr;
	rootEle = doc.GetElementRoot();
	rootEle->reset();
	rootEle->SetElementName(L"ModelInfo");

	// 构建Las层级
	toolkit::CXmlElement* pLas = rootEle->GetChildElementAt(LOAD_LAS_LASLEVEL);
	pLas->SetAttrValue(STRING_BASE_POSITION_X_ATTRIBUTE_NAME, CStringToolkit::DblToStr(center.x()));
	pLas->SetAttrValue(STRING_BASE_POSITION_Y_ATTRIBUTE_NAME, CStringToolkit::DblToStr(center.y()));
	pLas->SetAttrValue(STRING_BASE_POSITION_Z_ATTRIBUTE_NAME, CStringToolkit::DblToStr(center.z()));
	pLas->SetAttrValue(ALL_LAS_RADIUS, CStringToolkit::DblToStr(dLength));
	pLas->SetAttrValue(ALL_LAS_HIGHT, CStringToolkit::DblToStr(dHight));
	pLas->SetAttrValue(L"MinZ", CStringToolkit::DblToStr(dMin_Z));

	// 保持原本结构单独写入第三层级信息
	std::vector<MoudelInfo>& wideMoudelInfo =
		_vecLevelInfo.at(eWideLevelPageLod)._LevelMoudelInfoVec;
	toolkit::CXmlElements* pLass = pLas->GetChildElements();
	for (size_t i = 0; i < wideMoudelInfo.size(); ++i)
	{
		toolkit::CXmlElement* pelement = pLass->InsertAt(i);
		pelement->SetElementName(LOAD_LAS_LASLEVEL);
		MoudelInfo& moudelInfo = wideMoudelInfo.at(i);
		pelement->SetAttrValue(L"PageLodID", moudelInfo.strPageLodID);
		pelement->SetAttrValue(L"MinX", CStringToolkit::DblToStr(moudelInfo.dMinx));
		pelement->SetAttrValue(L"MinY", CStringToolkit::DblToStr(moudelInfo.dMinY));
		pelement->SetAttrValue(L"MinZ", CStringToolkit::DblToStr(moudelInfo.dMinZ));
		pelement->SetAttrValue(L"MaxX", CStringToolkit::DblToStr(moudelInfo.dMaxX));
		pelement->SetAttrValue(L"MaxY", CStringToolkit::DblToStr(moudelInfo.dMaxY));
		pelement->SetAttrValue(L"MaxZ", CStringToolkit::DblToStr(moudelInfo.dMaxZ));
	}

	// 构建层级信息(0-2层级)
	for (int nIndex = 0; nIndex < _vecLevelInfo.size() - 1; ++nIndex)
	{
		toolkit::CXmlElement* pMinModelXml =
			rootEle->GetChildElementAt(EPageLodLevel2Text((EPageLodLevel)nIndex));
		toolkit::CXmlElements* pelements = pMinModelXml->GetChildElements();
		std::vector<MoudelInfo>& LevelMoudelInfoVec = _vecLevelInfo.at(nIndex)._LevelMoudelInfoVec;
		for (size_t i = 0; i < LevelMoudelInfoVec.size(); ++i)
		{
			MoudelInfo& moudelInfo = LevelMoudelInfoVec.at(i);
			toolkit::CXmlElement* pelement = pelements->InsertAt(i);
			pelement->SetElementName(moudelInfo.strModelLevel);
			pelement->SetAttrValue(IVE_FILE_NAME, moudelInfo.strFileName);
			pelement->SetAttrValue(PAGED_LOD_ID, moudelInfo.strPageLodID);
			pelement->SetAttrValue(L"MinX", CStringToolkit::DblToStr(moudelInfo.dMinx));
			pelement->SetAttrValue(L"MinY", CStringToolkit::DblToStr(moudelInfo.dMinY));
			pelement->SetAttrValue(L"MinZ", CStringToolkit::DblToStr(moudelInfo.dMinZ));
			pelement->SetAttrValue(L"MaxX", CStringToolkit::DblToStr(moudelInfo.dMaxX));
			pelement->SetAttrValue(L"MaxY", CStringToolkit::DblToStr(moudelInfo.dMaxY));
			pelement->SetAttrValue(L"MaxZ", CStringToolkit::DblToStr(moudelInfo.dMaxZ));
			pelement->SetAttrValue(PIECE_POINTS_NUM,
								   CStringToolkit::IntToStr(moudelInfo.nPointCnt));
		}
	}
	doc.SaveFile(szFilePath, toolkit::fmtXMLUTF8);
}