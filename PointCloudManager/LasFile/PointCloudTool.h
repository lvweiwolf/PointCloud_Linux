//////////////////////////////////////////////////////////////////////
// 文件名称：PointCloudTool.h
// 功能描述：点云文件解析工具
// 创建标识：吴建峰 2026/3/01
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////

#ifndef POINTCLOUDTOOL_H_
#define POINTCLOUDTOOL_H_

#include <LASlib/lasreader.hpp>
#include <LasFile/OsgDefines.h>

#include <BusinessNode/PCNodeType.h>
#include <BusinessNode/BnsProjectNode.h>
#include <BusinessNode/BnsPointCloudNode.h>

#include <unordered_map>

class CPointCloudWriteThread;

class CPointCloudTool
{
	const int nLevel = 4; // 共拆分为4级

	// 层级信息
	struct SLevelInfo
	{
		int _nX500Size;	   // 最小划分单位x总数
		int _nY500Size;	   // 最小划分单位y总数
		int _nZ500Size;	   // 最小划分单位z总数
		float _fPieceSize; // 切片大小

		float _fPageLodMinPixel; // pageLod最小显示像素
		float _fPageLodMaxPixel; // pageLod最大显示像素

		std::vector<d3s::share_ptr<LasOsg::PieceInfo>> _PieceInfoVec; // 最小层级PieceInfo
		std::vector<MoudelInfo> _LevelMoudelInfoVec;				  // Level模型信息记录集合

		SLevelInfo()
		{
			_nX500Size = 0;
			_nY500Size = 0;
			_nZ500Size = 0;
			_fPieceSize = 0.0;

			_fPageLodMinPixel = 0.0;
			_fPageLodMaxPixel = 0.0;
		};
	};

	struct WriteTask
	{
		// WriteFileType eFileType;	//写入文件类型
		std::string strFilePath; // 文件路径
		CloudPtr cloudPrt;		 // 点云点数组
		osg::Vec3d center;		 // 块的中心点,做偏移矩阵用
		pc::data::CModelNodePtr pPageLod;
		boost::shared_ptr<std::vector<int>> pointIndices; // 关联点云中的索引
	};

public:
	CPointCloudTool(const CString& strLasFilePath,
					pc::data::CModelNodePtr pProjectNode,
					const CString& strOutPath);
	~CPointCloudTool();

public:
	// 类型映射
	static std::map<unsigned, unsigned> ReadTypeMapping();

public:
	// 加载las
	void LoadLas();

	/*
	 *  函数介绍：	保存点位信息
	 *  输入参数：	dKeyX，	dKeyY，dKeyZ		此块的最小XYZ
	 *  输入参数：	pPiece						父级块
	 *  输出参数：	PieceInfo*					新块
	 */
	void SavePoinToPieceInfo(pcl::PointXYZRGBA& pointXYZRGBA, const I64& curPointIndex);

protected:
	// 打开LAS文件
	void FileOpen();

	// 获取包围盒
	bool GetBoundingBox(size_t& nPointCount,
						double& dMaxX,
						double& dMaxY,
						double& dMaxZ,
						double& dMinX,
						double& dMinY,
						double& dMinZ);

	/**
	 *  @brief    精细切片
	 *
	 *  @param    std::vector<osg::ref_ptr<CPointCloudpagedLod>> lasOsgNodeVec 粗切片构造的节点
	 *  @param   pOffSet 自定义中心偏移
	 *  @return   void
	 */
	std::vector<std::pair<CString, osg::ref_ptr<CPointCloudpagedLod>>> CutLasFile(
		const osg::Vec3d* pOffSet);

	/*
	 *  函数介绍：	读取Las文件
	 *  输入参数：	filename			osgNode
	 */
	pc::data::CModelNodePtr Las2Osgb();

	/**
	 *  @brief   获取统一的区域
	 *
	 *  @param    double dValue  坐标值X或Y或Z
	 *  @param    double dCentreValue 相对中心点
	 *  @param    bool bMax 是否最大min,max
	 *  @return   double 新的值
	 */
	double GetAreaValue(double dValue, double dCentreValue, bool bMax);

	/**
	 * 创建层级信息
	 * @param [in] value	点云各方向的最大最小值
	 * @return
	 */
	void CreateLevelInfo(double Max_x,
						 double Min_x,
						 double Max_y,
						 double Min_y,
						 double Max_z,
						 double Min_z);

	/*
	 *  函数介绍：	创建las块的信息
	 */
	d3s::share_ptr<LasOsg::PieceInfo> CreateLasPieceInfoPtr(const CString& strFilename);

	/**
	 * 构建所有切片信息
	 * @return
	 */
	void CreateLasAllPiece(d3s::share_ptr<LasOsg::PieceInfo> thisPiece,
						   double Max_x,
						   double Max_y,
						   double Max_z,
						   double Min_x,
						   double Min_y,
						   double Min_z);

	/**
	 * 点分类到切片
	 * @param [in] lasreader
	 * @return
	 */
	void ProcessPointToPiece();
	void ProcessPointToPiece2();

	/**
	 * 读取点
	 * @param [in] point			点信息
	 * @param [in] dCenterPointX 中心点x
	 * @param [in] dCenterPointY 中心点y
	 * @param [in] dCenterPointZ 中心点z
	 * @param [in] bReadClassify 是否读取分类（通过pcl::PointXYZRGBA::a传递分类）
	 * @return
	 */
	bool ReadPoint(pcl::PointXYZRGBA& point,
				   const double& dCenterPointX,
				   const double& dCenterPointY,
				   const double& dCenterPointZ,
				   const bool& bReadClassify);

	/**
	 * 批量读取点 - 性能优化版本
	 * @param [out] points       点数组
	 * @param [in] maxCount      最大读取数量
	 * @param [in] dCenterPointX 中心点x
	 * @param [in] dCenterPointY 中心点y
	 * @param [in] dCenterPointZ 中心点z
	 * @param [in] bReadClassify 是否读取分类
	 * @return 实际读取的点数
	 */
	size_t ReadPoints(pcl::PointXYZRGBA* points,
					  size_t maxCount,
					  const double& dCenterPointX,
					  const double& dCenterPointY,
					  const double& dCenterPointZ,
					  const bool& bReadClassify);

	// void SavePoinToPieceInfo(pcl::PointXYZRGBA& pointXYZRGBA, const I64& curPointIndex);

	/**
	 * 写入PCD文件
	 * @param [in] pPiece
	 * @return
	 */
	void WritePCDFile(d3s::share_ptr<LasOsg::PieceInfo> pPiece);

	/*
	 *  函数介绍：	构建所有Padelod信息
	 *  输入参数：
	 */
	pc::data::CModelNodePtr BuildAllPageLod(d3s::share_ptr<LasOsg::PieceInfo> pPiece);

	/*
	 *  函数介绍：	构建pagelod层级信息
	 *  输入参数：	pPiece		块信息
	 */
	void BuildPageLodInfo(d3s::share_ptr<LasOsg::PieceInfo> pPiece);

	/*
	 *  函数介绍：	执行写osg文件任务
	 *  输入参数：	pPiece		块信息
	 */
	void ExcuteWriteOsgTask(const CPointCloudTool::WriteTask& writeTask);

	/**
	 *  @brief   点云点转osgNode
	 *
	 *  @param   CloudPtr newCloud
	 *  @param   const osg::Vec3d & center 偏移点
	 *  @return   osg::ref_ptr<osg::Node>
	 */
	osg::ref_ptr<osg::MatrixTransform> PointCloud2Osgb(const CPointCloudTool::WriteTask& writeTask);

	/**
	 *  函数介绍    	写最小模型信息
	 *
	 *  输入参数    	osg::ref_ptr<osg::Node> pModelnode		osg模型
	 *  输入参数    	CString strFileName						文件名
	 *  输入参数    	CString strPageLodID					pagelodid
	 *  输入参数    	size_t nPtCnt							点数量
	 *  输出参数
	 *  返回值   	void
	 */
	void WriteMoudleInfo(const osg::BoundingBox& boundingBox,
						 CString strFileName,
						 CString strPageLodID,
						 size_t nPtCnt,
						 EPageLodLevel eLevel);

	/**
	 * 递归构建pagedlod
	 * @param [in] iterPieceInfo
	 * @param [in] pWidePageLod
	 * @return
	 */
	void BuildPageLodTree(d3s::share_ptr<LasOsg::PieceInfo> pPieceInfo);

	/**
	 * 构建粗略层级pagelod
	 * @param [in] pPiece
	 * @return
	 */
	void BuildWidePageLod(d3s::share_ptr<LasOsg::PieceInfo> pPiece,
						  pc::data::CModelNodePtr pLasPagedLod);

	/*
	 *  函数介绍：	点云转OsgNode
	 *  输入参数：	newCloud 点云
	 */
	osg::ref_ptr<osg::Node> WidePointCloud2Osgb(int& nPointCount);

	/*
	 *  函数介绍：	写最模型信息到XML文件
	 */
	void SavePieceInfo2Xml(double dLength, const osg::Vec3d& center, double dHight, double dMin_Z);

	/**
	 * 获取扫描日期
	 * @param [in] nYear
	 * @param [in] nDay
	 * @return
	 */
	bool GetScanTime();

protected:
	CString _strLasFilePath;
	pc::data::CModelNodePtr _pProjectNode;
	CString _strOutPath;

	osg::Vec3d _centerPoint; // 中心相对坐标
	double _dAllLasRadius;	 // 所有las的半径
	double _dLasCenterHight;
	double _dHight;
	double _dMin_Z;

	LASreadOpener _lasreadopener; // las文件操作器
	LASreader* _pLasreader;		  // las文件读取器

	bool _bShuZiLvDiLas; // 是否读取数字绿地las
	bool _bBoowayLas;	 // 是否读取博微导出las（区别记录点云类别的属性）
	double _dOffSetZ;	 // Z偏移，确保点云Z大于0

	int _nLasIdex; // 当前Las索引
	std::vector<SLevelInfo>
		_vecLevelInfo; // 0-3分别对应精细-粗糙 （记录各层级的切片 避免查询的方式）
	int _nStartX;	   // lasX最小//向下取整处理过
	int _nStartY;	   // lasY最小
	int _nStartZ;	   // lasZ最小

	std::map<unsigned, unsigned> _typeMapping;

	CPointCloudWriteThread* _writeFileThread; // 写文件线程
};

#endif // POINTCLOUDTOOL_H_
