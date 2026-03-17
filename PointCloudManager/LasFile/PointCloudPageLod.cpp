#include <LasFile/PointCloudPageLod.h>
#include <LasFile/IPageLodAcceptHandler.h>
#include <LasFile/SamplesOnOffGeometry.h>
#include <LasFile/PointCloudToolkit.h>
#include <LasFile/PointCloudPropertyTool.h>
#include <LasFile/PointCloudToolDefine.h>

#include <Tool/FileToolkit.h>

#include <osg/Geode>
#include <osg/ProxyNode>
#include <osgDB/ReadFile>
#include <osgUtil/CullVisitor>
#include <osgUtil/IntersectionVisitor>

// 最小卸载帧数
const int INT_MIN_EXPIRY_FRAMES = 20;
// 拷贝最小卸载帧数
const int INT_MIN_COPY_EXPIRY_FRAMES = 5;

// 动态渲染时，最大定顶点检测
const int INT_MAX_DYNAMIC_RENDER_NUM = 2000000;

const int INT_NORMALPOINT_RANGE_OFFSET = 1000; // 动态渲染时，正常体量点云渲染像素范围偏移量
const int INT_LARGEPOINT_RANGE_OFFSET =
	2000; // 动态渲染时，渲染顶点大于 INT_MAX_DYNAMIC_RENDER_NUM 值时，点云渲染像素范围偏移量

// 大点云侧视视角处理
const int TOPEXTEND = 600;			// 视角近平面扩展临界值
const int TOPHANDEL = 1000;			// top在范围内需要处理的判断值
const int VIEWLENGTHHANDEL = 10000; // 远近平面距离，需要处理的判断值
const int ZEROSHOW = 1000;			// 0层级展现参数
const int ONESHOW = 2000;			// 1层级展现参数
const int TWOSHOW = 5000;			// 2层级展现参数

const CString STRING_VERTEX_SHADER_RELATIVE_PATH = L"";
const CString STRING_FRAGMENT_SHADER_RELATIVE_PATH = L"";

// 点云pagelod遍历处理器
class CPointCloudPageLodAcceptHandler : public IPageLodAcceptHandler
{
public:
	CPointCloudPageLodAcceptHandler() {}

	~CPointCloudPageLodAcceptHandler() {}

private:
	/**
	 * 处理遍历器接口函数
	 * @param [in/out] nv 节点遍历器
	 * @return true : 标识已处理 false : 标识未处理
	 */
	bool HandleAccept(osg::NodeVisitor& nv)
	{

		osgUtil::IntersectionVisitor* pIntersection =
			dynamic_cast<osgUtil::IntersectionVisitor*>(&nv);
		if (pIntersection != nullptr)
			return true;
		return false;
	}
};

// 占用节点引用计数遍历器
class COccupiedReferenceVisitor : public osg::NodeVisitor
{
public:
	COccupiedReferenceVisitor() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {};

public:
	virtual void apply(osg::Node& node) override
	{
		_nodes.emplace_back(&node);
		osg::NodeVisitor::apply(node);
	}

private:
	std::vector<osg::ref_ptr<osg::Node>> _nodes;
};

CPointCloudpagedLod::CPointCloudpagedLod()
	: _nPointsNum(0),
	  _pAcceptHandler(new CPointCloudPageLodAcceptHandler),
	  _pTexProxyNode(new osg::ProxyNode),
	  _pColorProxyNode(new osg::ProxyNode)
{
	_bDirty = false;
	_bIsLoad = false;
}

CPointCloudpagedLod::CPointCloudpagedLod(const CPointCloudpagedLod& plod,
										 const osg::CopyOp& copyop /*= CopyOp::SHALLOW_COPY*/)
	: osg::PagedLOD(plod, copyop), _pAcceptHandler(plod._pAcceptHandler)
{
	_bDirty = plod._bDirty;
	_bIsLoad = plod._bIsLoad;
	_strId = plod._strId;
	_modelBoundingBox = plod._modelBoundingBox;
	_realBoundingBox = plod._realBoundingBox;
	_nPointsNum = plod._nPointsNum;
	_strNewFileName = plod._strNewFileName;
	_strOldFileName = plod._strOldFileName;
	if (_pAcceptHandler == nullptr)
		_pAcceptHandler = new CPointCloudPageLodAcceptHandler;

	// 拷贝时提前占用引用计数，防止拷贝过程中因内存释放后操作野指针异常
	if (nullptr != plod._pTexProxyNode)
	{
		COccupiedReferenceVisitor occupiedReferenceV;
		plod._pTexProxyNode->traverse(occupiedReferenceV);
		_pTexProxyNode = new osg::ProxyNode(*plod._pTexProxyNode, osg::CopyOp::DEEP_COPY_ALL);
	}
	if (nullptr != plod._pColorProxyNode)
	{
		COccupiedReferenceVisitor occupiedReferenceV;
		plod._pColorProxyNode->traverse(occupiedReferenceV);
		_pColorProxyNode = new osg::ProxyNode(*plod._pColorProxyNode, osg::CopyOp::DEEP_COPY_ALL);
	}

	// 将超出帧数差的节点卸载，避免手动分类内存过高
	if (!_children.empty())
	{
		unsigned cindex = _children.size() - 1;
		if (!_perRangeDataList[cindex]._filename.empty() &&
			_perRangeDataList[cindex]._frameNumber + INT_MIN_COPY_EXPIRY_FRAMES < 0)
		{
			osg::Node* nodeToRemove = _children[cindex].get();
			Group::removeChildren(cindex, 1);
		}
	}
}

CPointCloudpagedLod::~CPointCloudpagedLod() {}

bool CPointCloudpagedLod::addChild(Node* child)
{
	if (child == nullptr)
		return false;
	AttachTexCoordArray(child);
	ReplaceSamplesOnOffGeometry(child->asGroup());
	return osg::PagedLOD::addChild(child);
}

bool CPointCloudpagedLod::addChild(Node* child, float rmin, float rmax)
{
	if (child == nullptr)
		return false;
	AttachTexCoordArray(child);
	ReplaceSamplesOnOffGeometry(child->asGroup());
	return osg::PagedLOD::addChild(child, rmin, rmax);
}

bool CPointCloudpagedLod::addChild(Node* child,
								   float rmin,
								   float rmax,
								   const std::string& filename,
								   float priorityOffset /*= 0.0f*/,
								   float priorityScale /*= 1.0f*/)
{
	if (child == nullptr)
		return false;
	AttachTexCoordArray(child);
	ReplaceSamplesOnOffGeometry(child->asGroup());
	return osg::PagedLOD::addChild(child, rmin, rmax, filename, priorityOffset, priorityScale);
}

static int _nShowBoundBoxLevel = -1;
void CPointCloudpagedLod::SetShowBoundingBox(int nLevel) { _nShowBoundBoxLevel = nLevel; }
void CPointCloudpagedLod::ApplyBoundingBoxGeode(osg::NodeVisitor& nv)
{
	if (_nShowBoundBoxLevel == -1)
	{
		return;
	}
	int nCount = CStringToolkit::CountWord(_strId, L'#');
	int nLevel = 3;
	if (nCount == 6)
	{
		nLevel = CStringToolkit::StrToInt(CStringToolkit::ReadWord(_strId, 5, L'#'));
	}

	if (_nShowBoundBoxLevel < 4 && _nShowBoundBoxLevel != nLevel)
	{
		return;
	}

	if (_pBoundingBoxGeode == NULL)
	{
		osg::Vec4 color = osg::Vec4(1.0, 0.0, 0.0, 1);
		if (nLevel == 0)
		{
			color = osg::Vec4(1.0, 1.0, 0.0, 1);
		}
		else if (nLevel == 1)
		{
			color = osg::Vec4(0.0, 0.0, 1.0, 1);
		}
		else if (nLevel == 2)
		{
			color = osg::Vec4(0.0, 1.0, 0.0, 1);
		}
		_pBoundingBoxGeode = CPointCloudToolkit::BuildBoundingBoxGeode(_realBoundingBox, color);
		_pBoundingBoxGeode->getOrCreateStateSet()->setAttributeAndModes(
			new osg::Program(),
			osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
	}

	if (nv.asCullVisitor())
	{
		nv.apply(*_pBoundingBoxGeode);
	}
}


int CPointCloudpagedLod::GetPagedLodLevel()
{
	int nCount = CStringToolkit::CountWord(_strId, L'#');
	int nLevel = 3; // 默认第三层级
	if (nCount == 6)
	{
		nLevel = CStringToolkit::StrToInt(CStringToolkit::ReadWord(_strId, 5, L'#'));
	}
	return nLevel;
}

bool CPointCloudpagedLod::JudgeLoadFineBigLas(osg::NodeVisitor& nv)
{
	bool bLoadBigLas = true;
	osgUtil::CullVisitor* pCVis = dynamic_cast<osgUtil::CullVisitor*>(&nv);
	if (nullptr != pCVis)
	{
		osg::RenderInfo pRender = pCVis->getRenderInfo();
		osg::View* pView = pRender.getView();
		if (nullptr != pView)
		{
			osg::Camera* pCamera1 = nullptr;
			pCamera1 = pView->getCamera();
			if (pCamera1 != nullptr)
			{
				double left, right, bottom, top, dNear, dFar;
				float drequired = nv.getDistanceToViewPoint(getCenter(), true);
				pCamera1->getProjectionMatrixAsOrtho(left, right, bottom, top, dNear, dFar);
				double dViewLendth = dFar - dNear;
				double dExtend = 0.0;
				if (top < TOPEXTEND)
				{
					// 扩展平面距离，800、10为试出的参数（可修改）
					dExtend = (800 - top) * 10;
				}
				if (dViewLendth + dExtend > VIEWLENGTHHANDEL && top < TOPHANDEL)
				{
					// 模型中心点距离近平面
					double dLendth = drequired - dNear;
					if (dLendth > TWOSHOW && GetPagedLodLevel() == 2)
						bLoadBigLas = false;
					if (dLendth > ONESHOW && GetPagedLodLevel() == 1)
						bLoadBigLas = false;
					if (dLendth > ZEROSHOW && GetPagedLodLevel() == 0)
						bLoadBigLas = false;
				}
			}
		}
	}
	return bLoadBigLas;
}

// 重写class osg::NodeAcceptOp，增加方法容错
class NodeAcceptOpEx
{
public:
	NodeAcceptOpEx(osg::NodeVisitor& nv) : _nv(nv) {}
	NodeAcceptOpEx(const NodeAcceptOpEx& naop) : _nv(naop._nv) {}

public:
	void operator()(osg::Node* node)
	{
		if (nullptr != node)
			node->accept(_nv);
	}
	void operator()(osg::ref_ptr<osg::Node> node)
	{
		if (nullptr != node)
			node->accept(_nv);
	}

protected:
	NodeAcceptOpEx& operator=(const NodeAcceptOpEx&) { return *this; }
	osg::NodeVisitor& _nv;
};

void CPointCloudpagedLod::traverse(osg::NodeVisitor& nv)
{
	osg::PagedLOD::traverse(nv);

	//// set the frame number of the traversal so that external nodes can find out how active this
	//  // node is.
	// unsigned int frameNumber = nv.getFrameStamp() ? nv.getFrameStamp()->getFrameNumber() : 0;
	// if (nv.getFrameStamp() &&
	//	nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
	//{
	//	unsigned int nFrameNumber = 0;
	//	if (0 == nFrameNumber)	//未启用rtt相机
	//	{
	//		setFrameNumberOfLastTraversal(nv.getFrameStamp()->getFrameNumber());
	//	}
	//	else
	//	{
	//		setFrameNumberOfLastTraversal(nFrameNumber);
	//		frameNumber = nFrameNumber;
	//	}
	//}

	// double timeStamp = nv.getFrameStamp() ? nv.getFrameStamp()->getReferenceTime() : 0.0;
	//
	// bool bIsCullVisitor = false;
	// d3s::render::IRenderParam* pRenderParam = NULL;
	// if (nv.asCullVisitor())
	//{
	//	bIsCullVisitor = true;
	//	pRenderParam = (d3s::render::IRenderParam*)(nv.asCullVisitor()->getUserData());
	//	ApplyBoundingBoxGeode(nv);
	// }
	//
	// switch (nv.getTraversalMode())
	//{
	// case(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN):
	//	std::for_each(_children.begin(), _children.end(), NodeAcceptOpEx(nv));
	//	break;
	// case(osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN):
	//{
	//	osg::Vec3d pointcenter = getCenter();
	//	float required_range = 0;
	//	if (_rangeMode == DISTANCE_FROM_EYE_POINT)
	//	{
	//		required_range = nv.getDistanceToViewPoint(getCenter(), true);
	//	}
	//	else
	//	{
	//		osg::CullStack* cullStack = dynamic_cast<osg::CullStack*>(&nv);
	//		if (cullStack && cullStack->getLODScale() > 0.0f)
	//		{
	//			required_range = cullStack->clampedPixelSize(getBound()) / cullStack->getLODScale();
	//		}
	//		else
	//		{
	//			// fallback to selecting the highest res tile by
	//			// finding out the max range
	//			for (unsigned int i = 0; i < _rangeList.size(); ++i)
	//			{
	//				required_range = osg::maximum(required_range, _rangeList[i].first);
	//			}
	//		}
	//	}
	//	//展现大点云的控制
	//	//bool bLoadBigLas = JudgeLoadFineBigLas(nv);

	//	//自己加的规则：子层级为脏，且未加载，自身已经加载到内存，则渲染自身
	//	size_t nChild = _children.size();
	//	if (_perRangeDataList.size()== nChild)
	//	{
	//		bool bIsLoadSelf=false;
	//		for (size_t i=0;i<nChild;++i)
	//		{
	//			osg::ref_ptr<osg::Node>  pNode= _children[i];
	//			osg::ref_ptr<CPointCloudpagedLod> pPageLod =
	// dynamic_pointer_cast<CPointCloudpagedLod>(pNode); 			if (!pPageLod)
	// continue; 			if
	//(!pPageLod->GetDirty()) 				continue; 			if (pPageLod->getNumFileNames() !=
	// pPageLod->getNumChildren())
	//			{
	//				bIsLoadSelf = true;
	//				break;
	//			}
	//		}
	//		if(bIsLoadSelf)
	//			_children[nChild-1]->accept(nv);
	//	}

	//	// 如果视图为脏状态，只渲染上一个LOD层级范围
	//	int lastChildTraversed = -1;
	//	bool needToLoadChild = false;
	//	int nDirtyRange = 1;
	//	if (pRenderParam != NULL && pRenderParam->GetRenderVectorNumber() >
	// INT_MAX_DYNAMIC_RENDER_NUM)
	//	{
	//		nDirtyRange = required_range - INT_LARGEPOINT_RANGE_OFFSET > 0 ? required_range -
	// INT_LARGEPOINT_RANGE_OFFSET : 1;
	//	}
	//	else
	//	{
	//		nDirtyRange = required_range - INT_NORMALPOINT_RANGE_OFFSET > 0 ? required_range -
	// INT_NORMALPOINT_RANGE_OFFSET : 1;
	//	}
	//	auto nRangeCnt = _rangeList.size();
	//	for (unsigned int i = 0; i < nRangeCnt; ++i)
	//	{
	//		if (pRenderParam != NULL &&
	//			nv.asCullVisitor()->getFrameStamp()->getFrameNumber() -
	// pRenderParam->GetDirtyFrameNumber() < 5 && 			_rangeList[i].first > nDirtyRange)
	//		{
	//			pRenderParam->SetRenderComplate(false);
	//			continue;
	//		}
	//		if (_rangeList[i].first <= required_range && required_range < _rangeList[i].second)
	//		{
	//			// 435271
	//(偶现）手动分类：使用剖切分类功能框选后，剖切视图部分的点云会一直闪烁，附图V1.0.0.25
	//			// 卸载设置一个最小不显示帧数
	//			_perRangeDataList[i]._minExpiryFrames = INT_MIN_EXPIRY_FRAMES;
	//			if (i < _children.size()/*&& bLoadBigLas*/)//这里判断自己
	//			{
	//				if (bIsCullVisitor)
	//				{
	//					_perRangeDataList[i]._timeStamp = timeStamp;
	//					_perRangeDataList[i]._frameNumber = frameNumber;
	//				}

	//				_children[i]->accept(nv);
	//				lastChildTraversed = (int)i;
	//			}
	//			else
	//			{
	//				needToLoadChild = true;
	//			}
	//		}
	//	}
	//
	//	if ((needToLoadChild/*&&bLoadBigLas*/))
	//	{
	//		unsigned int numChildren = _children.size();

	//		// select the last valid child.
	//		if (numChildren > 0 && ((int)numChildren - 1) != lastChildTraversed)
	//		{
	//			if (bIsCullVisitor)
	//			{
	//				_perRangeDataList[numChildren - 1]._timeStamp = timeStamp;
	//				_perRangeDataList[numChildren - 1]._frameNumber = frameNumber;
	//			}
	//			_children[numChildren - 1]->accept(nv);
	//		}

	//		// now request the loading of the next unloaded child.
	//		if (!_disableExternalChildrenPaging &&
	//			nv.getDatabaseRequestHandler() &&
	//			numChildren < _perRangeDataList.size())
	//		{
	//			// compute priority from where abouts in the required range the distance falls.
	//			float priority = (_rangeList[numChildren].second - required_range) /
	//(_rangeList[numChildren].second - _rangeList[numChildren].first);

	//			// invert priority for PIXEL_SIZE_ON_SCREEN mode
	//			if (_rangeMode == PIXEL_SIZE_ON_SCREEN)
	//			{
	//				priority = -priority;
	//			}

	//			// modify the priority according to the child's priority offset and scale.
	//			priority = _perRangeDataList[numChildren]._priorityOffset + priority *
	//_perRangeDataList[numChildren]._priorityScale;

	//			if (_databasePath.empty())
	//			{
	//				nv.getDatabaseRequestHandler()->requestNodeFile(_perRangeDataList[numChildren]._filename,
	// nv.getNodePath(), priority, nv.getFrameStamp(),
	//_perRangeDataList[numChildren]._databaseRequest, _databaseOptions.get());
	//			}
	//			else
	//			{
	//				//请求自己的ive加载说明未加载
	//				if (!_bDirty)
	//				{
	//					//加载纹理数组
	//					RequestLoadTexture(nv, priority);

	//					//加载颜色数组
	//					if ((0 == _pColorProxyNode->getNumFileNames() ||
	//						0 == _pColorProxyNode->getDatabaseRequest(0)) &&
	//						0 == _pColorProxyNode->getNumChildren())
	//					{
	//						std::string strFileName = _perRangeDataList[numChildren]._filename;
	//						strFileName.replace(strFileName.find(pc::data::strFilExt),
	// strFileName.length(), pc::data::ColorPostStr + pc::data::strFilExt);
	//						_pColorProxyNode->setFileName(0, strFileName);
	//						nv.getDatabaseRequestHandler()->requestNodeFile(_databasePath +
	//_pColorProxyNode->getFileName(0), 							osg::NodePath({
	//_pColorProxyNode.get() }), priority + 9, nv.getFrameStamp(),
	// _pColorProxyNode->getDatabaseRequest(0), _pColorProxyNode->getDatabaseOptions());
	//					}

	//					// prepend the databasePath to the child's filename.
	//					nv.getDatabaseRequestHandler()->requestNodeFile(_databasePath +
	//_perRangeDataList[numChildren]._filename, osg::NodePath({this}), priority, nv.getFrameStamp(),
	//_perRangeDataList[numChildren]._databaseRequest, _databaseOptions.get());
	//_bIsLoad = true;
	//				}
	//
	//			}
	//		}

	//		// 解决开启RTT时，点云修改数据未及时刷新至画布上的问题
	//		if (nullptr != pRenderParam)
	//			pRenderParam->SetRenderComplate(false);
	//	}

	//	//判断DatabasePager是否完成节点加载
	//	osgDB::DatabasePager* pDatabasePager =
	//(osgDB::DatabasePager*)(nv.getDatabaseRequestHandler()); 	if (nullptr != pDatabasePager &&
	//(0
	//!= pDatabasePager->getFileRequestListSize() || 			0 !=
	//! pDatabasePager->getDataToCompileListSize()
	//|| 			0 != pDatabasePager->getDataToMergeListSize()))
	//	{
	//		d3s::render::IRenderParam* pRenderParam =
	//(d3s::render::IRenderParam*)(nv.asCullVisitor()->getUserData()); 		if (nullptr !=
	// pRenderParam) 			pRenderParam->SetRenderComplate(false);
	//	}

	//	break;
	//}
	// default:
	//	break;
	//}
}

void CPointCloudpagedLod::accept(osg::NodeVisitor& nv)
{
	if (_pAcceptHandler != nullptr && _pAcceptHandler->HandleAccept(nv))
	{
		if (nv.validNodeMask(*this))
		{
			nv.pushOntoNodePath(this);
			nv.apply(*dynamic_cast<osg::LOD*>(this));
			nv.popFromNodePath();
		}
		return;
	}

	osg::PagedLOD::accept(nv);
}
bool CPointCloudpagedLod::GetDirty() { return _bDirty; }

void CPointCloudpagedLod::SetDirty(bool bDirty /*= true*/) { _bDirty = bDirty; }

CString CPointCloudpagedLod::GetIDKey() { return _strId; }

void CPointCloudpagedLod::SetIDKey(CString strID) { _strId = strID; }

size_t CPointCloudpagedLod::GetPointsNum(void) { return _nPointsNum; }

void CPointCloudpagedLod::SetPointsNum(size_t nPointNum) { _nPointsNum = nPointNum; }

const osg::BoundingBox& CPointCloudpagedLod::GetModelBoundingBox() const
{
	return _modelBoundingBox;
}

void CPointCloudpagedLod::SetModelBoundingBox(const osg::BoundingBox& modelBoundingBox)
{
	_modelBoundingBox = modelBoundingBox;
}

bool CPointCloudpagedLod::GetIsLoad() { return _bIsLoad; }

std::vector<osg::ref_ptr<CPointCloudpagedLod>> CPointCloudpagedLod::GetNextLevelChidren(void)
{
	std::vector<osg::ref_ptr<CPointCloudpagedLod>> vNextLevelChidren;
	for (auto& node : _children)
	{
		osg::ref_ptr<CPointCloudpagedLod> pPointCloudPagedLod =
			dynamic_pointer_cast<CPointCloudpagedLod>(node);
		if (pPointCloudPagedLod.valid())
		{
			vNextLevelChidren.emplace_back(pPointCloudPagedLod);
		}
	}
	return vNextLevelChidren;
}

void CPointCloudpagedLod::ClearLoadFile(void)
{
	auto vIter = _children.begin();
	auto vEndIter = _children.end();
	while (vIter != vEndIter)
	{
		osg::ref_ptr<osg::Node> pNode = *vIter;
		if (!pNode.valid())
		{
			++vIter;
			continue;
		}
		osg::ref_ptr<CPointCloudpagedLod> pPointCloudPagedLod =
			dynamic_pointer_cast<CPointCloudpagedLod>(pNode);
		if (pPointCloudPagedLod.valid())
		{
			pPointCloudPagedLod->ClearLoadFile();
			++vIter;
		}
		else
		{
			vIter = _children.erase(vIter);
			vEndIter = _children.end();
		}
	}
}

void CPointCloudpagedLod::ClearDatabaseRequest(bool bClearChild /* = true*/)
{
	int nChild = _children.size();
	int nRangeData = _perRangeDataList.size();
	int nChildPagedLod = 0;
	for (unsigned int i = 0; i < nChild; ++i)
	{
		osg::ref_ptr<osg::Node> pNode = _children[i];
		osg::ref_ptr<CPointCloudpagedLod> pPageLod =
			dynamic_pointer_cast<CPointCloudpagedLod>(pNode);
		if (!pPageLod)
			continue;
		++nChildPagedLod;
		if (bClearChild)
		{
			pPageLod->ClearDatabaseRequest();
		}
	}
	for (int j = nRangeData - 1; j >= nChildPagedLod; --j)
	{
		osg::ref_ptr<osg::Referenced> databaseRequest = _perRangeDataList[j]._databaseRequest;
		_perRangeDataList[j]._databaseRequest = nullptr;
	}
}

void CPointCloudpagedLod::SetNewFileName(const std::string& name) { _strNewFileName = name; }

std::string CPointCloudpagedLod::GetNewFileName() { return _strNewFileName; }

void CPointCloudpagedLod::SetOldFileName(const std::string& name)
{
	_strOldFileName = name;
	// 更新纹理数组名称
	_pTexProxyNode->removeChildren(0, _pTexProxyNode->getNumChildren()); // 重置节点
	std::string strFileName = name;
	strFileName.replace(strFileName.find(pc::data::strFilExt),
						strFileName.length(),
						pc::data::TexPostStr + pc::data::strFilExt);
	_pTexProxyNode->setFileName(0, strFileName);
}

std::string CPointCloudpagedLod::GetOldFileName() { return _strOldFileName; }
void CPointCloudpagedLod::setFileName(unsigned int childNo, const std::string& filename)
{
	expandPerRangeDataTo(childNo);
	_perRangeDataList[childNo]._filename = filename;

	// 重置颜色代理节点
	_pColorProxyNode->removeChildren(0, _pColorProxyNode->getNumChildren()); // 重置节点

	if (!filename.empty())
	{
		SetOldFileName(filename);
	}
}

void CPointCloudpagedLod::ReplaceSamplesOnOffGeometry(osg::Group* pGroup)
{
	if (nullptr == pGroup)
		return;
	size_t nCount = pGroup->getNumChildren();
	for (size_t i = 0; i < nCount; ++i)
	{
		//[问题]：剖切后视图闪烁内存急剧飙升
		//[历史原因]：当节点本身为CSamplesOnOffGeometry时，调用拷贝构造为(const
		// CSamplesOnOffGeometry& conle)，其内部深拷贝，顶点数据重复，内存飙升
		//[修改策略]：增加容错，防止重复替换
		//[修改人]：李成 2022/10/27
		osg::Node* pNode = pGroup->getChild(i);
		if (nullptr == pNode || nullptr != dynamic_cast<CSamplesOnOffGeometry*>(pNode))
			continue;

		osg::Geometry* pChildGeometry = pNode->asGeometry();
		osg::Group* pChildGroup = pNode->asGroup();
		if (nullptr != pChildGeometry)
		{
			osg::ref_ptr<CSamplesOnOffGeometry> pSamplesOnOffGeometry =
				new CSamplesOnOffGeometry(*pChildGeometry);
			pGroup->replaceChild(pChildGeometry, pSamplesOnOffGeometry);
		}
		else if (nullptr != pChildGroup)
		{
			ReplaceSamplesOnOffGeometry(pChildGroup);
		}
	}
}

bool CPointCloudpagedLod::removeExpiredChildren(double expiryTime,
												unsigned int expiryFrame,
												osg::NodeList& removedChildren)
{
	if (_children.size() > _numChildrenThatCannotBeExpired)
	{
		unsigned cindex = _children.size() - 1;
		if (!_perRangeDataList[cindex]._filename.empty() &&
			_perRangeDataList[cindex]._timeStamp + _perRangeDataList[cindex]._minExpiryTime <
				expiryTime &&
			_perRangeDataList[cindex]._frameNumber + _perRangeDataList[cindex]._minExpiryFrames <
				expiryFrame)
		{
			_bIsLoad = false;
			osg::Node* nodeToRemove = _children[cindex].get();
			removedChildren.push_back(nodeToRemove);
			if (0 != _pTexProxyNode->getNumChildren())
			{
				removedChildren.push_back(_pTexProxyNode->getChild(0));
			}
			if (0 != _pColorProxyNode->getNumChildren())
			{
				removedChildren.push_back(_pColorProxyNode->getChild(0));
			}
			_pTexProxyNode->osg::Group::removeChildren(0, _pTexProxyNode->getNumChildren());
			_pColorProxyNode->osg::Group::removeChildren(0, _pColorProxyNode->getNumChildren());
			if (1 == _pTexProxyNode->getNumFileNames())
			{
				_pTexProxyNode->getDatabaseRequest(0) = 0;
			}
			if (1 == _pColorProxyNode->getNumFileNames())
			{
				_pColorProxyNode->getDatabaseRequest(0) = 0;
			}
			return Group::removeChildren(cindex, 1);
		}
	}
	return false;
}

void CPointCloudpagedLod::LockNode()
{
	//_CSC.Enter();
}

void CPointCloudpagedLod::UnLockNode()
{
	//_CSC.Leave();
}

const osg::BoundingBox& CPointCloudpagedLod::GetRealBoundingBox() const { return _realBoundingBox; }

void CPointCloudpagedLod::SetRealBoundingBox(const osg::BoundingBox& realBoundingBox)
{
	_realBoundingBox = realBoundingBox;
}

bool CPointCloudpagedLod::AttachTexCoordArray(Node* pNode)
{
	if (nullptr == pNode->asTransform()) // 关联的文件均为MatrixTransform节点
		return false;

	// 取纹理
	CGeometryNodeVisitor texVisitor;
	_pTexProxyNode->accept(texVisitor);

	// 取颜色
	CGeometryNodeVisitor colorxVisitor;
	_pColorProxyNode->accept(colorxVisitor);

	CGeometryNodeVisitor geoVisitor;
	pNode->accept(geoVisitor);

	size_t geoCount = geoVisitor._vecGeo.size();

	if (texVisitor._vecGeo.empty()) // 强制加载文件
	{
		// d3s::CLog::Info(L"强制加载纹理数组文件!（%s）", CString(GetOldFileName().c_str()));
		std::string strTexFilePath =
			_databasePath +
			(0 == _pTexProxyNode->getNumFileNames() ? "" : _pTexProxyNode->getFileName(0));
		osg::ref_ptr<osg::Node> pTmpNode = osgDB::readNodeFile(strTexFilePath);
		if (nullptr != pTmpNode)
		{
			pTmpNode->accept(texVisitor);
		}
		else
		{
			// d3s::CLog::Info(L"强制加载纹理数组失败!（%s）", CString(strTexFilePath.c_str()));
		}
	}

	// 附加纹理信息
	if (!texVisitor._vecGeo.empty())
	{
		osg::Geometry::ArrayList& texArrList = texVisitor._vecGeo.at(0)->getTexCoordArrayList();
		// ASSERT(geoCount == texArrList.size() && L"geometry与纹理数量不对应！");
		if (geoCount == texArrList.size())
		{
			for (size_t nIndex = 0; nIndex < geoCount; ++nIndex)
			{
				// [问题]：527098
				// 文件打开：文件关闭后重打开，高概率的出现界面显示点云模型会延迟3~6秒
				// [历史原因]：新的数组设定到geometry中需要共享顶点缓存对象，存在查询操作耗时
				// [修改策略]：直接将顶点数组缓存对象设置过去
				// 范岳 2022/12/21
				texArrList.at(nIndex)->setVertexBufferObject(
					geoVisitor._vecGeo.at(nIndex)->getVertexArray()->getVertexBufferObject());
				geoVisitor._vecGeo.at(nIndex)->setTexCoordArray(0, texArrList.at(nIndex));
			}
		}
		else
		{
			// d3s::CLog::Info(L"geometry与纹理数量不对应!（%s）",
			// CString(GetOldFileName().c_str()));
		}
	}
	else if (!geoVisitor._vecGeo.empty() &&
			 (geoVisitor._vecGeo.front()->getTexCoordArray(0) == nullptr)) // 检查是否已存在纹理数组
	{
		// 设置默认的纹理确保节点正常显示
		// d3s::CLog::Info(L"设置默认的纹理确保节点正常显示!（%s）",
		// CString(GetOldFileName().c_str()));
		for (size_t nIndex = 0; nIndex < geoCount; ++nIndex)
		{
			osg::ref_ptr<osg::Vec3Array> pVertexArray =
				dynamic_cast<osg::Vec3Array*>(geoVisitor._vecGeo.at(nIndex)->getVertexArray());
			if (nullptr == pVertexArray)
				continue;

			int nCount = pVertexArray->size();
			osg::ref_ptr<osg::Vec2Array> pTexCoordArray = new osg::Vec2Array();
			pTexCoordArray->resize(nCount, CPointCloudPropertyTool::GetDefaultProperty());
			geoVisitor._vecGeo.at(nIndex)->setTexCoordArray(0, pTexCoordArray);
		}
	}

	// 附加颜色
	if (colorxVisitor._vecGeo.empty()) // 强制加载文件
	{
		std::string strColorFilePath =
			_databasePath +
			(0 == _pColorProxyNode->getNumFileNames() ? "" : _pColorProxyNode->getFileName(0));

		// d3s::CLog::Info(L"强制加载颜色数组文件!（%s）", CString(strColorFilePath.c_str()));
		osg::ref_ptr<osg::Node> pTmpNode = osgDB::readNodeFile(strColorFilePath);
		if (nullptr != pTmpNode)
		{
			pTmpNode->accept(colorxVisitor);
		}
		else
		{
			// d3s::CLog::Info(L"强制加载颜色数组失败!（%s）", CString(strColorFilePath.c_str()));
		}
	}
	if (!colorxVisitor._vecGeo.empty())
	{
		// ASSERT(geoCount == colorxVisitor._vecGeo.size() && L"geometry与颜色数量不对应！");
		if (geoCount == colorxVisitor._vecGeo.size())
		{
			for (size_t nIndex = 0; nIndex < geoCount; ++nIndex)
			{
				auto tmpColorArray = colorxVisitor._vecGeo.at(nIndex)->getColorArray();
				tmpColorArray->setVertexBufferObject(
					geoVisitor._vecGeo.at(nIndex)->getVertexArray()->getVertexBufferObject());
				geoVisitor._vecGeo.at(nIndex)->setColorArray(
					colorxVisitor._vecGeo.at(nIndex)->getColorArray());
			}
		}
	}

	return true;
}

void CPointCloudpagedLod::RequestLoadTexture(osg::NodeVisitor& nv, float priority)
{
	if (0 == _pTexProxyNode->getDatabaseRequest(0) && 0 == _pTexProxyNode->getNumChildren())
	{
		std::string strTexFilePath = _databasePath + _pTexProxyNode->getFileName(0);
		CString strTempFilePath =
			CStringToolkit::convertUTF8toUTF16(strTexFilePath.c_str()).c_str();
		if (!CFileToolkit::FileExist(strTempFilePath))
		{
			// d3s::CLog::Error(L"加载纹理数组错误!（%s）",
			// CString(_pTexProxyNode->getFileName(0).c_str()));

			// 检查目录下是否存在纹理名
			CSimpleArray<CString> vFilePath;
			CFileToolkit::ReadDirectory(
				CStringToolkit::convertUTF8toUTF16(_databasePath.c_str()).c_str(),
				vFilePath,
				false);
			const std::string& strFileName = getFileName(getNumFileNames() - 1);
			CString strFileBaseName = CFileToolkit::GetFileBaseName(
				CStringToolkit::convertUTF8toUTF16(strFileName.c_str()).c_str());
			CSimpleArray<CString> vFindFile;
			for (int i = 0; i < vFilePath.GetSize(); ++i)
			{
				CString strPath = vFilePath[i];
				int nFileBaseStart = strPath.Find(strFileBaseName);
				int nTexStart = strPath.Find(L"_tex");
				if (-1 != nFileBaseStart && -1 != nTexStart)
				{
					vFindFile.Add(strPath);
				}
			}

			// 查找最大索引
			int nMaxValue = -1;
			for (int nIndex = 0; nIndex < vFindFile.GetSize(); ++nIndex)
			{
				CString strFile = vFindFile[nIndex];
				CString strFileBaseName = CFileToolkit::GetFileBaseName(strFile);
				std::vector<CString> strSplitStrings;
				CStringToolkit::SplitString(strFileBaseName, strSplitStrings, L"&");
				if (2 == strSplitStrings.size())
				{
					CString strValue = strSplitStrings.at(1).Left(
						strSplitStrings.at(1).GetLength() - pc::data::TexPostStr.length());
					int tmpValue = CStringToolkit::StrToInt(strValue);
					if (tmpValue > nMaxValue)
					{
						nMaxValue = tmpValue;
						strTexFilePath = CStringToolkit::convertUTF16toUTF8(strFile);
					}
				}
			}
			// d3s::CLog::Info("查找到最新的纹理数组! (%s)", strTexFilePath.c_str());
		}

		osg::NodePath nodePath = osg::NodePath({ _pTexProxyNode.get() });
		nv.getDatabaseRequestHandler()->requestNodeFile(strTexFilePath,
														nodePath,
														priority + 10,
														nv.getFrameStamp(),
														_pTexProxyNode->getDatabaseRequest(0),
														_pTexProxyNode->getDatabaseOptions());
	}
}

void CPointCloudpagedLodSaveLoader::ObjectLoad(osg::ref_ptr<osg::Object>& pObject,
											   osgDB::IDataStream* pDataStream)
{
	if (NULL == pDataStream)
		return;
	if (NULL == pObject.get())
	{
		pObject = new CPointCloudpagedLod;
	}

	osg::ref_ptr<CPointCloudpagedLod> pPageLod = dynamic_pointer_cast<CPointCloudpagedLod>(pObject);
	if (NULL == pPageLod)
		return;

	pPageLod->SetPointsNum((size_t)pDataStream->readLong());
	pPageLod->SetDirty(pDataStream->readBool());
	pPageLod->_bIsLoad = pDataStream->readBool();
	pPageLod->SetIDKey(CStringToolkit::UTF8ToCString(pDataStream->readString().c_str()));
	std::string strFileName = pDataStream->readString();
	pPageLod->SetOldFileName(strFileName);
	osg::Vec3d minPoint = pDataStream->readVec3d();
	osg::Vec3d maxPoint = pDataStream->readVec3d();
	osg::BoundingBox modelBoundingBox(minPoint, maxPoint);
	pPageLod->SetModelBoundingBox(modelBoundingBox);
	pPageLod->_strNewFileName = pDataStream->readString();

	// 关联文件的真实包围盒
	pPageLod->_realBoundingBox._min = pDataStream->readVec3d();
	pPageLod->_realBoundingBox._max = pDataStream->readVec3d();
}

void CPointCloudpagedLodSaveLoader::ObjectSave(osg::ref_ptr<osg::Object> pObject,
											   osgDB::IDataStream* pDataStream)
{
	if (NULL == pDataStream || NULL == pObject.get())
		return;

	osg::ref_ptr<CPointCloudpagedLod> pPageLod = dynamic_pointer_cast<CPointCloudpagedLod>(pObject);
	if (NULL == pPageLod)
		return;

	pDataStream->writeLong((long)pPageLod->GetPointsNum());
	pDataStream->writeBool(pPageLod->GetDirty());
	pDataStream->writeBool(pPageLod->GetIsLoad());
	std::string strValue;
	strValue = CStringToolkit::CStringToUTF8(pPageLod->GetIDKey());
	pDataStream->writeString(strValue);
	std::string strNameValue = pPageLod->GetOldFileName();
	pDataStream->writeString(strNameValue);
	osg::BoundingBox modelBoundingBox = pPageLod->GetModelBoundingBox();
	osg::Vec3d minPoint = modelBoundingBox._min;
	osg::Vec3d maxPoint = modelBoundingBox._max;
	pDataStream->writeVec3d(minPoint);
	pDataStream->writeVec3d(maxPoint);
	pDataStream->writeString(pPageLod->_strNewFileName);

	pDataStream->writeVec3d(pPageLod->_realBoundingBox._min);
	pDataStream->writeVec3d(pPageLod->_realBoundingBox._max);
}

bool CPointCloudpagedLodSaveLoader::IsSaveChild() { return false; }