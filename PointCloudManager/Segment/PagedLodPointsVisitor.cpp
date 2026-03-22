
#include <Segment/PagedLodPointsVisitor.h>
#include <Segment/PCQueryWrapperToolkit.h>

#include <BusinessNode/BnsPointCloudNode.h>
#include <LasFile/PointCloudPropertyTool.h>
#include <LasFile/PointCloudToolkit.h>

#include <include/ClassificationDef.h>


#include <Tool/FileToolkit.h>

// #include "PointCloudTool.h"
// #include "PointCloudCommomTool.h"
// #include "ClusterManagerSet.h"
// #include "ClusterManager.h"
// #include "PointCloudSegmentation/include/ClassificationDef.h"
// #include <PCQueryWrapperToolkit.h>
//
//  植被类型索引
const int INT__VEGETATION_TYPE_INDEX = 2;
// 高植被类型索引
const int INT_HIGH_VEGETATION_TYPE_INDEX = 15;

static const pc::data::PointCloudBoundBox2D dummyBox;


CPointReadWriterVisitor::CPointReadWriterVisitor(CPointReadWriter* pWriter, bool bReadWrite)
	: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
	  _pWriter(pWriter),
	  _bReadWrite(bReadWrite),
	  _bPolygon(false)
{
	_pClusterManager = CClusterManagerSet::GetInst()->GetClusterManager(_pWriter->_param._strPrjId);
}

CPointReadWriterVisitor::~CPointReadWriterVisitor() {}

void CPointReadWriterVisitor::SetPointCloudIndividualClass(
	osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
	size_t texIndex,
	size_t pointCloudIndex)
{
	if (nullptr == pTexCoordArray || nullptr == _pWriter ||
		nullptr == _pWriter->_param._pPointCloud)
		return;
	uint32_t nType = 0;
	if (!CPointCloudPropertyTool::GetSegmentProperty(pTexCoordArray, texIndex, nType))
		return;
	if (_pWriter->_param._vegetationTypes.end() != _pWriter->_param._vegetationTypes.find(nType))
		_pWriter->_param._pPointCloud->SetClassification(pointCloudIndex,
														 d3s::pcs::Classification::eHighVegetation);
	else if (nType == _pWriter->_param._nGroundType)
		_pWriter->_param._pPointCloud->SetClassification(pointCloudIndex,
														 d3s::pcs::Classification::eGround);
}

void CPointReadWriterVisitor::WriteTreeIndividual(const osg::Vec3& point,
												  osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
												  size_t texIndex,
												  size_t pointCloudIndex)
{
	if (nullptr == pTexCoordArray || nullptr == _pWriter ||
		nullptr == _pWriter->_param._pPointCloud)
		return;
	osg::Vec3 tmpPnt = point;
	tmpPnt.z() = _pWriter->_boundingBox.center().z();
	if (!_pWriter->_boundingBox.contains(tmpPnt))
		return;

	// 单木ID
	unsigned int treeId = _pWriter->_param._pPointCloud->GetTreeId(pointCloudIndex);
	if (treeId > 0)
		CreateCluster(pTexCoordArray, texIndex, treeId, point);
}


void CPointReadWriterVisitor::WriteSetSegment(const osg::Vec3& screenPoint,
											  osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
											  size_t texIndex,
											  size_t pointCloudIndex)
{
	if (nullptr == pTexCoordArray || nullptr == _pWriter ||
		nullptr == _pWriter->_param._pPointCloud)
		return;
	if (_bPolygon && !CPointCloudToolkit::PtInPolygon(screenPoint, _pWriter->_vecSelectPoints))
		return;
	if (CPointCloudPropertyTool::IsHide(pTexCoordArray, texIndex, std::vector<unsigned>{}))
		return;

	// 保存原分类
	uint32_t nCurType = -1;
	CPointCloudPropertyTool::GetSegmentProperty(pTexCoordArray, texIndex, nCurType, false);

	// 转换分类类型
	uint32_t nType = _pWriter->_param._pPointCloud->GetClassification(pointCloudIndex);
	auto iter = _pWriter->_param._mapConvertType.find(nType);
	if (iter != _pWriter->_param._mapConvertType.end())
		nType = iter->second;

	// 隐藏噪点
	auto denoiseCfg = CPCQueryWrapperToolkit::GetDenoiseParam();
	if (_pWriter->_param._denoiseIndexSet.end() !=
			_pWriter->_param._denoiseIndexSet.find(pointCloudIndex) &&
		denoiseCfg.notDenoiseTypes.end() == denoiseCfg.notDenoiseTypes.find(nType))
	{
		CPointCloudPropertyTool::SetDisplayEnableProperty(pTexCoordArray, texIndex, false);
	}

	// 已配置且不勾选则不显示（未配置的分类默认显示）。目前没有高植被类型，默认为植被
	auto pIter = _pWriter->_param._vecSegShowTypep.find(nType);
	if (_pWriter->_param._vecSegShowTypep.end() == pIter || pIter->second)
	{
		nType = (nType == INT_HIGH_VEGETATION_TYPE_INDEX ? INT__VEGETATION_TYPE_INDEX : nType);
		CPointCloudPropertyTool::SetSegmentProperty(pTexCoordArray, texIndex, nType, false);
	}

	// 清理指定分类所属的簇
	if (_pWriter->_param._clearTypeCluster.end() !=
		_pWriter->_param._clearTypeCluster.find(nCurType))
	{
		int nClusterID = 0;
		if (CPointCloudPropertyTool::GetClusterProperty(pTexCoordArray,
														texIndex,
														nClusterID,
														false) &&
			nClusterID > 0)
		{
			tbb::mutex::scoped_lock lock(_pWriter->_param._pClearCluster->_mutex);
			_pWriter->_param._pClearCluster->_clearCluster.insert(nClusterID);
		}
	}
}

void CPointReadWriterVisitor::ReadWriteNoBound(osg::ref_ptr<osg::Vec3Array>& vertArray,
											   osg::Matrix& matrix,
											   const osg::Vec3d& boxCenter,
											   osg::ref_ptr<osg::Vec2Array> pTexCoordArray)
{
	if (nullptr == vertArray || nullptr == pTexCoordArray || nullptr == _pWriter ||
		nullptr == _pWriter->_param._pPointCloud)
		return;
	size_t nVertArraySize = vertArray->size();
	if (_bReadWrite)
	{
		for (size_t nVertexIndex = 0; nVertexIndex < nVertArraySize; ++nVertexIndex)
		{
			osg::Vec3 point = (*vertArray)[nVertexIndex];
			point = matrix.preMult(point);
			point = point - boxCenter;
			_pWriter->_param._pPointCloud->FillOsgPoint(point,
														_pWriter->_param._nBeginIndex +
															nVertexIndex);
		}
		// 单木分割，提前设置点类型
		if (_pWriter->_param._eSegment == eTreeIndividual)
		{
			size_t nVertArraySize = vertArray->size();
			for (size_t nVertexIndex = 0; nVertexIndex < nVertArraySize; ++nVertexIndex)
			{
				SetPointCloudIndividualClass(pTexCoordArray,
											 nVertexIndex,
											 _pWriter->_param._nBeginIndex + nVertexIndex);
			}
		}
	}
	else
	{
		for (size_t nVertexIndex = 0; nVertexIndex < nVertArraySize; ++nVertexIndex)
		{
			osg::Vec3 point = (*vertArray)[nVertexIndex];
			point = matrix.preMult(point);
			osg::Vec3 screenPoint = (point + boxCenter) * _pWriter->_vpwMatrix;
			switch (_pWriter->_param._eSegment)
			{
			case eAutoSegment:
				WriteSetSegment(screenPoint,
								pTexCoordArray,
								nVertexIndex,
								_pWriter->_param._nBeginIndex + nVertexIndex);
				break;
			case eTreeIndividual:
				WriteTreeIndividual(point,
									pTexCoordArray,
									nVertexIndex,
									_pWriter->_param._nBeginIndex + nVertexIndex);
				break;
			default:
				break;
			}
		}
	}
	_pWriter->_param._nBeginIndex += vertArray->size();
}

void CPointReadWriterVisitor::apply(osg::Geometry& geometry)
{
	osg::ref_ptr<osg::Vec3Array> vertArray =
		static_cast<osg::Vec3Array*>(geometry.getVertexArray());
	if (!vertArray.valid())
		return;
	osg::ref_ptr<osg::Vec2Array> pTexCoordArray =
		static_cast<osg::Vec2Array*>(geometry.getTexCoordArray(0));

	osg::Matrix matrix = osg::computeLocalToWorld(getNodePath());
	if (_pWriter->_bBound)
	{
		size_t nCount = vertArray->size();
		for (size_t i = 0; i < nCount; i++)
		{
			auto point = (*vertArray)[i] * matrix;
			if (_pWriter->_boundBox.IsInBound(point.x(), point.y()))
			{
				ReadWriteByBound(point, pTexCoordArray, _pWriter->_param._boundBox.center(), i);
				++_pWriter->_nEndIndex;
			}
		}
	}
	else
	{
		ReadWriteNoBound(vertArray, matrix, _pWriter->_param._boundBox.center(), pTexCoordArray);
	}
}

void CPointReadWriterVisitor::ReadWriteByBound(osg::Vec3& point,
											   osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
											   const osg::Vec3d& boxCenter,
											   size_t i)
{
	if (nullptr == pTexCoordArray || nullptr == _pWriter ||
		nullptr == _pWriter->_param._pPointCloud)
		return;

	if (_bReadWrite)
	{
		osg::Vec3d offectPoint = point - boxCenter;
		_pWriter->_param._pPointCloud->FillOsgPoint(offectPoint, _pWriter->_nEndIndex);
		// 单木分割，提前设置点类型
		if (_pWriter->_param._eSegment == eTreeIndividual)
			SetPointCloudIndividualClass(pTexCoordArray, i, _pWriter->_nEndIndex);
	}
	else
	{
		osg::Vec3d offectPoint = point + boxCenter;
		osg::Vec3 screenPoint = offectPoint * _pWriter->_vpwMatrix;
		switch (_pWriter->_param._eSegment)
		{
		case eAutoSegment:
			WriteSetSegment(screenPoint, pTexCoordArray, i, _pWriter->_nEndIndex);
			break;
		case eTreeIndividual:
			WriteTreeIndividual(point, pTexCoordArray, i, _pWriter->_nEndIndex);
			break;
		default:
			break;
		}
	}
}

void CPointReadWriterVisitor::CreateCluster(osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
											size_t nVertexIndex,
											unsigned treeId,
											const osg::Vec3& point)
{
	// 获取Page节点
	pc::data::CModelNodePtr pPage = _pWriter->_param._pLodNode;
	if (!pPage.valid() || !_pClusterManager.valid())
		return;

	// 移除点所属旧簇
	int nClusterID = -1;
	CPointCloudPropertyTool::GetClusterProperty(pTexCoordArray, nVertexIndex, nClusterID);

	d3s::share_ptr<CClusterItem> pOldClusterItem = _pClusterManager->FindClusterByID(nClusterID);
	d3s::share_ptr<CClusterItem> pClusterItem = nullptr;

	if (_pWriter->_param._pTreeClusterMap->_treeClusterMap.end() !=
		_pWriter->_param._pTreeClusterMap->_treeClusterMap.find(treeId))
	{
		pClusterItem = _pWriter->_param._pTreeClusterMap->_treeClusterMap[treeId];
		CPointCloudPropertyTool::SetClusterProperty(pTexCoordArray,
													nVertexIndex,
													pClusterItem->GetId());
	}

	{
		tbb::mutex::scoped_lock locl(_pWriter->_param._pClusterBoxMap->_mutex);

		if (nullptr != pOldClusterItem)
		{
			pOldClusterItem->RemovePagedLodID(pPage->GetId());
			if (pOldClusterItem->GetPagedLodID().empty())
				_pClusterManager->RemoveClusterItem(pOldClusterItem);
		}

		// 添加簇对应的Page id
		if (nullptr == pClusterItem)
		{
			uint32_t nType = 0;
			CPointCloudPropertyTool::GetSegmentProperty(pTexCoordArray, nVertexIndex, nType);
			pClusterItem = _pClusterManager->CreateCluster(nType, true);
			_pWriter->_param._pTreeClusterMap->_treeClusterMap[treeId] = pClusterItem;
			CPointCloudPropertyTool::SetClusterProperty(pTexCoordArray,
														nVertexIndex,
														pClusterItem->GetId());
		}

		pClusterItem->AddPagedLodID(pPage->GetId());
		_pWriter->_param._pClusterBoxMap->_clusterBoxMap[pClusterItem->GetId()].expandBy(point);
	}
}

void CPointReadWriterVisitor::SetPolygonParam()
{
	if (nullptr != _pWriter &&
		(!_pWriter->_vecSelectPoints.empty() || _pWriter->_boundingBox.valid()))
		_bPolygon = true;
}


CPointReadWriter::CPointReadWriter(const SPointReadWriterParam& param)
	: _boundBox(dummyBox), _bBound(false), _nEndIndex(param._nBeginIndex), _param(param)
{
}

CPointReadWriter::CPointReadWriter(const SPointReadWriterParam& param,
								   const pc::data::PointCloudBoundBox2D& boundBox)
	: _boundBox(boundBox), _bBound(true), _nEndIndex(param._nBeginIndex), _param(param)
{
}

void CPointReadWriter::Read()
{
	if (nullptr == _param._pLodNode)
		return;

	pc::data::tagPagedLodFile strModelPath =
		CPCQueryWrapperToolkit::GetPagedLodModelPath(_param._pLodNode);
	std::string sPointInfoFileName = CStringToolkit::CStringToUTF8(strModelPath.strPointInfoFile);
	std::string sPointTexFileName = CStringToolkit::CStringToUTF8(strModelPath.strPointTexFile);
	_pNode = CPointCloudToolkit::ReadNode(sPointInfoFileName, sPointTexFileName);

	if (!_pNode.valid())
	{
		d3s::CLog::Error(L"读取 （%s） 文件节点为空!", (LPCTSTR)strModelPath.strPointInfoFile);
		return;
	}

	CPointReadWriterVisitor filter(this, true);
	_pNode->accept(filter);
}

void CPointReadWriter::Write()
{
	CBnsPointCloudNode pPointCloudPagedLod = _param._pLodNode;
	if (pPointCloudPagedLod.IsNull())
		return;

	pc::data::tagPagedLodFile strModelPath =
		CPCQueryWrapperToolkit::GetPagedLodModelPath(_param._pLodNode);
	std::string sPointInfoFileName = CStringToolkit::CStringToUTF8(strModelPath.strPointInfoFile);
	std::string sPointTexFileName = CStringToolkit::CStringToUTF8(strModelPath.strPointTexFile);
	_pNode = CPointCloudToolkit::ReadNode(sPointInfoFileName, sPointTexFileName);

	if (!_pNode.valid())
	{
		d3s::CLog::Error("读取 （%s） 文件节点为空!", sPointInfoFileName.c_str());
		return;
	}

	CPointReadWriterVisitor filter(this, false);
	filter.SetPolygonParam();
	_pNode->accept(filter);


	// 写第零层级文件
	const std::string sTexFileName = CStringToolkit::CStringToUTF8(strModelPath.strPointInfoFile);

	if (!CPointCloudToolkit::WriteNode(_pNode, sTexFileName, CPointCloudToolkit::eTexCoord))
	{
		d3s::CLog::Error("写入 （%s） 文件失败!", sTexFileName.c_str());
		return;
	}
}

void CPointReadWriter::SetPolygonParam(const std::vector<osg::Vec3d>& vecSelectPonits,
									   const osg::BoundingBox& boundingBox,
									   const osg::Matrix& vpwMatrix)
{
	_vecSelectPoints = vecSelectPonits;
	_vpwMatrix = vpwMatrix;
	_boundingBox = boundingBox;
}
