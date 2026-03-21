#include <Segment/PCQueryWrapperToolkit.h>
#include <Segment/PointCloudBoxQuery.h>
#include <Segment/PropertyVisitorCommand.h>
#include <LasFile/PointCloudToolkit.h>
#include <LasFile/PointCloudPropertyTool.h>
#include <BusinessNode/PCNodeType.h>
#include <BusinessNode/BnsProjectNode.h>
#include <BusinessNode/BnsPointCloudNode.h>

#include <include/ClusterManagerSet.h>

#include <Eigen/Core>

#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <osg/ComputeBoundsVisitor>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#define MAX_TYPE 100

pc::data::SDenoiseCfg ReadDenoiseParam()
{
	pc::data::SDenoiseCfg denoiseLevel;
	denoiseLevel.notDenoiseTypes.insert(4);
	denoiseLevel.notDenoiseTypes.insert(5);

	pc::data::SDenoiseParam param1;
	param1._dRadius = 3;
	param1._nMinPtSize = 100;
	denoiseLevel.params.push_back(param1);

	pc::data::SDenoiseParam param2;
	param2._dRadius = 3;
	param2._nMinPtSize = 150;
	denoiseLevel.params.push_back(param2);

	pc::data::SDenoiseParam param3;
	param3._dRadius = 2;
	param3._nMinPtSize = 200;
	denoiseLevel.params.push_back(param3);

	pc::data::SDenoiseParam param4;
	param4._dRadius = 2;
	param4._nMinPtSize = 250;
	denoiseLevel.params.push_back(param4);

	pc::data::SDenoiseParam param5;
	param5._dRadius = 1;
	param5._nMinPtSize = 300;
	denoiseLevel.params.push_back(param5);
	return denoiseLevel;
}

pc::data::SDenoiseCfg CPCQueryWrapperToolkit::GetDenoiseParam()
{
	static pc::data::SDenoiseCfg denoiseLevel = ReadDenoiseParam();
	return denoiseLevel;
}

pc::data::tagPagedLodFile CPCQueryWrapperToolkit::GetPagedLodModelPath(
	pc::data::CModelNodePtr pPagedLod)
{
	CBnsPointCloudNode pPointCloudPagedLod = pPagedLod;
	if (pPointCloudPagedLod.IsNull())
		return {};

	CString strPointInfoFileName = pPointCloudPagedLod.getFileName();
	if (strPointInfoFileName.IsEmpty())
		return {};
	CString strPointInfoFilePath = pPointCloudPagedLod.getDatabasePath();
	strPointInfoFilePath.TrimRight(L"/");
	strPointInfoFilePath.TrimRight(L"\\");
	strPointInfoFilePath += L"/";
	strPointInfoFilePath += strPointInfoFileName;

	// strTexFileName.replace(strTexFileName.find(pc::data::strFilExt), strTexFileName.length(),
	// pc::data::TexPostStr + pc::data::strFilExt);
	std::string strTexFileName = CStringToolkit::CStringToUTF8(strPointInfoFileName);
	strTexFileName.replace(strTexFileName.find(pc::data::strFilExt),
						   strTexFileName.length(),
						   pc::data::TexPostStr + pc::data::strFilExt);
	CString strPointTexFilePath = pPointCloudPagedLod.getDatabasePath();
	strPointTexFilePath.TrimRight(L"/");
	strPointTexFilePath.TrimRight(L"\\");
	strPointTexFilePath += L"/";
	strPointTexFilePath += CStringToolkit::UTF8ToCString(strTexFileName);

	return { strPointInfoFilePath, strPointTexFilePath };
}

void CPCQueryWrapperToolkit::QueryBoundingBoxList(
	const pc::data::CModelNodeVector& pcElements,
	const osg::Matrix& vpwMatrix,
	const std::vector<osg::Vec3d>& vSelectPoints,
	pc::data::PointCloudBoundToPointNum& boundToPointNum,
	pc::data::CModelNodeVector& vecResult)
{
	if (pcElements.size() == 0)
		return;

	for (auto pcElement : pcElements)
	{
		CBnsPointCloudNode bnsPcNode = pcElement;
		if (bnsPcNode.IsNull())
			continue;

		if (vSelectPoints.size() == 0 ||
			CPointCloudToolkit::JudgePolygonInModel(bnsPcNode.GetModelBoundingBox(),
													vpwMatrix,
													vSelectPoints))
		{
			vecResult.push_back(pcElement);
		}
	}

	if (vecResult.size() == 0)
		return;

	// 获取最精细级pagedlod节点
	std::vector<pc::data::CModelNodePtr> leafPagedLodList;
	for (size_t i = 0; i < vecResult.size(); ++i)
	{
		pc::data::CModelNodePtr pPointCloudElement = vecResult[i];
		CPointCloudBoxQuery::GetLevelPagedLodList(pPointCloudElement,
												  pc::data::JINGXI_LEVEL,
												  leafPagedLodList);
	}

	// 获取包围盒列表
	if (leafPagedLodList.empty())
		return;

	pc::data::PointCloudBoundBox2DVector boundBoxVecTemp;
	for (auto& leafPagedLod : leafPagedLodList)
	{
		CBnsPointCloudNode bnsPcNode = leafPagedLod;
		if (bnsPcNode.IsNull())
			continue;

		const osg::BoundingBox& boundingBox = bnsPcNode.GetModelBoundingBox();
		if (vSelectPoints.size() > 0 &&
			!CPointCloudToolkit::JudgePolygonInModel(boundingBox, vpwMatrix, vSelectPoints))
			continue;
		pc::data::PointCloudBoundBox2D boundBox(boundingBox.xMin(),
												boundingBox.xMax(),
												boundingBox.yMin(),
												boundingBox.yMax());

		// 计算点数量
		auto findIter = boundToPointNum.find(boundBox);
		if (boundToPointNum.end() == findIter)
		{
			boundToPointNum[boundBox] = bnsPcNode.GetPointsNum();
		}
		else
		{
			boundToPointNum[boundBox] += bnsPcNode.GetPointsNum();
		}
	}
}

void CPCQueryWrapperToolkit::QueryBoundingBoxList(
	const pc::data::CModelNodeVector& pcElements,
	const osg::BoundingBox& boundingBox,
	pc::data::PointCloudBoundToPointNum& boundToPointNum,
	pc::data::CModelNodeVector& vecResult)
{
	if (pcElements.size() == 0)
		return;

	for (auto pcElement : pcElements)
	{
		CBnsPointCloudNode bnsPcNode = pcElement;
		if (bnsPcNode.IsNull())
			continue;

		// 构造模型包围盒投影到屏幕的二维包围盒
		osg::BoundingBox modelBox = bnsPcNode.GetModelBoundingBox();
		modelBox.zMin() = boundingBox.zMin();
		modelBox.zMax() = boundingBox.zMax();
		if (boundingBox.valid() && boundingBox.intersects(modelBox))
			vecResult.push_back(pcElement);
	}
	if (vecResult.size() == 0)
		return;

	// 获取最精细级pagedlod节点
	pc::data::CModelNodeVector leafPagedLodList;
	for (size_t i = 0; i < vecResult.size(); ++i)
	{
		CBnsPointCloudNode bnsPcNode = vecResult[i];
		CPointCloudBoxQuery::GetLevelPagedLodList(bnsPcNode,
												  pc::data::JINGXI_LEVEL,
												  leafPagedLodList);
	}
	if (leafPagedLodList.empty())
		return;

	pc::data::PointCloudBoundBox2DVector boundBoxVecTemp;
	for (const auto& leafPagedLod : leafPagedLodList)
	{
		CBnsPointCloudNode bnsPcNode = leafPagedLod;
		if (bnsPcNode.IsNull())
			continue;

		osg::BoundingBox modelBox = bnsPcNode.GetModelBoundingBox();
		osg::BoundingBox modelBox2D = modelBox;
		modelBox2D.zMin() = boundingBox.zMin();
		modelBox2D.zMax() = boundingBox.zMax();
		if (boundingBox.valid() && !boundingBox.intersects(modelBox2D))
			continue;

		pc::data::PointCloudBoundBox2D boundBox(modelBox.xMin(),
												modelBox.xMax(),
												modelBox.yMin(),
												modelBox.yMax());
		// 计算点数量
		auto findIter = boundToPointNum.find(boundBox);
		if (boundToPointNum.end() == findIter)
			boundToPointNum[boundBox] = bnsPcNode.GetPointsNum();
		else
			boundToPointNum[boundBox] += bnsPcNode.GetPointsNum();
	}
}

struct SClusterBox : public d3s::ReferenceCountObj
{
	SClusterBox(std::map<unsigned, osg::BoundingBox>& clusterBoxMap,
				const std::set<unsigned>& intTypeFind,
				d3s::share_ptr<CClusterManager> pClusterMgr)
		: _clusterBoxMap(clusterBoxMap), _intTypeFind(intTypeFind), _pClusterMgr(pClusterMgr)
	{
	}

	std::map<unsigned, osg::BoundingBox>& _clusterBoxMap; // 簇对应的包围盒
	std::set<unsigned> _intTypeFind;					  // 仅在指定分类下查询
	d3s::share_ptr<CClusterManager> _pClusterMgr;		  // 簇管理器
	tbb::mutex _mutex;									  // 互斥锁
};

bool QueryClusterBox(pc::data::SVisitorCallbackParam& callbackParam)
{
	if (nullptr == callbackParam._pTexCoordArray)
		return false;

	int nClusterID = 0;
	CPointCloudPropertyTool::GetClusterProperty(callbackParam._pTexCoordArray,
												callbackParam._nIndex,
												nClusterID);
	if (nClusterID <= 0)
		return false;
	auto pData = static_cast<SClusterBox*>(callbackParam._pAdditionalParam.get());
	if (nullptr == pData || nullptr == pData->_pClusterMgr)
		return false;

	// 若提供指定分类，则只取对应分类所属簇
	uint32_t nType = 0;
	CPointCloudPropertyTool::GetSegmentProperty(callbackParam._pTexCoordArray,
												callbackParam._nIndex,
												nType);

	if (!pData->_intTypeFind.empty() &&
		pData->_intTypeFind.end() == pData->_intTypeFind.find(nType))
	{
		return false;
	}

	if (nullptr == pData->_pClusterMgr->FindClusterByID(nClusterID))
	{
		return false;
	}

	{
		tbb::mutex::scoped_lock lock(pData->_mutex);
		pData->_clusterBoxMap[nClusterID].expandBy(callbackParam._point);
	}

	return true;
}


bool CPCQueryWrapperToolkit::QueryClusterByBox(const pc::data::CModelNodeVector& pcElements,
											   const osg::BoundingBox& boundingBox,
											   std::map<unsigned, osg::BoundingBox>& clusterBoxMap,
											   const std::set<unsigned>& intTypeFind)
{
	if (pcElements.empty())
		return false;

	CBnsProjectNode bnsProject;

	if (nullptr != pcElements.front())
	{
		pc::data::CModelNodePtr pProjectNode =
			pcElements.front()->GetTypeParent((int)eBnsProjectRoot);

		bnsProject = pProjectNode;
	}

	if (bnsProject.IsNull())
		return false;

	pc::data::CModelNodeVector vPagedLods;
	for (auto& pcElement : pcElements)
	{
		if (nullptr == pcElement)
			continue;

		CPointCloudBoxQuery::GetLevelPagedLodList(pcElement,
												  CPointCloudBoxQuery::nAllLevel,
												  vPagedLods);
	}

	d3s::share_ptr<CClusterManager> clusterManager =
		CClusterManagerSet::GetInst()->GetClusterManager(bnsProject.GetID());

	if (!clusterManager.valid())
		return false;


	tbb::parallel_for(tbb::blocked_range<size_t>(0, vPagedLods.size()),
					  [&](const tbb::blocked_range<size_t>& r) {
						  for (size_t i = r.begin(); i != r.end(); ++i)
						  {
							  auto modelPath = GetPagedLodModelPath(vPagedLods.at(i));

							  std::string pointInfoFilename =
								  CStringToolkit::CStringToUTF8(modelPath.strPointInfoFile);
							  std::string pointTexFilename =
								  CStringToolkit::CStringToUTF8(modelPath.strPointTexFile);

							  auto node =
								  CPointCloudToolkit::ReadNode(pointInfoFilename, pointTexFilename);

							  if (!node.valid())
								  return;

							  osg::ComputeBoundsVisitor cbv;
							  node->accept(cbv);
							  osg::BoundingBox bbox = cbv.getBoundingBox();
							  bbox.zMin() = boundingBox.zMin();
							  bbox.zMax() = boundingBox.zMax();

							  if (!boundingBox.intersects(bbox))
								  return;

							  pc::data::SVisitorInfos visitorInfos;
							  visitorInfos._pPropVisitorCallback = QueryClusterBox;
							  visitorInfos._pAdditionalParam =
								  new SClusterBox(clusterBoxMap, intTypeFind, clusterManager);
							  visitorInfos._visitorInfo._boundingBox = boundingBox;
							  visitorInfos._visitorType = pc::data::SVisitorInfos::eBoundingBoxXY;

							  CPcPropVisitorCommand::Excute(node, visitorInfos);
						  }
					  });

	return true;
}


#if 0
bool CPCQueryWrapperToolkit::QueryClusterByPolygn(
	const pc::data::CModelNodeVector& pcElements,
	const std::vector<osg::Vec3d>& polygnPnts,
	std::map<unsigned, osg::BoundingBox>& clusterBoxMap,
	const std::set<unsigned>& intTypeFind)
{
	if (pcElements.empty())
		return false;
	
	osg::BoundingBox box;
	for (const auto& iter : polygnPnts)
		box.expandBy(iter);

	std::vector<pc::data::tagPagedLodFile> filePaths;
	for (const auto& pPointCloudElement : pcElements)
	{
		std::vector<osg::ref_ptr<CPointCloudpagedLod>> pageLods;
		CPointCloudBoxQuery::GetLevelPagedLodList(pPointCloudElement,
												  CPointCloudBoxQuery::nAllLevel,
												  pageLods);
		for (auto pPagedLod : pageLods)
		{
			box.zMax() = box.zMin() = pPagedLod->GetRealBoundingBox().zMax();
			if (nullptr == pPagedLod || !box.intersects(pPagedLod->GetRealBoundingBox()))
				continue;
			filePaths.push_back(PointCloudTool::GetPagedLodModelPath(pPagedLod));
		}
	}

	d3s::share_ptr<CClusterManager> pClusterMgr;
	if (nullptr != pcElements.front())
		pClusterMgr =
			CClusterManagerSet::GetInst()->GetClusterManager(pcElements.front()->GetPrjID());
	if (nullptr == pClusterMgr)
		return false;

	pc::data::SVisitorInfos visitorInfos;
	visitorInfos._pPropVisitorCallback = QueryClusterBox;
	visitorInfos._pAdditionalParam = new SClusterBox(clusterBoxMap, intTypeFind, pClusterMgr);
	visitorInfos._visitorInfo._coarsePolygonPoints = polygnPnts;
	visitorInfos._visitorType = pc::data::SVisitorInfos::ePolygonXY;
	CTbbFind parallel(filePaths, visitorInfos);
	CTBBParallel::For(0, filePaths.size(), parallel);
	return true;
}

#endif

void CPCQueryWrapperToolkit::GetEigenVectors(osg::Vec3d& outMajorPoint,
											 osg::Vec3d& outMiddlePoint,
											 osg::Vec3d& outMinorPoint,
											 const std::vector<osg::Vec3d>& vPoints)
{
	if (vPoints.empty())
	{
		return;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pCloud->points.reserve(vPoints.size());
	for (auto& point : vPoints)
	{
		pCloud->points.emplace_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
	}
	pCloud->width = pCloud->points.size();
	pCloud->height = 1;
	Eigen::Vector3f majorVector, middleVector, minorVector;
	{
		Eigen::Vector4f centroid;
		Eigen::Matrix3f covariance;
		pcl::compute3DCentroid(*pCloud, centroid);
		pcl::computeCovarianceMatrixNormalized(*pCloud, centroid, covariance);
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance,
															  Eigen::ComputeEigenvectors);
		Eigen::Matrix3f eigenVectors = solver.eigenvectors();
		Eigen::Vector3f eigenValues = solver.eigenvalues();
		Eigen::Vector3f majorAxis;
		Eigen::Vector3f middleAxis;
		Eigen::Vector3f minorAxis;
		float fMajorValue = 0.0f;
		float fMiddleValue = 0.0f;
		float fMinorValue = 0.0f;
		{
			unsigned int nTemp = 0;
			unsigned int nMajorIndex = 0;
			unsigned int nMiddleIndex = 1;
			unsigned int nMinorIndex = 2;
			if (eigenValues.real()(nMajorIndex) < eigenValues.real()(nMiddleIndex))
			{
				nTemp = nMajorIndex;
				nMajorIndex = nMiddleIndex;
				nMiddleIndex = nTemp;
			}
			if (eigenValues.real()(nMajorIndex) < eigenValues.real()(nMinorIndex))
			{
				nTemp = nMajorIndex;
				nMajorIndex = nMinorIndex;
				nMinorIndex = nTemp;
			}
			if (eigenValues.real()(nMiddleIndex) < eigenValues.real()(nMinorIndex))
			{
				nTemp = nMinorIndex;
				nMinorIndex = nMiddleIndex;
				nMiddleIndex = nTemp;
			}
			fMajorValue = eigenValues.real()(nMajorIndex);
			fMiddleValue = eigenValues.real()(nMiddleIndex);
			fMinorValue = eigenValues.real()(nMinorIndex);
			majorAxis = eigenVectors.col(nMajorIndex).real();
			middleAxis = eigenVectors.col(nMiddleIndex).real();
			minorAxis = eigenVectors.col(nMinorIndex).real();
			majorAxis.normalize();
			middleAxis.normalize();
			minorAxis.normalize();
			float det = majorAxis.dot(middleAxis.cross(minorAxis));
			if (det <= 0.0f)
			{
				majorAxis(0) = -majorAxis(0);
				majorAxis(1) = -majorAxis(1);
				majorAxis(2) = -majorAxis(2);
			}
		}
		majorVector = majorAxis;
		middleVector = middleAxis;
		minorVector = minorAxis;
	}
	outMajorPoint = osg::Vec3d(majorVector[0], majorVector[1], majorVector[2]);
	outMiddlePoint = osg::Vec3d(middleVector[0], middleVector[1], middleVector[2]);
	outMinorPoint = osg::Vec3d(minorVector[0], minorVector[1], minorVector[2]);
}

double CPCQueryWrapperToolkit::CalcArea(const osg::BoundingBox& boundBox, EPlaneType eType)
{
	if (boundBox.xMin() > boundBox.xMax() || boundBox.yMin() > boundBox.yMax() ||
		boundBox.zMin() > boundBox.zMax())
		return 0.0;

	double dArea = 0.0;
	switch (eType)
	{
	case CPCQueryWrapperToolkit::eXYPlane:
	{
		dArea = (boundBox.xMax() - boundBox.xMin()) * (boundBox.yMax() - boundBox.yMin());
	}
	break;
	case CPCQueryWrapperToolkit::eXZPlane:
	{
		dArea = (boundBox.xMax() - boundBox.xMin()) * (boundBox.zMax() - boundBox.zMin());
	}
	break;
	case CPCQueryWrapperToolkit::eYZPlane:
	{
		dArea = (boundBox.yMax() - boundBox.yMin()) * (boundBox.zMax() - boundBox.zMin());
	}
	break;
	default:
		break;
	}

	return dArea;
}
