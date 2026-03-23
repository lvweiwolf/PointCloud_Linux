#include <Segment/PCAlgorithmToolkit.h>
#include <Segment/PCQueryWrapperToolkit.h>
#include <Segment/PointCloudBoxQuery.h>
#include <Segment/PropertyVisitorCommand.h>
#include <LasFile/PointCloudToolkit.h>
#include <LasFile/PointCloudPropertyTool.h>
#include <BusinessNode/BnsProjectNode.h>

#include <include/PointCloudDistanceMeasure.h>
#include <include/ClusterManagerSet.h>
#include <include/PointCloudSegAPI.h>

#include <osgDB/WriteFile>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <atomic>

#define PARABOLA_MIN_POINT_SIZE 3
#define EUCLIDEAN_TOLERANCE 1.5		  // 聚类距离阈值
#define POINT_CLOUD_SUB_FINE 3		  // 次精细层级
#define CALCULATE_PCDIR_STEP 3		  // 计算点云包围盒方向步长
#define CALCULATE_PCDIR_VALIDINDEX -1 // 计算点云包围盒无效包围盒索引

namespace {

	// 查询树点集
	struct STreeCoordinateInfo
	{
		osg::BoundingBox box;		  // 树包围盒
		std::vector<osg::Vec3d> pnts; // 点位集合
		std::set<CString> filePaths;  // 文件集合
	};

	struct SClusterPntsMap : public d3s::ReferenceCountObj
	{
		std::map<int, STreeCoordinateInfo> pntsMap; // <树id，点位；文件>
	};

	bool FindTreePntsCallbackFun(pc::data::SVisitorCallbackParam& callbackParam)
	{
		if (nullptr == callbackParam._pAdditionalParam || nullptr == callbackParam._pTexCoordArray)
			return false;

		int nClusterID = 0;
		SClusterPntsMap* pOutTypePointMap =
			static_cast<SClusterPntsMap*>(callbackParam._pAdditionalParam.get());

		if (!CPointCloudPropertyTool::GetClusterProperty(callbackParam._pTexCoordArray,
														 callbackParam._nIndex,
														 nClusterID))
		{
			return false;
		}

		if (pOutTypePointMap->pntsMap.end() != pOutTypePointMap->pntsMap.find(nClusterID))
			pOutTypePointMap->pntsMap[nClusterID].pnts.push_back((osg::Vec3d&)callbackParam._point);

		return true;
	}

	/*
	* 	<!-- 去除草坪块状树，标准树形扫高取顶底距离，伞盖树形取包围盒对角距离
			》树顶底位置P、Q；树木有效扫高H
			》树木宽度大于 系数L(L>0) 倍PQ距离，则H等于PQ距离
			》树顶到 系数M(0<M<1) 处点个数大于 系数N(0<N<1)
	倍总点数，则H等于包围盒对角长度。反之等于PQ距离
	*/
	// 根据系数M、N计算树木信息
	void ComputeTreeInfoByCoefficientMN(const double& dM,
										const double& dN,
										const std::vector<osg::Vec3d>& pnts,
										const osg::BoundingBox& box,
										pc::data::treeInfo& treeInfo)
	{
		// 根据顶树包围盒，结合树点集合，计算更精确的顶包围盒
		osg::BoundingBox topBox = box;
		topBox.zMin() = topBox.zMax() - dM * (topBox.zMax() - topBox.zMin());
		osg::BoundingBox topBoxAccurate;
		size_t nInTopBoxPntCount = 0;

		for (const auto& iter : pnts)
		{
			if (topBox.contains(iter))
			{
				++nInTopBoxPntCount;
				topBoxAccurate.expandBy(iter);
			}
		}

		if (!topBoxAccurate.valid())
		{
			treeInfo.topPnt = box.center();
			treeInfo.topPnt.z() = box.zMax();
			treeInfo.bottomPnt = box.center();
			treeInfo.bottomPnt.z() = box.zMin();
			treeInfo.dValidSweepHeight = (treeInfo.topPnt - treeInfo.bottomPnt).length();
			return;
		}

		treeInfo.topPnt = box.center();
		treeInfo.topPnt.z() = box.zMax();
		treeInfo.bottomPnt = box.center();
		treeInfo.bottomPnt.z() = box.zMin();

		if (nInTopBoxPntCount > dN * pnts.size())
			treeInfo.dValidSweepHeight = (box._max - treeInfo.bottomPnt).length();
		else
			treeInfo.dValidSweepHeight = (treeInfo.topPnt - treeInfo.bottomPnt).length();
	}

	// 根据系数L计算树木信息
	bool ComputeTreeInfoByCoefficientL(const double& dL,
									   const osg::BoundingBox& box,
									   pc::data::treeInfo& treeInfo)
	{
		double dHeight = box.zMax() - box.zMin();
		double dWidth = std::max(box.yMax() - box.yMin(), box.xMax() - box.xMin());

		if (dWidth > dL * dHeight)
		{
			treeInfo.topPnt = treeInfo.bottomPnt = box.center();
			treeInfo.bottomPnt.z() = box.zMin();
			treeInfo.topPnt.z() = box.zMax();
			treeInfo.dValidSweepHeight = dHeight;
			return true;
		}

		return false;
	}

	d3s::share_ptr<SClusterPntsMap> GetClusterPntsMap(
		const CString& strPrjId,
		const std::map<unsigned, osg::BoundingBox>& clusterBoxMap,
		const double& dL,
		std::map<unsigned, pc::data::treeInfo>& treeInfoMap)
	{
		d3s::share_ptr<CClusterManager> pClusterMgr =
			CClusterManagerSet::GetInst()->GetClusterManager(strPrjId);

		if (nullptr == pClusterMgr)
			return nullptr;

		d3s::share_ptr<SClusterPntsMap> pClusterPntsMap = new SClusterPntsMap;
		const std::vector<osg::Vec3d> tmpPnts;

		for (const auto& iter : clusterBoxMap)
		{
			pc::data::treeInfo treeInfo;
			if (ComputeTreeInfoByCoefficientL(dL, iter.second, treeInfo))
			{
				treeInfoMap[iter.first] = treeInfo;
				continue;
			}

			auto pCluster = pClusterMgr->FindClusterByID(iter.first);
			if (nullptr == pCluster)
				continue;

			pClusterPntsMap->pntsMap[iter.first].box = iter.second;
			pClusterPntsMap->pntsMap[iter.first].pnts = tmpPnts;
			pClusterPntsMap->pntsMap[iter.first].filePaths = pCluster->GetPagedLodID();
		}

		return pClusterPntsMap;
	}
}

/////////////////////////////////////////////////////////////////////////
bool CPCAlgorithmToolkit::CheckInRangePointCloud(const pc::data::CModelNodeVector& pcElements,
												 const pc::data::tagTravelInfo& travelInfo,
												 pc::data::tagPointIndex& result)
{
	/*
		// 收集包围盒满足条件的pagedlod节点
		tagPagedLodArray arrPagedLod;
		CTbbCollectPagedLod parallel(vecPCElement, travelInfo, arrPagedLod);
		CTBBParallel::For(0, vecPCElement.size(), parallel);

		// 收集在区域中的点
		std::vector<pc::data::tagPointIndex> vecResult;
		tagPointArray arrPoint(vecResult);
		CTbbCollectPoint parallelCollectPoint(arrPagedLod._vecPagedLod,
											travelInfo,
											arrPoint,
											eCheckExistPnt);
		CTBBParallel::For(0, arrPagedLod._vecPagedLod.size(), parallelCollectPoint);

		if (vecResult.empty())
			return false;

		result = vecResult.front();
		return true;
	*/
	return false;
}



double CPCAlgorithmToolkit::GetMinDistance(osg::Vec3d& minOutContrastPoint,
										   osg::Vec3d& minOutReferencePoint,
										   const std::vector<osg::Vec3d>& vContrastPoints,
										   const std::vector<osg::Vec3d>& vReferencePoints,
										   const pc::data::EMeasureType& measureType)
{
	CPointCloudDistanceMeasure::EMeasureType type;
	switch (measureType)
	{
	case pc::data::EMeasureType::eHorizontalMeasure:
	{
		type = CPointCloudDistanceMeasure::EMeasureType::eHorizontalMeasure;
		break;
	}
	case pc::data::EMeasureType::eVerticalMeasure:
	{
		type = CPointCloudDistanceMeasure::EMeasureType::eVerticalMeasure;
		break;
	}
	case pc::data::EMeasureType::eClearanceMeasure:
	{
		type = CPointCloudDistanceMeasure::EMeasureType::eClearanceMeasure;
		break;
	}
	default:
		break;
	}

	return CPointCloudDistanceMeasure::GetMinDistance(minOutContrastPoint,
													  minOutReferencePoint,
													  type,
													  vContrastPoints,
													  vReferencePoints);
}


double CPCAlgorithmToolkit::QuickGetMinClearanceDistance(
	pc::data::MeasureResult& measureResult,
	const std::vector<osg::Vec3d>& vContrastPts,
	const std::vector<osg::Vec3d>& vReferencePts,
	double dReferenceThin,
	double dContrastThin)
{
	CPointCloudDistanceMeasure::MeasureResult result;
	double dDistance = CPointCloudDistanceMeasure::QuickGetMinClearanceDistance(result,
																				vContrastPts,
																				vReferencePts,
																				dReferenceThin,
																				dContrastThin);
	measureResult.minContrastPt = (osg::Vec3d&)result.minContrastPt;
	measureResult.minReferencePt = (osg::Vec3d&)result.minReferencePt;
	return dDistance;
}


bool CPCAlgorithmToolkit::IsValidCalTreeInfoCoefficient()
{
	double dL = 0.0;
	double dM = 0.0;
	double dN = 0.0;

	if (!CPCQueryWrapperToolkit::GetTreeInfoCoefficient(dL, dM, dN))
		return false;

	return (dL > 0 || (dM > 0.0 && dM < 1.0) || (dN > 0.0 && dN < 1.0));
}


bool CPCAlgorithmToolkit::ComputeTreeInfo(const pc::data::CModelNodeVector& pcElements,
										  const std::map<unsigned, osg::BoundingBox>& clusterBoxMap,
										  std::map<unsigned, pc::data::treeInfo>& treeInfoMap)
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

	double dL = 0.0;
	double dM = 0.0;
	double dN = 0.0;

	if (!CPCQueryWrapperToolkit::GetTreeInfoCoefficient(dL, dM, dN))
		return false;

	// 获取精细层级的 std::map<CString, pc::data::tagPagedLodFile>

	std::map<CString, pc::data::tagPagedLodFile> pageIdFilePathMap;
	for (auto pPointCloudElement : pcElements)
	{
		if (nullptr == pPointCloudElement)
			continue;

		pc::data::CModelNodeVector pagedLods;
		CPointCloudBoxQuery::GetLevelPagedLodList(pPointCloudElement,
												  pc::data::EPageLodLevel::eFine,
												  pagedLods);
		for (auto pageLOD : pagedLods)
		{
			if (!pageLOD.valid())
				continue;

			pageIdFilePathMap[pageLOD->GetId()] =
				CPCQueryWrapperToolkit::GetPagedLodModelPath(pageLOD);
		}
	}

	// 获取簇对应的pageid
	d3s::share_ptr<SClusterPntsMap> pClusterPntsMap =
		GetClusterPntsMap(bnsProject.GetID(), clusterBoxMap, dL, treeInfoMap);

	if (nullptr == pClusterPntsMap)
		return false;

	// 遍历计算
	for (auto iter : pClusterPntsMap->pntsMap)
	{
		// 读取簇所在的page文件
		for (const auto& strPageId : iter.second.filePaths)
		{
			// id对应不到文件说明已读取或者为非精细层级
			auto findIter = pageIdFilePathMap.find(strPageId);
			if (pageIdFilePathMap.end() == findIter)
				continue;

			// 遍历读点
			const pc::data::tagPagedLodFile& pageFile = findIter->second;
			auto pNode = CPointCloudToolkit::ReadNode(
				CStringToolkit::CStringToUTF8(pageFile.strPointInfoFile),
				CStringToolkit::CStringToUTF8(pageFile.strPointTexFile));

			pc::data::SVisitorInfos operatorInfo;
			operatorInfo._bAllTraversal = true;
			operatorInfo._pPropVisitorCallback = FindTreePntsCallbackFun;
			operatorInfo._pAdditionalParam = pClusterPntsMap;
			CPcPropVisitorCommand::Excute(pNode, operatorInfo);

			// 清除已读page文件
			pageIdFilePathMap.erase(strPageId);
		}

		// 根据簇点集计算树木信息
		ComputeTreeInfoByCoefficientMN(dM,
									   dN,
									   pClusterPntsMap->pntsMap[iter.first].pnts,
									   pClusterPntsMap->pntsMap[iter.first].box,
									   treeInfoMap[iter.first]);

		// 清除当前簇点集缓存
		pClusterPntsMap->pntsMap[iter.first].pnts.clear();
		pClusterPntsMap->pntsMap[iter.first].filePaths.clear();
	}

	return true;
}

void CPCAlgorithmToolkit::ComputeClusterArea(
	const pc::data::CModelNodeVector& pcElements,
	const std::vector<std::pair<int, osg::BoundingBox>>& clusterIDs,
	std::vector<double>& areas)
{
	struct BoundingBoxData : public d3s::ReferenceCountObj
	{
		virtual ~BoundingBoxData() {}

		std::atomic<double> xmin = DBL_MAX;
		std::atomic<double> ymin = DBL_MAX;
		std::atomic<double> xmax = -DBL_MAX;
		std::atomic<double> ymax = -DBL_MAX;

		int height = 0;
		int width = 0;
		int clusterID = 0;
		double resolution = 0.5;

		std::atomic<double> area = 0.0;
		std::vector<std::shared_ptr<std::atomic<bool>>> masks;

		tbb::mutex mutex;
	};

	if (pcElements.empty())
		return;

	CBnsProjectNode bnsProject;

	if (nullptr != pcElements.front())
	{
		pc::data::CModelNodePtr pProjectNode =
			pcElements.front()->GetTypeParent((int)eBnsProjectRoot);

		bnsProject = pProjectNode;
	}

	if (bnsProject.IsNull())
		return;

	if (clusterIDs.empty())
		return;

	auto fileMap = CPCQueryWrapperToolkit::GetPointCloudFileMap(pcElements);
	auto clusterMgr = CClusterManagerSet::GetInst()->GetClusterManager(bnsProject.GetID());

	if (!clusterMgr)
		return;

	// 初始化面积
	areas.resize(clusterIDs.size(), 0.0);

	for (size_t c = 0; c < clusterIDs.size(); ++c)
	{
		const auto& clusterID = clusterIDs.at(c).first;
		const auto& clusterBox = clusterIDs.at(c).second;
		auto cluster = clusterMgr->FindClusterByID(clusterID);
		if (!cluster)
			continue;

		std::set<CString> idSet = cluster->GetPagedLodID();
		std::vector<CString> idList(idSet.begin(), idSet.end());
		d3s::share_ptr<BoundingBoxData> boundParam = new BoundingBoxData();
		boundParam->clusterID = clusterID;
		boundParam->xmin = clusterBox.xMin();
		boundParam->ymin = clusterBox.yMin();
		boundParam->xmax = clusterBox.xMax();
		boundParam->ymax = clusterBox.yMax();

		// 固定分辨率
		double xres = (boundParam->xmax - boundParam->xmin) / 1000.0;
		double yres = (boundParam->ymax - boundParam->ymin) / 1000.0;
		double resolution = std::max(0.5, std::min(xres, yres));

		int numRows = std::floor((boundParam->ymax - boundParam->ymin) / resolution) + 1;
		int numColumns = std::floor((boundParam->xmax - boundParam->xmin) / resolution) + 1;

		boundParam->height = numRows;
		boundParam->width = numColumns;
		boundParam->resolution = resolution;
		boundParam->masks.resize(numRows * numColumns);

		auto CompuateAreaFunc = [&](int i) {
			const auto& file = fileMap[idList[i]];
			std::string pointFilename = CStringToolkit::CStringToUTF8(file.strPointInfoFile);
			std::string texFilename = CStringToolkit::CStringToUTF8(file.strPointTexFile);

			auto pagedLOD = CPointCloudToolkit::ReadNode(pointFilename, texFilename);
			if (!pagedLOD)
				return;

			pc::data::SVisitorInfos visitorInfos;
			visitorInfos._bAllTraversal = true;
			visitorInfos._pAdditionalParam = boundParam;
			visitorInfos._pPropVisitorCallback =
				[](pc::data::SVisitorCallbackParam& callbackParam) {
					if (nullptr == callbackParam._pTexCoordArray)
						return false;

					auto bound = (BoundingBoxData*)(callbackParam._pAdditionalParam.get());

					// 获得点所属簇
					int pointClusterID = 0;
					CPointCloudPropertyTool::GetClusterProperty(callbackParam._pTexCoordArray,
																callbackParam._nIndex,
																pointClusterID);

					if (pointClusterID != bound->clusterID)
						return false;

					const auto& point = callbackParam._point; // 遍历点

					int ri = std::floor((point.y() - bound->ymin) / bound->resolution);
					int ci = std::floor((point.x() - bound->xmin) / bound->resolution);
					ri = std::min(std::max(ri - 1, 0), bound->height - 1);
					ci = std::min(std::max(ci, 0), bound->width - 1);

					auto pixelArea = bound->resolution * bound->resolution;
					auto mask = bound->masks[ri * bound->width + ci];

					// 更新面积（加锁双重判定，如果后续存在小概率冲突问题，就考虑编译是否将两个if优化为一个，百度禁止代码段编译可解决）
					if (!mask)
					{
						tbb::mutex::scoped_lock lock(bound->mutex);
						if (!mask)
						{
							*mask = true;
							bound->area = bound->area + pixelArea;
						}
					}

					return true;
				};

			CPcPropVisitorCommand::Excute(pagedLOD, visitorInfos);
		};


		tbb::parallel_for(tbb::blocked_range<size_t>(0, idList.size()),
						  [&](const tbb::blocked_range<size_t>& r) {
							  for (size_t i = r.begin(); i != r.end(); ++i)
							  {
								  CompuateAreaFunc(i);
							  }
						  });

		areas[c] = boundParam->area; // 统计面积
	}
}

bool CPCAlgorithmToolkit::ComputeDensity(const pc::data::CModelNodeVector& pcElements,
										 const pc::data::SPolygonParam& polygonParam,
										 double& dPolygonArea,
										 double& dDensity,
										 double& dPtMaxHeightDiffer,
										 const bool bNeewHigHlight)
{
	int nErrType = 0;
	std::map<int, std::vector<osg::Vec3d>> vOutTypePointMap;

	if (!CPCQueryWrapperToolkit::QueryPoints(pcElements, polygonParam, vOutTypePointMap, nErrType))
		return false;

	std::vector<osg::Vec3d> pointList;
	for (auto iter : vOutTypePointMap)
	{
		pointList.insert(pointList.end(), iter.second.begin(), iter.second.end());
	}

	// 计算面积
	std::vector<osg::Vec3d> polygonPts;
	osg::Matrix matInverseVPW = osg::Matrix::inverse(polygonParam._matVPW);
	for (osg::Vec3d point : polygonParam._polygonPoints)
	{
		osg::Vec3d pt = point * matInverseVPW;
		polygonPts.emplace_back(pt);
	}
	// d3m::algo::surf::CSurfToolkit().ComputeArea(polygonPts, dPolygonArea);
	dPolygonArea = CalPolygonArea(polygonPts);

	// 计算密度
	dDensity = (dPolygonArea == 0 ? 0 : (double)pointList.size() / dPolygonArea);

	// 最大高差
	if (pointList.empty())
		return false;

	double dMaxZ = pointList.front().z(), dMinZ = pointList.front().z();

	for (size_t i = 0; i < pointList.size(); ++i)
	{
		double dZ = pointList[i].z();

		if (dMaxZ < dZ)
			dMaxZ = dZ;
		else if (dMinZ > dZ)
			dMinZ = dZ;
	}

	dPtMaxHeightDiffer = dMaxZ - dMinZ;

	return true;
}


bool CPCAlgorithmToolkit::CreateImage(const std::vector<std::vector<osg::Vec4>>& colorList,
									  const CString& strPath,
									  bool bSameColor,
									  int nWidth,
									  int nHeight)
{
	if (colorList.empty() || strPath.IsEmpty())
		return false;

	if (!bSameColor)
	{
		nWidth = colorList.front().size();
		nHeight = colorList.size();
	}
	if (nHeight == 0 || nWidth == 0)
		return false;

	try
	{
		osg::ref_ptr<osg::Image> pImage = new osg::Image;
		pImage->allocateImage(nWidth, nHeight, 1, GL_RGB, GL_UNSIGNED_BYTE);
		for (size_t nY = 0; nY < nHeight; ++nY)
		{
			if (colorList.at(nY).size() < nWidth)
				continue;

			for (size_t nX = 0; nX < nWidth; ++nX)
				pImage->setColor((osg::Vec4&)colorList.at(nY).at(nX), nX, nY);
		}
		std::string strImagePath = CStringToolkit::CStringToUTF8(strPath);
		osgDB::writeImageFile(*pImage.get(), strImagePath);
	}
	catch (...)
	{
	}

	return true;
}


bool CPCAlgorithmToolkit::EuclideanCluster(const pc::data::CModelNodeVector& pcElements,
										   std::vector<std::vector<osg::Vec3d>>& clusterList,
										   const unsigned& nType,
										   const osg::Matrix& vpwMatrix,
										   const std::vector<osg::Vec3d>& selectPointList,
										   const bool& bFastExecute,
										   const unsigned& nMinPointSize)
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

	pc::data::SPolygonParam polygonParam;
	polygonParam._matVPW = vpwMatrix;
	polygonParam._polygonPoints = selectPointList;
	std::map<int, std::vector<osg::Vec3d>> pointMap;
	int nErrorType = 0;
	CPCQueryWrapperToolkit::QueryPoints(pcElements, polygonParam, pointMap, nErrorType);

	if (pointMap[nType].empty())
	{
		d3s::CLog::Error(_T("拟合对象中没有nType(%d)类型，拟合失败"), nType);
		return false;
	}

	// 构造PointCloud
	IPointCloudPtr pPointCloud = d3s::pcs::CreatePointCloud();
	pPointCloud->SetGeoInfo(bnsProject.getBasePos(), bnsProject.GetEpsg());

	pPointCloud->SetSize(pointMap[nType].size());
	int nPntIndex = 0;
	for (const auto& iter : pointMap[nType])
	{
		pPointCloud->FillOsgPoint(iter, nPntIndex++);
	}

	// 聚类
	d3s::pcs::ICloudClustering* pCloudClustering = d3s::pcs::CreateCloudClustering();

	pCloudClustering->SetCloudInput(pPointCloud);
	d3s::pcs::ICloudClustering::Clusters clusters;

	if (bFastExecute)
	{
		pCloudClustering->EuclideanClusterFast(EUCLIDEAN_TOLERANCE,
											   nMinPointSize,
											   UINT_MAX,
											   clusters);
	}
	else
	{
		pCloudClustering->EuclideanCluster(EUCLIDEAN_TOLERANCE, nMinPointSize, UINT_MAX, clusters);
	}

	delete pCloudClustering;

	for (const auto& iter : clusters)
	{
		std::vector<osg::Vec3d> pointList;
		for (const auto& iterIndex : iter)
		{
			auto point = pointMap[nType].at(iterIndex);
			pointList.push_back(pointMap[nType].at(iterIndex));
		}

		clusterList.push_back(pointList);
	}
	return true;
}


bool CPCAlgorithmToolkit::GlFastEuclidean(const pc::data::CModelNodeVector& pcElements,
										  const unsigned& nType,
										  std::vector<osg::BoundingBox>& towerBoxs)
{
	d3s::CLog::Info(_T("CPCAlgorithmWrapperToolkit::GlFastEuclidean进入"));

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



	std::map<unsigned, std::vector<osg::Vec3d>> pointMap;
	CPCQueryWrapperToolkit::QueryPointsByType(pcElements,
											  std::vector<unsigned>{ nType },
											  pointMap,
											  POINT_CLOUD_SUB_FINE);
	if (pointMap.empty())
	{
		d3s::CLog::Error(
			_T("CPCAlgorithmWrapperToolkit::GlFastEuclidean退出：聚类对象中没有nType(%d)类型"),
			nType);
		return false;
	}

	// 构造PointCloud
	IPointCloudPtr pPointCloud = d3s::pcs::CreatePointCloud();
	pPointCloud->SetGeoInfo(bnsProject.getBasePos(), bnsProject.GetEpsg());
	pPointCloud->SetSize(pointMap[nType].size());

	int nPntIndex = 0;
	for (const auto& iter : pointMap[nType])
	{
		pPointCloud->FillOsgPoint(iter, nPntIndex++);
	}

	// 聚类
	d3s::pcs::ICloudClustering* pCloudClustering = d3s::pcs::CreateCloudClustering();
	pCloudClustering->SetCloudInput(pPointCloud);
	d3s::pcs::ICloudClustering::Clusters clusters;
	
	// 快速聚类最少点个数为100
	unsigned nMinPointSize = 100;
	pCloudClustering->EuclideanClusterFast(EUCLIDEAN_TOLERANCE, nMinPointSize, UINT_MAX, clusters);
	delete pCloudClustering;
	
	for (const auto& iter : clusters)
	{
		osg::BoundingBox tmpBox;
		for (const auto& iterIndex : iter)
		{
			auto point = pointMap[nType].at(iterIndex);
			tmpBox.expandBy(pointMap[nType].at(iterIndex));
		}
		towerBoxs.push_back(tmpBox);
	}

	d3s::CLog::Info(_T("CPCAlgorithmWrapperToolkit::GlFastEuclidean退出"));
	
	return true;
}

double CPCAlgorithmToolkit::CalPolygonArea(const std::vector<osg::Vec3d>& polygonPnts)
{
	boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> poly;
	for (const auto& vertex : polygonPnts)
	{
		poly.outer().push_back(
			boost::geometry::model::d2::point_xy<double>(vertex.x(), vertex.y()));
	}

	boost::geometry::correct(poly);
	return boost::geometry::area(poly);
}