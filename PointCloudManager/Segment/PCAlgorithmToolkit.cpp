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

#include <numeric>
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


	void GetArea(const std::vector<osg::Vec3d>& vTileCentroids, std::vector<osg::Vec3d>& outPnts)
	{
		double dxMin = DBL_MAX, dyMin = DBL_MAX;
		double dxMax = -DBL_MAX, dyMax = -DBL_MAX;
		osg::Vec3d xMin, yMin, xMax, yMax;
		for (auto& pnt : vTileCentroids)
		{
			if (pnt.x() < dxMin)
			{
				dxMin = pnt.x();
				xMin = pnt;
			}
			else if (pnt.x() > dxMax)
			{
				dxMax = pnt.x();
				xMax = pnt;
			}

			if (pnt.y() < dyMin)
			{
				dyMin = pnt.y();
				yMin = pnt;
			}
			else if (pnt.y() > dyMax)
			{
				dyMax = pnt.y();
				yMax = pnt;
			}
		}
		outPnts.emplace_back(xMin);
		outPnts.emplace_back(xMax);
		outPnts.emplace_back(yMin);
		outPnts.emplace_back(yMax);
	}


	static bool VectorVaild(std::vector<int>& vTileIndices)
	{
		bool bResult = false;
		for (auto var : vTileIndices)
		{
			if (var != CALCULATE_PCDIR_VALIDINDEX)
			{
				bResult = true;
				break;
			}
		}

		return bResult;
	}

	void CalculateBoxDir(std::vector<osg::BoundingBox>& boundingBoxs,
						 const osg::Vec3d& centerPt,
						 std::vector<std::pair<osg::BoundingBox, osg::Vec3>>& pcBoxDir)
	{
		std::vector<osg::Vec3d> boxCenters;
		osg::Vec3d totalCenter;
		for (const auto& box : boundingBoxs)
		{
			boxCenters.emplace_back(box.center());
			totalCenter += box.center();
		}

		totalCenter /= boundingBoxs.size();
		osg::Vec3d major, middle, minor;
		CPCQueryWrapperToolkit::GetEigenVectors(major, middle, minor, boxCenters);

		if ((totalCenter - centerPt) * major < 0)
			major = -major;

		major.normalize();
		osg::BoundingBox tempBox;

		for (const auto& iter : boundingBoxs)
			tempBox.expandBy(iter);

		pcBoxDir.push_back(std::pair<osg::BoundingBox, osg::Vec3>(tempBox, major));

		// 排序获取末尾坐标
		std::sort(boundingBoxs.begin(),
				  boundingBoxs.end(),
				  [&](const osg::BoundingBox& A, const osg::BoundingBox& B) {
					  const auto& lhs = A.center();
					  const auto& rhs = B.center();
					  osg::Vec3 d1 = lhs - centerPt;
					  double l1 = d1.length();
					  d1.normalize();
					  double project1 = major * d1 * l1;
					  osg::Vec3 d2 = rhs - centerPt;
					  double l2 = d2.length();
					  d2.normalize();
					  double project2 = major * d2 * l2;
					  return project1 < project2;
				  });
	}



	void CalculateDir(std::vector<CPCAlgorithmToolkit::SPowerParam>& towerDirPts,
					  std::vector<CPCAlgorithmToolkit::SPowerParam>& towerPts,
					  const CPCAlgorithmToolkit::SPowerParam& tmpPt,
					  const double& dMaxSpan)
	{
		// 查找当前点距离最近的下一个点
		double dMinSpan = DBL_MAX;
		size_t dMinSpanIndex = 0;
		size_t nCount = towerPts.size();

		for (size_t i = 0; i < towerPts.size(); ++i)
		{
			double dSpan = (tmpPt._towerPt - towerPts[i]._towerPt).length();
			if (dMinSpan > dSpan)
			{
				dMinSpan = dSpan;
				dMinSpanIndex = i;
			}
		}

		// 若存在下一个点，则递归查找后续
		if (dMinSpan < dMaxSpan && dMinSpanIndex < towerPts.size())
		{
			towerDirPts.push_back(towerPts[dMinSpanIndex]);
			towerPts.erase(towerPts.begin() + dMinSpanIndex);

			if (!towerPts.empty())
			{
				CalculateDir(towerDirPts, towerPts, towerDirPts.back(), dMaxSpan);
			}
		}
	}


	// 对数组的添加操作进行加锁 用于多线程
	struct tagPagedLodArray
	{
		void AddArray(const std::vector<osg::ref_ptr<CPointCloudpagedLod>>& vecPagedLod)
		{
			tbb::mutex::scoped_lock lock(_mutex);
			_vecPagedLod.insert(_vecPagedLod.end(), vecPagedLod.begin(), vecPagedLod.end());
		}

		std::vector<osg::ref_ptr<CPointCloudpagedLod>> _vecPagedLod;
		tbb::mutex _mutex;
	};

	// 收集节点数据
	class CTbbCollectPagedLod
	{
	public:
		CTbbCollectPagedLod(const pc::data::CModelNodeVector& pcElements,
							const pc::data::tagTravelInfo& travelInfo,
							tagPagedLodArray& arrPagedLod)
			: _vecPCElement(pcElements), _travelInfo(travelInfo), _arrPagedLod(arrPagedLod)
		{
			// 根据直线与半径参数构建包围盒
			d3s::platform::data::Vec3 vecDir = _travelInfo.endPnt - _travelInfo.beginPnt;
			d3s::platform::data::Vec3 center = (_travelInfo.endPnt + _travelInfo.beginPnt) / 2;
			d3s::share_ptr<d3s::element::CConeElement> pConeElement =
				new d3s::element::CConeElement(d3s::platform::data::Vec3(0, 0, 0),
											   _travelInfo.fRadius,
											   _travelInfo.fRadius,
											   vecDir.length() + _travelInfo.fRadius * 2);
			pConeElement->Build();

			// 进行旋转+平移
			vecDir.normalize();
			d3s::platform::data::Matrix matrix =
				d3s::platform::data::Matrix::rotate(d3s::platform::data::Vec3(0, 1, 0), vecDir) *
				d3s::platform::data::Matrix::translate(center);
			pConeElement->SetMatrix(matrix);


			_boundingboxTravel = (osg::BoundingBox&)pConeElement->getBoundingBox();
		};

		void operator()(const int& i) const
		{
			osg::ref_ptr<osg::Node> pNode = CElementNodeHandler::GetSceneNode(_vecPCElement.at(i));
			if (nullptr == pNode)
				return;

			CPagedLodVisitor nodeVisitor(_travelInfo, _boundingboxTravel);
			pNode->accept(nodeVisitor);
			_arrPagedLod.AddArray(nodeVisitor.vecPagedLod);
		}

	private:
		const std::vector<d3s::share_ptr<CPointCloudElement>>& _vecPCElement;
		const pc::data::tagTravelInfo& _travelInfo;
		tagPagedLodArray& _arrPagedLod;
		osg::BoundingBox _boundingboxTravel;
	};

	// 对于满足条件的节点进行遍历找点
	struct tagPointArray
	{
		tagPointArray(std::vector<pc::data::tagPointIndex>& vecResult)
			: _vecResult(vecResult) {

			  };

		void AddArray(const std::vector<pc::data::tagPointIndex>& vecPoint)
		{
			toolkit::CCriticalSectionSync cs(_CSC);
			_vecResult.insert(_vecResult.end(), vecPoint.begin(), vecPoint.end());
		};

		bool IsEmpty()
		{
			toolkit::CCriticalSectionSync cs(_CSC);
			return _vecResult.empty();
		};

	private:
		std::vector<pc::data::tagPointIndex>& _vecResult;
		toolkit::CCriticalSectionHandle _CSC;
	};

	class CTbbCollectPoint
	{
	public:
		CTbbCollectPoint(const std::vector<osg::ref_ptr<CPointCloudpagedLod>>& vecPCElement,
						 const pc::data::tagTravelInfo& travelInfo,
						 tagPointArray& arrPoint,
						 ESearchMode eSearchMode)
			: _vecPCNode(vecPCElement),
			  _travelInfo(travelInfo),
			  _arrPoint(arrPoint),
			  _eSearchMode(eSearchMode) {};

		void operator()(const int& i) const
		{
			// 仅检查是否存在点
			if ((_eSearchMode == eCheckExistPnt) && !_arrPoint.IsEmpty())
			{
				return;
			}

			// 检查节点是否需要加载
			unsigned int nFilePos = _vecPCNode.at(i)->getNumPriorityScales();
			unsigned int nChildNum = _vecPCNode.at(i)->getNumChildren();
			osg::ref_ptr<osg::Node> pTmpNode = nullptr;
			if (nFilePos == nChildNum)
			{
				pTmpNode = _vecPCNode.at(i)->getChild(nFilePos - 1);
			}
			else if (pc::data::eAllDataTravel == _travelInfo.eMode)
			{
				// 加载文件
				std::string pointFilePath = _vecPCNode.at(i)->getDatabasePath() +
											_vecPCNode.at(i)->getFileName(nFilePos - 1);
				std::string texFileName =
					_vecPCNode.at(i)->getDatabasePath() + _vecPCNode.at(i)->GetOldFileName();
				pTmpNode = CPointCloudToolkit::ReadNode(pointFilePath, texFileName);
			}

			if (nullptr == pTmpNode)
			{
				return;
			}

			// 遍历节点
			CPCCollectVisitor nodeVisitor(_travelInfo, _eSearchMode);
			nodeVisitor._vecPoint.reserve(_vecPCNode.at(i)->GetPointsNum()); // 扩容数组
			pTmpNode->accept(nodeVisitor);
			_arrPoint.AddArray(nodeVisitor._vecPoint);
		}

	private:
		const std::vector<osg::ref_ptr<CPointCloudpagedLod>>& _vecPCNode;
		const pc::data::tagTravelInfo& _travelInfo;
		tagPointArray& _arrPoint;
		ESearchMode _eSearchMode;
	};
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

bool CPCAlgorithmToolkit::CalculatePointCloudDir(
	const pc::data::CModelNodeVector& pcElements,
	const osg::Vec3& startPt,
	std::vector<std::pair<osg::BoundingBox, osg::Vec3>>& pcBoxDir)
{
	std::vector<osg::BoundingBox> pageLodBoxs;
	std::vector<osg::Vec3d> pageLodBoxCenters;
	for (auto& pPointCloudElement : pcElements)
	{
		pc::data::CModelNodeVector pageLODs;
		CPointCloudBoxQuery::GetLevelPagedLodList(pPointCloudElement,
												  pc::data::JINGXI_LEVEL,
												  pageLODs);
		for (auto pageLOD : pageLODs)
		{
			CBnsPointCloudNode bnsPageLOD = pageLOD;
			if (bnsPageLOD.IsNull())
				continue;


			auto box = bnsPageLOD.GetModelBoundingBox();

			box._min.z() = box._max.z() = 0;
			pageLodBoxs.emplace_back(box);
			pageLodBoxCenters.emplace_back(box.center());
		}
	}

	std::vector<osg::Vec3d> outPnts;
	GetArea(pageLodBoxCenters, outPnts);
	std::sort(outPnts.begin(), outPnts.end(), [&](const osg::Vec3d& pnt, const osg::Vec3d& anoPnt) {
		return (startPt - pnt).length() < (startPt - anoPnt).length();
	});

	osg::Vec3d firstCenter = outPnts.front();
	// 创建瓦片索引
	std::vector<int> vTileIndices(pageLodBoxs.size());
	std::iota(vTileIndices.begin(), vTileIndices.end(), 0);
	double dStep = CALCULATE_PCDIR_STEP * pageLodBoxs.front().radius();
	int nStep = CALCULATE_PCDIR_STEP / CALCULATE_PCDIR_STEP;
	std::vector<osg::BoundingBox> tmpBoxs;

	while (VectorVaild(vTileIndices))
	{
		for (size_t i = 0; i < vTileIndices.size(); ++i)
		{
			if (vTileIndices[i] == CALCULATE_PCDIR_VALIDINDEX)
				continue;
			double dDis = dStep * nStep;
			if (fabs(pageLodBoxs[vTileIndices[i]].center().x() - firstCenter.x()) > dDis ||
				fabs(pageLodBoxs[vTileIndices[i]].center().y() - firstCenter.y()) > dDis)
				continue;
			tmpBoxs.push_back(pageLodBoxs[vTileIndices[i]]);
			vTileIndices[i] = CALCULATE_PCDIR_VALIDINDEX;
		}
		++nStep;
		if (tmpBoxs.empty())
			continue;

		// 满足分辨方向的条件
		osg::BoundingBox allTmpBox;
		for (const auto& iter : tmpBoxs)
			allTmpBox.expandBy(iter);
		double xLength = allTmpBox.xMax() - allTmpBox.xMin(),
			   yLength = allTmpBox.yMax() - allTmpBox.yMin();
		if (xLength * CALCULATE_PCDIR_STEP < yLength || yLength * CALCULATE_PCDIR_STEP < xLength)
		{
			CalculateBoxDir(tmpBoxs, firstCenter, pcBoxDir);
			firstCenter = tmpBoxs.back().center();
			tmpBoxs.clear();
			nStep = 1;
		}
	}
	if (!tmpBoxs.empty())
		CalculateBoxDir(tmpBoxs, firstCenter, pcBoxDir);

	return true;
}

bool CPCAlgorithmToolkit::CalculatePowerLineDir(std::vector<SPowerParam>& towerPts,
												const osg::Vec3d& startPt,
												const double& dMaxSpan)
{
	// 杆塔数量为0,、1计算走向均无意义
	if (towerPts.empty() || towerPts.size() == 1)
		return false;

	// 查找起点位置
	size_t nStartPtIndex = 0;
	double dLength = DBL_MAX;
	size_t nCount = towerPts.size();

	for (size_t i = 0; i < nCount; ++i)
	{
		double dCurLength = (startPt - towerPts[i]._towerPt).length();
		if (dLength > dCurLength)
		{
			dLength = dCurLength;
			nStartPtIndex = i;
		}
	}

	SPowerParam tmpPt = towerPts[nStartPtIndex];
	towerPts.erase(towerPts.begin() + nStartPtIndex);

	// 按起点延伸，查询线路
	std::vector<SPowerParam> towerBeginDirPts;
	std::vector<SPowerParam> towerEndDirPts;
	CalculateDir(towerBeginDirPts, towerPts, tmpPt, dMaxSpan);
	CalculateDir(towerEndDirPts, towerPts, tmpPt, dMaxSpan);

	// 判断起始（通过所选起点在线路次序判断起始，否则通过距离）
	towerPts.clear();

	if (towerBeginDirPts.size() == 0 && towerEndDirPts.size() == 0)
	{
		towerPts.push_back(tmpPt);
	}
	else if (towerBeginDirPts.size() < towerEndDirPts.size())
	{
		towerPts.insert(towerPts.end(), towerBeginDirPts.rbegin(), towerBeginDirPts.rend());
		towerPts.push_back(tmpPt);
		towerPts.insert(towerPts.end(), towerEndDirPts.begin(), towerEndDirPts.end());
	}
	else if (towerBeginDirPts.size() > towerEndDirPts.size())
	{
		towerPts.insert(towerPts.end(), towerEndDirPts.rbegin(), towerEndDirPts.rend());
		towerPts.push_back(tmpPt);
		towerPts.insert(towerPts.end(), towerBeginDirPts.begin(), towerBeginDirPts.end());
	}
	else if ((towerBeginDirPts.back()._towerPt - tmpPt._towerPt).length() <
			 (towerEndDirPts.back()._towerPt - tmpPt._towerPt).length())
	{
		towerPts.insert(towerPts.end(), towerBeginDirPts.rbegin(), towerBeginDirPts.rend());
		towerPts.push_back(tmpPt);
		towerPts.insert(towerPts.end(), towerEndDirPts.begin(), towerEndDirPts.end());
	}
	else
	{
		towerPts.insert(towerPts.end(), towerEndDirPts.rbegin(), towerEndDirPts.rend());
		towerPts.push_back(tmpPt);
		towerPts.insert(towerPts.end(), towerBeginDirPts.begin(), towerBeginDirPts.end());
	}

	return true;
}

bool CPCAlgorithmToolkit::ApproxCluster(const std::vector<osg::Vec3d>& cluster,
										std::vector<osg::Vec3d>& curve,
										const double& dT)
{
	// ICloudSegmentationServicePtr pCloudSegmentationService =
	// 	SHARE_PTR_CAST(d3s::pcs::ICloudSegmentationService,
	// 				   GetService(SERVICE_POINTCLOUD_SEGMENTATION));
	// if (nullptr == pCloudSegmentationService.get())
	// {
	// 	d3s::CLog::Error(_T("d3s::pcs::ICloudSegmentationService服务为空"));
	// 	return false;
	// }

	if (cluster.size() < PARABOLA_MIN_POINT_SIZE)
	{
		d3s::CLog::Error(_T("拟合点个数小于3"));
		return false;
	}

	// 构造PointCloud
	IPointCloudPtr pPointCloud = d3s::pcs::CreatePointCloud();
	pPointCloud->SetSize(cluster.size());
	int nPntIndex = 0;

	for (const auto& iter : cluster)
	{
		pPointCloud->FillOsgPoint((osg::Vec3d&)iter, nPntIndex++);
	}

	// 拟合
	d3s::pcs::ICloudCurveFitting* pCloudCurveFitting = d3s::pcs::CreatePowerlineFitting();

	pCloudCurveFitting->SetCloudInput(pPointCloud);
	d3s::pcs::PARACURVE params;

	if (!pCloudCurveFitting->CurveFitting(0, params))
	{
		d3s::CLog::Error(_T("d3s::pcs::ICloudCurveFitting::CurveFitting拟合失败"));
		return false;
	}

	// 采样
	osg::Matrix rotateInverse = osg::Matrix::inverse((osg::Matrix&)params.rotate);
	for (double y = params.ymin; y < params.ymax; y += dT)
	{
		double z = params.a * y * y + params.b * y + params.c;
		curve.push_back(osg::Vec3d(params.xmean, y, z));
	}

	for (auto& point : curve)
	{
		point = rotateInverse.preMult(point);
		point += (osg::Vec3d&)params.origin;
	}

	delete pCloudCurveFitting;
	pCloudCurveFitting = nullptr;
	return true;
}

std::vector<int> CPCAlgorithmToolkit::Approx(const std::vector<osg::Vec3d>& cluster,
											 const std::vector<osg::Vec3d>& points,
											 const double& dHreshold)
{
	std::vector<int> indexList;
	// ICloudSegmentationServicePtr pCloudSegmentationService =
	// 	SHARE_PTR_CAST(d3s::pcs::ICloudSegmentationService,
	// 				   GetService(SERVICE_POINTCLOUD_SEGMENTATION));
	// if (nullptr == pCloudSegmentationService.get())
	// {
	// 	d3s::CLog::Error(_T("d3s::pcs::ICloudSegmentationService服务为空"));
	// 	return indexList;
	// }

	if (cluster.size() < PARABOLA_MIN_POINT_SIZE)
	{
		d3s::CLog::Error(_T("拟合点个数小于3"));
		return indexList;
	}

	// 构造PointCloud
	IPointCloudPtr pPointCloud = d3s::pcs::CreatePointCloud();
	pPointCloud->SetSize(cluster.size());
	int nPntIndex = 0;

	for (const auto& iter : cluster)
	{
		pPointCloud->FillOsgPoint((osg::Vec3d&)iter, nPntIndex++);
	}

	// 拟合
	d3s::pcs::ICloudCurveFitting* pCloudCurveFitting = d3s::pcs::CreatePowerlineFitting();
	pCloudCurveFitting->SetCloudInput(pPointCloud);
	d3s::pcs::PARACURVE params;
	if (!pCloudCurveFitting->CurveFitting(0, params))
	{
		d3s::CLog::Error(_T("d3s::pcs::ICloudCurveFitting::CurveFitting拟合失败"));
		return indexList;
	}

	// 判断抛物线方程拟合结果 和 点云簇的距离是否满足误差范围
	double dDistance2 = dHreshold * dHreshold;
	int nCount = points.size();
	osg::Matrix mt = (osg::Matrix&)params.rotate;
	osg::Matrix ivMt = osg::Matrixd::inverse(mt);
	for (int i = 0; i < nCount; ++i)
	{
		// 原始点和抛物线方程点之间的坐标转换
		osg::Vec3d tempPoint;
		osg::Vec3d point = (points[i] - (osg::Vec3d&)params.origin) * (osg::Matrix&)params.rotate;
		tempPoint.y() = point.y();
		double dY = point.y();
		double dZ = point.z() - ((osg::Vec3d&)params.origin).z();
		tempPoint.z() = params.a * dY * dY + params.b * dY + params.c;
		tempPoint.x() = params.xmean;

		// 抛物线点和原始点之间的坐标转换
		tempPoint = tempPoint * ivMt + (osg::Vec3d&)params.origin;

		// 判断和原始点的距离(points[i] tempPoint)
		double dPtLenth2 = (points[i] - tempPoint).length2();
		if (dPtLenth2 > dDistance2)
			continue;

		indexList.push_back(i);
	}

#if 0
	 AddDebuggerCurve(params);
#endif

	delete pCloudCurveFitting;
	pCloudCurveFitting = nullptr;
	return indexList;
}

std::vector<osg::Vec3d> CPCAlgorithmToolkit::GetPointsProjectionPlace(
	const pc::data::CModelNodeVector& pcElements,
	const std::vector<osg::Vec3d>& vecPnt)
{
	std::vector<osg::Vec3d> vecReturnPnt;

	osg::BoundingBox boundBox;
	for (auto iter : vecPnt)
		boundBox.expandBy(iter);

	pc::data::tagTravelInfo travelInfo;
	// 处理仅有一个点的情况
	if (1 == vecPnt.size())
	{
		// 向任意方向两侧延伸
		travelInfo.beginPnt = vecPnt.front() + osg::Vec3(1, 0, 0) * 0.5;
		travelInfo.endPnt = vecPnt.front() - osg::Vec3(1, 0, 0) * 0.5;
	}
	else
	{
		travelInfo.beginPnt = boundBox._min;
		travelInfo.endPnt = boundBox._max;
	}


	CBnsPointCloudNode bnsPageLOD = pcElements.front();
	travelInfo.fRadius = bnsPageLOD.GetModelBoundingBox().radius();
	travelInfo.eMode = pc::data::eAllDataTravel;
	travelInfo.bOnlyInLine = true;
	std::vector<pc::data::tagPointIndex> vecResult;
	vecResult.reserve(10000 * 600);
	CPCAlgorithmWrapperToolkit::SearchInRangePointCloud(pointEleVec, travelInfo, vecResult);

	std::vector<pc::data::tagPointIndex> vecPlaceIndex;
	for (auto& iterResult : vecResult)
	{
		uint32_t nType = iterResult._nType;
		switch (nType)
		{
		case 1: // 地面
				// case 2:	//植物
		{
			vecPlaceIndex.emplace_back(iterResult);
		}
		break;
			break;
		default:
			break;
		}
	}

	for (auto iterLine : vecPnt)
	{
		pc::Vec2 vec2Line(iterLine.x(), iterLine.y());
		pc::Vec3 tmpPnt;	  // 投影点
		float fDis = FLT_MAX; // 最近距离
		for (auto iterPlace : vecPlaceIndex)
		{
			pc::Vec2 vec2Place(iterPlace._pnt.x(), iterPlace._pnt.y());
			float fTmpDis = (vec2Line - vec2Place).length2();
			if (fTmpDis < fDis)
			{
				tmpPnt = iterPlace._pnt;
				fDis = fTmpDis;
			}
		}
		vecReturnPnt.emplace_back(tmpPnt);
	}

	return vecReturnPnt;
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

bool SearchInRangePointCloud(const pc::data::CModelNodeVector& pcElements,
							 const pc::data::tagTravelInfo& travelInfo,
							 std::vector<pc::data::tagPointIndex>& vecResult)
{
	// 收集包围盒满足条件的pagedlod节点
	tagPagedLodArray arrPagedLod;
	CTbbCollectPagedLod parallel(pcElements, travelInfo, arrPagedLod);
	CTBBParallel::For(0, pcElements.size(), parallel);

	// 收集在区域中的点
	tagPointArray arrPoint(vecResult);
	CTbbCollectPoint parallelCollectPoint(arrPagedLod._vecPagedLod,
										  travelInfo,
										  arrPoint,
										  eSearchAllPnt);
	CTBBParallel::For(0, arrPagedLod._vecPagedLod.size(), parallelCollectPoint);

	return true;
}