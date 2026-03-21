
#include <Segment/PCAutoSegmentToolkit.h>
#include <Segment/PCQueryWrapperToolkit.h>
#include <BusinessNode/PCNodeType.h>
#include <BusinessNode/BnsProjectNode.h>

#include <include/PointCloudSegAPI.h>
#include <include/ClusterManagerSet.h>

#include <Tool/LibToolkit.h>
#include <Tool/FileToolkit.h>

#include <numeric>
#include <include/Log.h>

// 自动分类包围盒最大点数
const int INT_AUTO_CLASSIFY_BOX_MAX_POINTS_SIZE = 32000000;
// 自动分类划分的包围盒投影面积最大值
const double C_AUTO_CLASSIFY_BOX_PROJECTION_AREA_MAX = 10000000;
// 单位内存块大小
const int INT_UNIT_MEMORY_BLOCK_SIZE = 1024 * 1024;
// 点云最精细层级
const int INT_POINT_CLOUD_TINY_LEVEL = 3;
// 分类包围盒高度
const double DOUBLE_BOUNDING_BOX_HEIGHT = 0.1;
// 自动分类点云拷贝进度
const int INT_PROGRESS_STEP_CLONE_POINT_CLOUD = 5;
// 自动分类进度占比
const int INT_PROGRESS_STEP_POINT_CLOUD = 90;
// 自动分类点云抽稀进度
const int INT_PROGRESS_STEP_SIMPLIFY_POINT_CLOUD = 95;
// 自动分类点云拷贝进度
const int INT_PROGRESS_STEP_REPLACE_POINT_CLOUD = 99;


void CPCAutoSegmentToolkit::GetTileInfo(const osg::BoundingBox& boundingBox,
										std::vector<osg::Vec3d> vecTowerPos,
										const pc::data::PointCloudBoundToPointNum& boundToPointNum,
										osg::Vec3d& center,
										size_t& nTotal,
										std::vector<osg::Vec3d>& vTileCentroids,
										std::vector<osg::BoundingBox>& vTileBounds,
										std::vector<size_t>& vPointSizes)
{
	// 处理标记了杆塔线路，杆塔线路包围盒点集不为空的情况
	if (vecTowerPos.size() > 1 && boundingBox.valid())
	{
		pc::data::PointCloudBoundBox2D lineDataBoundbox(boundingBox.xMin(),
														boundingBox.xMax(),
														boundingBox.yMin(),
														boundingBox.yMax());
		for (auto it = boundToPointNum.begin(); it != boundToPointNum.end(); ++it)
		{
			auto& bound2D = it->first;
			if (bound2D.IsIntersect(lineDataBoundbox))
			{
				osg::BoundingBox bound3D(bound2D._dXmin,
										 bound2D._dYmin,
										 0.0,
										 bound2D._dXmax,
										 bound2D._dYmax,
										 DOUBLE_BOUNDING_BOX_HEIGHT);

				vTileBounds.emplace_back(bound3D);
				vPointSizes.emplace_back(it->second);
				vTileCentroids.emplace_back(bound3D.center());
				center += bound3D.center();
				nTotal += it->second;
			}
		}
	}
	else
	{
		for (auto it = boundToPointNum.begin(); it != boundToPointNum.end(); ++it)
		{
			auto bound2D = it->first;
			osg::BoundingBox bound3D(bound2D._dXmin,
									 bound2D._dYmin,
									 0.0,
									 bound2D._dXmax,
									 bound2D._dYmax,
									 DOUBLE_BOUNDING_BOX_HEIGHT);

			vTileBounds.emplace_back(bound3D);
			vPointSizes.emplace_back(it->second);
			vTileCentroids.emplace_back(bound3D.center());
			center += bound3D.center();
			nTotal += it->second;
		}
	}
}

IValueBufferPtr CPCAutoSegmentToolkit::_roadVectorize = nullptr;

void CPCAutoSegmentToolkit::SegmentProgress(CAutoSegmentFileLoadSaveThread* pThread,
											const std::vector<osg::BoundingBox>& vDataBounds,
											pc::data::SegmentParam& segmentParam,
											const std::map<int, int>& mapConvertType,
											ESegmentType eSegment)
{
	if (pThread == nullptr)
		return;

	size_t nDataBoundsCount = vDataBounds.size();
	std::vector<osg::Vec3d> towerOrign = segmentParam._vecTowerPos;
	// 局部数据进行分类
	for (size_t i = 0; i < nDataBoundsCount; ++i)
	{
		const auto& bound3D = vDataBounds[i];

		// 处理杆塔包围盒偏移的情况
		segmentParam._vecTowerPos = towerOrign;
		TowerOffect(segmentParam, bound3D);

		pc::data::PointCloudBoundBox2D bound2D(bound3D.xMin(),
											   bound3D.xMax(),
											   bound3D.yMin(),
											   bound3D.yMax());


		// 获取范围内的局部点云，并进行分类
		IPointCloudPtr pPointCloud = ConvertPointCloud(pThread, bound2D);

		if (!pPointCloud.valid())
			continue;


		// 设置偏移坐标和地理坐标系信息
		pPointCloud->SetGeoInfo((osg::Vec3d&)segmentParam._offset_xyz, segmentParam._epsg);

#if 0
		{
			CString strOutputDir = _T(R"(G:\220kV\)");

			CString strName;
			strName.Format(_T("%d.las"), i + 1);

			std::string filename = CT2A(strOutputDir + strName);
			pPointCloud->Write(filename);
		}
#endif

		// 自动分类
		{
			d3s::CTimeLog timeRecord(L"第 %d 个局部点云分类耗时:", i);

			if (!SegmentInternel(segmentParam, pPointCloud.get(), eSegment))
			{
				d3s::CLog::Warn(L"第 %d 个局部点云分类失败", i);
				continue;
			}
		}

		// pPointCloud->Write("temp.las");

		pThread->SetModifyPoints(bound2D);
	}
}

void RemoveCluster(const std::set<unsigned>& clearCluter, LPCTSTR strPrj)
{
	/*auto pCluterMgr = CClusterManagerSet::GetInst()->GetClusterManager(strPrj);
	if (nullptr == pCluterMgr)
		return;
	d3s::views::txn::ITxn* pCurTxn = d3s::views::txn::CTxnManager::GetManager()->GetCurrentTxn();
	if (nullptr == pCurTxn)
		return;

	auto pCluterTxn = new CPcClusterEditTxn(CPcClusterEditTxn::EEditType::eDel);
	for (const auto& iter : clearCluter)
	{
		d3s::share_ptr<CClusterItem> pCluter = pCluterMgr->FindClusterByID(iter);
		if (nullptr == pCluter)
			continue;
		pCluterMgr->RemoveClusterItem(pCluter);
		pCluterTxn->SetClusterData(strPrj, pCluter);
	}
	pCurTxn->SetNextTxn(pCluterTxn);*/
}

void CPCAutoSegmentToolkit::Segment(const pc::data::CModelNodeVector& vPointCloudElements,
									pc::data::SegmentParam segmentParam,
									std::map<unsigned, osg::BoundingBox>& clusterBoxMap,
									const std::map<int, int>& mapConvertType,
									ESegmentType eSegmentType,
									const std::set<unsigned>& vegetationTypes,
									const unsigned& nGroundType)
{
	try
	{
		if (vPointCloudElements.size() == 0)
		{
			d3s::CLog::Warn(L"CPCAutoSegmentToolkit::Segment 点云参数为空!");
			return;
		}
		if (nullptr != vPointCloudElements.front())
		{
			pc::data::CModelNodePtr pProjectNode =
				vPointCloudElements.front()->GetTypeParent((int)eBnsProjectRoot);
			CBnsProjectNode bnsProject = pProjectNode;
			if (bnsProject.IsNull())
				return;


			segmentParam._epsg = bnsProject.GetEpsg();
			segmentParam._offset_xyz = bnsProject.getBasePos();
		}

		d3s::CTimeLog allTimeRecord(L"自动分类耗时：");

		// 计算包围框
		std::vector<osg::BoundingBox> vDataBounds;
		pc::data::CModelNodeVector vecPointCloudElement;
		ComputeCloudBounds(vPointCloudElements,
						   segmentParam,
						   INT_AUTO_CLASSIFY_BOX_MAX_POINTS_SIZE,
						   vDataBounds,
						   vecPointCloudElement,
						   eSegmentType);

		// 打开自动分类读写线程
		SSegmentThreadParam param(clusterBoxMap);
		param._vPointCloudElements = vPointCloudElements;
		param._vBoundingBoxs = vDataBounds;
		param._showTypeMap = segmentParam._showTypeMap;
		param._mapConvertType = mapConvertType;
		param._nDenoiseLevel = segmentParam._nDenoiseLevel;
		param._eSegment = eSegmentType;
		param._clearTypeCluster = segmentParam._clearTypeCluster;
		param._vegetationTypes = vegetationTypes;
		param._nGroundType = nGroundType;
		CAutoSegmentFileLoadSaveThread* pThread = new CAutoSegmentFileLoadSaveThread(param);
		pThread->SetPolygonParam((std::vector<osg::Vec3d>&)segmentParam._vecSelectPoints,
								 (osg::BoundingBox&)segmentParam._boundingBox,
								 (osg::Matrix&)segmentParam._vpwMatrix);
		pThread->start();
		SegmentProgress(pThread, vDataBounds, segmentParam, mapConvertType, eSegmentType);

		pThread->join();

		std::set<unsigned> clearCluter = pThread->GetClearCluster();
		SAFE_DELETE(pThread);


		// 替换自动分类后的元素
		/*for (int i = 0; i < vecPointCloudElement.size(); ++i)
		{
			CPointCloudCommonTool::LoadNodeWhenOldNodeLoaded(pc::data::CModelNodeVector{vecPointCloudElement[i]}
			, pc::data::CModelNodeVector{vClonePointCloudElements[i]});
			d3s::views::primitive::EditElementHandle handle(vClonePointCloudElements[i]);
			d3s::views::txn::CTxnManager::GetManager()->GetCurrentTxn()->ReplaceElement(handle,
		vecPointCloudElement[i]);
		}*/
	}
	catch (...)
	{
	}
}

bool CPCAutoSegmentToolkit::TreeIndividual(const pc::data::CModelNodeVector& pointCloudElements,
										   const osg::BoundingBox& boundingBox,
										   const std::set<unsigned>& vegetationTypes,
										   const unsigned& nGroundType,
										   std::map<unsigned, osg::BoundingBox>& clusterBoxMap)
{
	CBnsProjectNode bnsProject;

	if (pointCloudElements.size() == 0)
	{
		d3s::CLog::Warn(L"CPCAutoSegmentToolkit::TreeIndividual 点云参数为空!");
		return false;
	}

	if (nullptr != pointCloudElements.front())
	{
		pc::data::CModelNodePtr pProjectNode =
			pointCloudElements.front()->GetTypeParent((int)eBnsProjectRoot);

		bnsProject = pProjectNode;
	}

	if (bnsProject.IsNull())
		return false;

	pc::data::SegmentParam segmentParam;
	segmentParam._boundingBox = boundingBox;

	Segment(pointCloudElements,
			segmentParam,
			clusterBoxMap,
			std::map<int, int>(),
			ESegmentType::eTreeIndividual,
			vegetationTypes,
			nGroundType);

	// 自动分类分块进行，区域重合会导致簇重叠，clusterBoxMap未同步更新簇管理器
	CString strPrjId = bnsProject.GetID();

	auto clusterManager = CClusterManagerSet::GetInst()->GetClusterManager(strPrjId);
	if (!clusterManager)
		return false;

	std::vector<int> badIdList;
	for (const auto& iter : clusterBoxMap)
	{
		if (!clusterManager->FindClusterByID(iter.first))
			badIdList.push_back(iter.first);
	}

	for (const auto& iter : badIdList)
		clusterBoxMap.erase(iter);

	return true;
}

pc::data::CModelNodeVector CPCAutoSegmentToolkit::ClonePointCloudElements(
	const pc::data::CModelNodeVector& vPointCloudElements)
{
	pc::data::CModelNodeVector vClonePointCloudElements;
	// for (auto& pointCloudElement : vPointCloudElements)
	//{
	//	d3s::share_ptr<d3s::element::CElement> pCloneElement =
	// CPointCloudCommonTool::CopyCpointCloudElement(pointCloudElement); 	if
	// (!pCloneElement.valid())
	//	{
	//		continue;
	//	}
	//	pc::data::CModelNodePtr pClonePointCloudElement =
	// dynamic_pointer_cast<CPointCloudElement>(pCloneElement); 	if
	// (!pClonePointCloudElement.valid())
	//	{
	//		continue;
	//	}
	//	// 清楚PagedLod中加载的节点
	//	osg::ref_ptr<osg::Node> pCloneNode =
	// CElementNodeHandler::GetSceneNode(pClonePointCloudElement); osg::ref_ptr<CPointCloudpagedLod>
	// pClonePageLod = dynamic_pointer_cast<CPointCloudpagedLod>(pCloneNode); 	if
	//(pClonePageLod.valid())
	//	{
	//		//清楚缓存及数据请求对象
	//		CPointCloudCommonTool::ClearAppointPagedLodData(pClonePageLod, true, true);
	//	}
	//	else
	//	{
	//		continue;
	//	}
	//	// 拷贝点云模型唯一ID
	//	CString strLasId =
	// pointCloudElement->GetAttribute()->GetAttrTextW(pc::data::POINTCLOUD_UNIQUEID);
	//	pClonePointCloudElement->GetAttribute()->SetAttrText(pc::data::POINTCLOUD_UNIQUEID,
	// strLasId);
	//	// 拷贝点云模型属性
	//	std::string strModelType;
	//	pointCloudElement->GetNode()->asNode()->getUserValue<std::string>(pc::data::STRING_MODEL_TYPE_NAME,
	// strModelType);
	//	pClonePointCloudElement->GetNode()->asNode()->setUserValue<std::string>(pc::data::STRING_MODEL_TYPE_NAME,
	// strModelType); 	vClonePointCloudElements.emplace_back(pClonePointCloudElement);
	// }
	return vClonePointCloudElements;
}

IPointCloudPtr CPCAutoSegmentToolkit::ConvertPointCloud(
	CAutoSegmentFileLoadSaveThread* pThread,
	const pc::data::PointCloudBoundBox2D& boundingBox2D)
{
	if (nullptr == pThread)
	{
		d3s::CLog::Error("CPCAutoSegmentToolkit::ConvertPointCloud 自动分类读写线程为空!");
		return nullptr;
	}

	//  查询范围内点云数据，并进行转换
	IPointCloudPtr pPointCloud = pThread->GetQueryPoints(boundingBox2D);
	return pPointCloud;
}

void CPCAutoSegmentToolkit::ComputeCloudBounds(
	const pc::data::CModelNodeVector& vOldPointCloudElements,
	const pc::data::SegmentParam& segmentParam,
	size_t nPointSize,
	std::vector<osg::BoundingBox>& vBoundingBoxs,
	pc::data::CModelNodeVector& vPointCloudElements)
{
	osg::BoundingBox boundingBox;
	pc::data::PointCloudBoundToPointNum boundToPointNum;

	const std::vector<osg::Vec3d>& selectPointList =
		(std::vector<osg::Vec3d>&)segmentParam._vecSelectPoints;
	for (const auto& iter : selectPointList)
		boundingBox.expandBy(iter);

	CPCQueryWrapperToolkit::QueryBoundingBoxList(vOldPointCloudElements,
												 (osg::Matrix&)segmentParam._vpwMatrix,
												 selectPointList,
												 boundToPointNum,
												 vPointCloudElements);
	if (boundToPointNum.empty())
	{
		d3s::CLog::Error("[QueryBoundingBoxList] 获得点云原色边界范围数量: %d.",
						 boundToPointNum.size());
		return;
	}

	MergeBoundingBoxs(boundToPointNum,
					  vBoundingBoxs,
					  nPointSize,
					  boundingBox,
					  segmentParam._vecTowerPos);
}

void CPCAutoSegmentToolkit::ComputeCloudBounds(
	const pc::data::CModelNodeVector& vOldPointCloudElements,
	const pc::data::SegmentParam& segmentParam,
	size_t nPointSize,
	std::vector<osg::BoundingBox>& vBoundingBoxs,
	pc::data::CModelNodeVector& vPointCloudElements,
	ESegmentType eSegmentType)
{
	pc::data::PointCloudBoundToPointNum boundToPointNum;

	osg::BoundingBox boundingBox;
	if (eSegmentType == eTreeIndividual)
	{
		boundingBox = (osg::BoundingBox&)segmentParam._boundingBox;
		CPCQueryWrapperToolkit::QueryBoundingBoxList(vOldPointCloudElements,
													 boundingBox,
													 boundToPointNum,
													 vPointCloudElements);
	}
	else
	{
		const std::vector<osg::Vec3d>& selectPointList =
			(std::vector<osg::Vec3d>&)segmentParam._vecSelectPoints;
		for (const auto& iter : selectPointList)
			boundingBox.expandBy(iter);
		CPCQueryWrapperToolkit::QueryBoundingBoxList(vOldPointCloudElements,
													 (osg::Matrix&)segmentParam._vpwMatrix,
													 selectPointList,
													 boundToPointNum,
													 vPointCloudElements);
	}

	if (boundToPointNum.empty())
	{
		d3s::CLog::Error("[QueryBoundingBoxList] 获得点云原色边界范围数量: %d.",
						 boundToPointNum.size());
		return;
	}

	MergeBoundingBoxs(boundToPointNum,
					  vBoundingBoxs,
					  nPointSize,
					  boundingBox,
					  segmentParam._vecTowerPos);
}

void CPCAutoSegmentToolkit::TowerOffect(pc::data::SegmentParam& segmentParam,
										const osg::BoundingBox& box)
{
	std::vector<osg::Vec3d> oldTowerList = segmentParam._vecTowerPos;
	std::vector<osg::Vec3d> newTowerList;
	for (auto tower : oldTowerList)
	{
		osg::Vec3d curTowerPos = tower;
		osg::Vec3d center = box.center();
		curTowerPos =
			osg::Vec3d(curTowerPos.x() - center.x(), curTowerPos.y() - center.y(), curTowerPos.z());
		newTowerList.push_back(curTowerPos);
	}
	segmentParam._vecTowerPos = newTowerList;
}

bool FindPointCloudBound(pc::data::PointCloudBoundToPointNum& boundToPointNum,
						 osg::Vec3d pnt,
						 pc::data::PointCloudBoundBox2D& ret,
						 int& nPtCount)
{
	bool bFind = false;
	for (auto it = boundToPointNum.begin(); it != boundToPointNum.end(); ++it)
	{
		if (it->first.IsInBound(pnt.x(), pnt.y()))
		{
			ret = it->first;
			nPtCount = it->second;
			bFind = true;
			break;
		}
	}

	return bFind;
}

bool IsHaveBound(const std::vector<pc::data::PointCloudBoundBox2D>& vecTowerRelBoundBox,
				 const pc::data::PointCloudBoundBox2D& boundBox)
{
	for (auto tempBox : vecTowerRelBoundBox)
	{
		if (tempBox == boundBox)
			return true;
	}

	return false;
}

bool VectorVaild(std::vector<int>& vTileIndices)
{
	bool bResult = false;
	for (auto var : vTileIndices)
	{
		if (var != -1)
		{
			bResult = true;
			break;
		}
	}
	return bResult;
}

bool BoundingBoxContain(const osg::BoundingBox& box1, const osg::BoundingBox& box2)
{
	if (!box1.valid() || !box1.valid())
		return false;
	if (box1.xMin() <= box2.xMin() && box1.yMin() <= box2.yMin() && box1.xMax() >= box2.xMax() &&
		box1.yMax() >= box2.yMax())
		return true;
	return false;
}

void CPCAutoSegmentToolkit::MergeBoundingBoxs(pc::data::PointCloudBoundToPointNum& boundToPointNum,
											  std::vector<osg::BoundingBox>& vBoundingBoxs,
											  size_t nPointSize,
											  const osg::BoundingBox& boundingBox,
											  std::vector<osg::Vec3d> vecTowerPos)
{
	// 统计瓦片边界和瓦片包含的点数量
	osg::Vec3d center;
	size_t nTotal = 0;
	std::vector<osg::Vec3d> vTileCentroids;
	std::vector<osg::BoundingBox> vTileBounds;
	std::vector<size_t> vPointSizes;

	// 获取有效点云包围盒区域(523822
	// 自动分类：丰厚线标记杆塔之后全局自动分类，绝缘子串的识别分类不准确 附图V1.4.0.13)
	GetTileInfo(boundingBox,
				vecTowerPos,
				boundToPointNum,
				center,
				nTotal,
				vTileCentroids,
				vTileBounds,
				vPointSizes);

	center *= 1.0 / vTileBounds.size();

	if (0 == nTotal || vTileBounds.size() != vPointSizes.size())
		return;

	// 创建瓦片索引
	std::vector<int> vTileIndices(vTileBounds.size());
	std::iota(vTileIndices.begin(), vTileIndices.end(), 0);

	// 计算主成分方向，PCA特征向量
	osg::Vec3d major, middle, minor;
	CPCQueryWrapperToolkit::GetEigenVectors(major, middle, minor, vTileCentroids);

	// 瓦片按照主方向排序以达到局部连续的效果
	std::sort(vTileIndices.begin(), vTileIndices.end(), [&](const int& i, const int& j) {
		const auto& lhs = vTileBounds[i].center();
		const auto& rhs = vTileBounds[j].center();
		osg::Vec3 d1 = lhs - center;
		double l1 = d1.length();
		d1.normalize();
		double project1 = major * d1 * l1;
		osg::Vec3 d2 = rhs - center;
		double l2 = d2.length();
		d2.normalize();
		double project2 = major * d2 * l2;

		return project1 < project2;
	});

	vBoundingBoxs.clear();
	osg::BoundingBox dataBound;
	osg::BoundingBox dataBoundNoRepeate;
	// 数据划分
	while (VectorVaild(vTileIndices))
	{
		size_t nBase = nPointSize;
		int nPart = std::round((double)nTotal / (double)nBase);
		nPart = std::max(1, nPart);
		nBase = (size_t)((double)nTotal / (double)nPart);
		d3s::CLog::Info("按照 %d 个点云进行划分.", nBase);
		int nPoints = 0;

		for (size_t i = 0; i < vTileIndices.size(); ++i)
		{
			if (vTileIndices[i] == -1)
			{
				continue;
			}
			int nTilePointSize = vPointSizes[vTileIndices[i]];
			osg::BoundingBox tempBound = dataBound;

			if (tempBound.valid() &&
				(tempBound.center() - vTileBounds[vTileIndices[i]].center()).length() >
					(tempBound.radius() + vTileBounds[vTileIndices[i]].radius()))
			{
				continue;
			}
			tempBound.expandBy(vTileBounds[vTileIndices[i]]);
			double dArea =
				CPCQueryWrapperToolkit::CalcArea(tempBound, CPCQueryWrapperToolkit::eXYPlane);

			bool bContain = false;
			// 被上一个包围盒覆盖则跳过
			if (vBoundingBoxs.size() != 0)
			{
				for (auto box : vBoundingBoxs)
				{
					if (BoundingBoxContain(box, vTileBounds[vTileIndices[i]]))
					{
						bContain = true;
						break;
					}
				}
			}

			if (nPoints + nTilePointSize > nBase || dArea > C_AUTO_CLASSIFY_BOX_PROJECTION_AREA_MAX)
			{
				nPoints = 0;
				vBoundingBoxs.push_back(dataBoundNoRepeate);
				dataBound.init();
				dataBoundNoRepeate.init();
			}
			dataBound.expandBy(vTileBounds[vTileIndices[i]]);
			if (!bContain)
				dataBoundNoRepeate.expandBy(vTileBounds[vTileIndices[i]]);
			nPoints += nTilePointSize;
			vTileIndices[i] = -1;
		}

		// 剩余部分包围框，扩展到末尾包围框，以保持连续
		if (dataBound.valid())
		{
			if (dataBoundNoRepeate.valid())
				vBoundingBoxs.push_back(dataBoundNoRepeate);
			dataBound.init();
			dataBoundNoRepeate.init();
		}
	}
}

bool FindSegShowType(const std::vector<char>& vecShowType, char cType)
{
	for (auto cShowType : vecShowType)
	{
		if (cShowType == cType)
			return true;
	}

	return false;
}

bool CPCAutoSegmentToolkit::SegmentInternel(const pc::data::SegmentParam& segmentParam,
											IPointCloudPtr pCloud,
											ESegmentType eSegment)
{
	if (nullptr == pCloud.get())
	{
		d3s::CLog::Error("CPCAutoSegmentToolkit::SegmentInternel 点云为空");
		return false;
	}

	toolkit::CXmlDocument doc;
	CString strDir = L"/AutoTypeCfg.xml";
	CString strCfgPath = CLibToolkit::GetAppModuleDir(nullptr);
	CString strCfgFilePath = strCfgPath + strDir;
	if (!CFileToolkit::FileExist(strCfgPath))
		return false;

	if (!doc.LoadFile(strCfgFilePath, toolkit::fmtXMLUTF8))
	{
		d3s::CLog::Error(L"(%s) 文件读取失败!", (LPCTSTR)strCfgFilePath);
		return false;
	}
	toolkit::CXmlElement* pRoot = doc.GetElementRoot();
	if (nullptr == pRoot)
	{
		d3s::CLog::Error(L"文件根节点获取失败!");
		return false;
	}

	// 地面分类
	if (eSegment == eAutoSegment &&
		!TopographicSegmentInternel(pRoot,
									strCfgPath,
									segmentParam._strTopographicFeatures,
									pCloud))
	{
		d3s::CLog::Error(L"CPCAutoSegmentToolkit::SegmentInternel 地面分类失败!");
	}


	// 导线分类
	if (eSegment == eAutoSegment &&
		!PowerCorridorsSegmentInternel(pRoot, strCfgPath, segmentParam, pCloud))
	{
		d3s::CLog::Error(L"CPCAutoSegmentToolkit::SegmentInternel 导线分类失败!");
	}

	// 环境分类
	if (eSegment == eAutoSegment && !EnvironmentsSegmentInternel(pRoot, strCfgPath, pCloud))
	{
		d3s::CLog::Error(L"CPCAutoSegmentToolkit::SegmentInternel 环境分类失败!");
	}

	// 单木分割
	if (eSegment == eTreeIndividual && !TreeIndividualSegmentInternel(pRoot, strCfgPath, pCloud))
	{
		d3s::CLog::Error(L"CPCAutoSegmentToolkit::SegmentInternel 单木分割失败!");
	}

	return true;
}

bool CPCAutoSegmentToolkit::TopographicSegmentInternel(toolkit::CXmlElement* pRoot,
													   const CString& strCfgPath,
													   const CString& strTopographicFeatures,
													   IPointCloudPtr pCloud)
{
	if (nullptr == pRoot || strCfgPath.IsEmpty() || strTopographicFeatures.IsEmpty() ||
		!pCloud.valid())
	{
		d3s::CLog::Error(L"CPCAutoSegmentToolkit::TopographicSegmentInternel 参数错误!");
		return false;
	}
	toolkit::CXmlElement* pTopographicFeaturesElement =
		pRoot->GetChildElementAt(L"TopographicFeatures");
	if (nullptr == pTopographicFeaturesElement)
	{
		d3s::CLog::Error(L"配置文件TopographicFeatures子节点为空!");
		return false;
	}
	toolkit::CXmlElements* pFeaturesElements = pTopographicFeaturesElement->GetChildElements(TRUE);
	int nFeaturesCount = pFeaturesElements->GetCount();
	for (int i = 0; i < nFeaturesCount; ++i)
	{
		toolkit::CXmlElement* pFeaturesElement = pFeaturesElements->GetAt(i);
		if (nullptr == pFeaturesElement)
			continue;
		CString strCurTopographicFeatures = pFeaturesElement->GetAttrValue(L"Features");
		if (0 != strCurTopographicFeatures.CompareNoCase(strTopographicFeatures))
			continue;
		return SegmentInternel(pCloud,
							   d3s::pcs::SegmentationType::GROUND,
							   strCfgPath + pFeaturesElement->GetAttrValue(L"Param"));
	}

	return false;
}

bool CPCAutoSegmentToolkit::PowerCorridorsSegmentInternel(
	toolkit::CXmlElement* pRoot,
	const CString& strCfgPath,
	const pc::data::SegmentParam& segmentParam,
	IPointCloudPtr pCloud)
{
	if (nullptr == pRoot || strCfgPath.IsEmpty() || segmentParam._strVolLevel.IsEmpty() ||
		!pCloud.valid())
	{
		d3s::CLog::Error(L"CPCAutoSegmentToolkit::PowerCorridorsSegmentInternel 参数错误!");
		return false;
	}
	toolkit::CXmlElement* pLineVoltageLevelElement = pRoot->GetChildElementAt(L"LineVoltageLevel");
	if (nullptr == pLineVoltageLevelElement)
	{
		d3s::CLog::Error(L"配置文件LineVoltageLevel子节点为空!");
		return false;
	}
	toolkit::CXmlElements* pLevelElements = pLineVoltageLevelElement->GetChildElements(TRUE);
	int nLevelCount = pLevelElements->GetCount();
	for (int i = 0; i < nLevelCount; ++i)
	{
		toolkit::CXmlElement* pLevelElement = pLevelElements->GetAt(i);
		if (nullptr == pLevelElement)
			continue;
		CString strCurLineVoltageLevel = pLevelElement->GetAttrValue(L"Level");
		if (0 != strCurLineVoltageLevel.CompareNoCase(segmentParam._strVolLevel))
			continue;
		return SegmentInternel(pCloud,
							   d3s::pcs::SegmentationType::POWERCORRIDORS,
							   strCfgPath + pLevelElement->GetAttrValue(L"Param"),
							   (std::vector<osg::Vec3d>&)segmentParam._vecTowerPos);
	}
	return false;
}

bool CPCAutoSegmentToolkit::EnvironmentsSegmentInternel(toolkit::CXmlElement* pRoot,
														const CString& strCfgPath,
														IPointCloudPtr pCloud)
{
	if (nullptr == pRoot || strCfgPath.IsEmpty() || !pCloud.valid())
	{
		d3s::CLog::Error(L"CPCAutoSegmentToolkit::EnvironmentsSegmentInternel 参数错误!");
		return false;
	}
	toolkit::CXmlElement* pEnvironmentsElement = pRoot->GetChildElementAt(L"Environments");
	if (nullptr == pEnvironmentsElement)
	{
		d3s::CLog::Error(L"配置文件Environments子节点为空!");
		return false;
	}
	return SegmentInternel(pCloud,
						   d3s::pcs::SegmentationType::ENVIROMENTS,
						   strCfgPath + pEnvironmentsElement->GetAttrValue(L"Param"));
}

bool CPCAutoSegmentToolkit::TreeIndividualSegmentInternel(toolkit::CXmlElement* pRoot,
														  const CString& strCfgPath,
														  IPointCloudPtr pCloud)
{
	if (nullptr == pRoot || strCfgPath.IsEmpty() || !pCloud.valid())
	{
		d3s::CLog::Error(L"CPCAutoSegmentToolkit::TreeIndividualSegmentInternel 参数错误!");
		return false;
	}
	toolkit::CXmlElement* pTreeIndividualElement = pRoot->GetChildElementAt(L"TreeIndividual");
	if (nullptr == pTreeIndividualElement)
	{
		d3s::CLog::Error(L"配置文件TreeIndividual子节点为空!");
		return false;
	}
	return SegmentInternel(pCloud,
						   d3s::pcs::SegmentationType::TREEINDIVIDUAL,
						   strCfgPath + pTreeIndividualElement->GetAttrValue(L"Param"));
}

bool CPCAutoSegmentToolkit::SegmentInternel(
	IPointCloudPtr pCloud,
	d3s::pcs::SegmentationType eSegmentationType,
	const CString& strCfgPath,
	const std::vector<osg::Vec3d>& vecTowerPoints /*= std::vector<osg::Vec3d>()*/)
{
	if (nullptr == pCloud.get() || !CFileToolkit::FileExist(strCfgPath))
	{
		d3s::CLog::Error(L"CPCAutoSegmentToolkit::SegmentInternel 点云为空或分割配置文件不存在!");
		return false;
	}

	d3s::share_ptr<d3s::pcs::ICloudSegmentation> pSegment =
		d3s::pcs::CreateSegmentation(eSegmentationType);
	if (nullptr == pSegment)
	{
		d3s::CLog::Error(L"ICloudSegmentation 创建失败");
		return false;
	}
	pSegment->SetCloudInput(pCloud);
	std::string strPath = CStringToolkit::CStringToUTF8(strCfgPath);
	IOptionsPtr options = d3s::pcs::CreateOptions(strPath.c_str());

	if (!options.valid())
	{
		d3s::CLog::Error(L"IOptionsPtr 创建失败");
		return false;
	}

	// 添加环境道路分类配置
	if (d3s::pcs::SegmentationType::ENVIROMENTS == eSegmentationType)
	{
		// 仅加载一次道路矢量文件
		if (!_roadVectorize.valid())
		{
			CString strFmt;
			CString strAppModule = CLibToolkit::GetAppModuleDir(nullptr);
			strFmt.Format(_T("%s..\\PluginsConfig\\PointCloud\\自动分类\\chinese_road.zip"),
						  (LPCTSTR)strAppModule);

			std::string roadVectorPath = CStringToolkit::CStringToUTF8(strFmt);
			_roadVectorize = d3s::pcs::CreateRoadVectorize(roadVectorPath);

			// if (!_roadVectorize.valid())
			// {
			// 	 d3s::CLog::Error(L"chinese_road.zip 读取失败");
			// 	return false;
			// }
		}

		if (_roadVectorize.valid())
			options->Set("RoadVectorize", (char*)_roadVectorize.get());
	}

	std::vector<double> vecData;
	// 设置杆塔坐标(至少标记两个杆塔)
	if (vecTowerPoints.size() >= 2)
	{
		for (auto towerPos : vecTowerPoints)
		{
			vecData.push_back(towerPos.x());
			vecData.push_back(towerPos.y());
			vecData.push_back(towerPos.z());
		}
		int nDataSize = 3 * vecTowerPoints.size();
		options->Set("DoubleArraySize", nDataSize);
		options->Set("DoubleArrayData", (char*)&vecData[0]);
	}

	std::vector<d3s::pcs::PointId> vec;
	return pSegment->Segment(options.get(), vec);
}