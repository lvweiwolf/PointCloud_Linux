#include <Segment/PointCloudBoxQuery.h>
#include <Segment/PCQueryWrapperToolkit.h>
#include <Segment/PagedLodPointsVisitor.h>

#include <BusinessNode/BnsPointCloudNode.h>

#include <include/Log.h>

#include <Tool/FileToolkit.h>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

// 过滤点云线程
typedef std::vector<CPointReadWriter*> CFilterPointThreadVec; // 点云过滤线程容器

static const int S_THREAD_CNT = 4;		 // 线程数
static const int THREAD_SLEEP_TIME = 10; // 用于线程切换

// class CTbbReadVec
// {
// public:
// 	CTbbReadVec(CFilterPointThreadVec& threadList) : _threadList(threadList) {}
// 	void operator()(const int& theIndex) const
// 	{
// 		try
// 		{
// 			if (theIndex >= _threadList.size())
// 			{
// 				d3s::CLog::Error(L"CTbbReadVec读取越界，List:%d, Index:%d",
// 								 _threadList.size(),
// 								 theIndex);
// 				return;
// 			}
// 			_threadList.at(theIndex)->Read();
// 		}
// 		catch (...)
// 		{
// 		}
// 	}
// 	CFilterPointThreadVec& _threadList;
// };

// class CTbbWriteVec
// {
// public:
// 	CTbbWriteVec(CFilterPointThreadVec& threadList) : _threadList(threadList) {}
// 	void operator()(const int& theIndex) const
// 	{
// 		try
// 		{
// 			if (theIndex >= _threadList.size())
// 			{
// 				d3s::CLog::Error(L"CTbbWriteVec读取越界，List:%d, Index:%d",
// 								 _threadList.size(),
// 								 theIndex);
// 				return;
// 			}
// 			_threadList.at(theIndex)->Write();
// 		}
// 		catch (...)
// 		{
// 		}
// 	}
// 	CFilterPointThreadVec& _threadList;
// };

void CPointCloudBoxQuery::GetLevelPagedLodList(
	pc::data::CModelNodePtr pointCloudElement,
	int nLevel,
	std::vector<pc::data::CModelNodePtr>& pointCloudPagedLodList)
{
	if (NULL == pointCloudElement)
		return;


	if (-1 == nLevel)
	{
		// 取所有的叶子级节点
		GetLeafPagedLodList(pointCloudElement, pointCloudPagedLodList);
		return;
	}

	// 获取指定层级的pagedlod节点
	GetSubLevelPagedLodList(pointCloudElement, nLevel, pointCloudPagedLodList);
}

void CPointCloudBoxQuery::GetSubLevelPagedLodList(
	pc::data::CModelNodePtr pPagedLod,
	int nLevel,
	std::vector<pc::data::CModelNodePtr>& pointCloudPagedLodList)
{
	if (NULL == pPagedLod)
		return;

	if (0 < nLevel)
	{
		--nLevel;
		size_t nChildNode = pPagedLod->getChildCount();
		for (size_t i = 0; i < nChildNode; ++i)
		{
			pc::data::CModelNodePtr pSubNode = pPagedLod->GetChildNode(i);
			if (NULL == pSubNode)
				continue;
			GetSubLevelPagedLodList(pSubNode, nLevel, pointCloudPagedLodList);
		}
	}
	else
	{
		pointCloudPagedLodList.emplace_back(pPagedLod);
	}
}

void CPointCloudBoxQuery::GetLeafPagedLodList(
	pc::data::CModelNodePtr pPagedLod,
	std::vector<pc::data::CModelNodePtr>& pointCloudPagedLodList)
{
	if (NULL == pPagedLod)
		return;

	pointCloudPagedLodList.emplace_back(pPagedLod);

	size_t nCount = pPagedLod->getChildCount();
	for (size_t i = 0; i < nCount; ++i)
	{
		pc::data::CModelNodePtr pSubNode = pPagedLod->GetChildNode(i);
		if (NULL == pSubNode)
			continue;

		GetLeafPagedLodList(pSubNode, pointCloudPagedLodList);
	}
}

void CPointCloudBoxQuery::InitBoundBoxToFileDic(
	const std::vector<pc::data::CModelNodePtr>& pointCloudPagedLodList,
	BoundToFile& boundBoxToFiles)
{
	size_t nCnt = pointCloudPagedLodList.size();
	for (size_t i = 0; i < nCnt; ++i)
	{
		auto& pointCloudPagedLod = pointCloudPagedLodList[i];
		CBnsPointCloudNode pLasPagedLod = pointCloudPagedLod;
		if (pLasPagedLod.IsNull())
			return;

		osg::BoundingBox boundingBox = pLasPagedLod.GetModelBoundingBox();
		pc::data::PointCloudBoundBox2D boundBox(boundingBox.xMin(),
												boundingBox.xMax(),
												boundingBox.yMin(),
												boundingBox.yMax());

		// 获取pagedlod对应的模型文件路径
		pc::data::tagPagedLodFile strModelPath =
			CPCQueryWrapperToolkit::GetPagedLodModelPath(pointCloudPagedLod);
		if (strModelPath.strPointInfoFile.IsEmpty())
		{
			d3s::CLog::Error(_T("InitBoundBoxToFileDic 第 %I64u 个pagedlod 模型路径为空"), i);
			continue;
		}
		boundBoxToFiles[boundBox].emplace_back(strModelPath);
	}
}

void CPointCloudBoxQuery::CalcValidNodeList(
	const std::vector<pc::data::CModelNodePtr>& pointCloudPagedLodList,
	std::vector<pc::data::CModelNodePtr>& validNodeList,
	std::vector<pc::data::CModelNodePtr>& validAllInNodeList,
	const pc::data::PointCloudBoundBox2D& boundBox)
{
	size_t nCnt = pointCloudPagedLodList.size();
	for (size_t i = 0; i < nCnt; ++i)
	{
		auto& pointCloudPagedLod = pointCloudPagedLodList[i];
		CBnsPointCloudNode pLasPagedLod = pointCloudPagedLod;
		if (pLasPagedLod.IsNull())
			return;

		osg::BoundingBox boundingBox = pLasPagedLod.GetModelBoundingBox();
		pc::data::PointCloudBoundBox2D tempBox(boundingBox.xMin(),
											   boundingBox.xMax(),
											   boundingBox.yMin(),
											   boundingBox.yMax());
		if (!tempBox.IsIntersect(boundBox))
			continue;

		if (boundBox.IsInBound(tempBox))
		{
			validAllInNodeList.emplace_back(pointCloudPagedLod);
		}
		else
		{
			validNodeList.emplace_back(pointCloudPagedLod);
		}
	}
}

void CPointCloudBoxQuery::MtReadPointsToPointCloud(const SMtReadParam& param)
{
	int nPointSize = 0;
	// 计算顶点总数
	for (auto var : param.validNodeList)
	{
		CBnsPointCloudNode pLasPagedLod = var;
		if (pLasPagedLod.IsNull())
			continue;

		nPointSize += pLasPagedLod.GetPointsNum();
	}
	for (auto var : param.validAllInNodeList)
	{
		CBnsPointCloudNode pLasPagedLod = var;
		if (pLasPagedLod.IsNull())
			continue;
		nPointSize += pLasPagedLod.GetPointsNum();
	}
	param.pPointCloud->SetSize(nPointSize);

	// 过滤筛选线程
	CFilterPointThreadVec threadList;
	size_t nNodeCnt = param.validAllInNodeList.size();
	nPointSize = 0;
	SPointReadWriterParam readWriterParam;
	readWriterParam._pPointCloud = param.pPointCloud;
	readWriterParam._mapConvertType = param.mapConvertType;
	readWriterParam._vecSegShowTypep = param.segShowTypeMap;
	readWriterParam._eSegment = param.eSegment;
	readWriterParam._vegetationTypes = param._vegetationTypes;
	readWriterParam._nGroundType = param.nGroundType;
	readWriterParam._boundBox = param.boxList;
	for (size_t i = 0; i < nNodeCnt; ++i)
	{
		// 获取pagedlod对应的模型文件路径
		readWriterParam._pLodNode = param.validAllInNodeList[i];
		CBnsPointCloudNode pLasPagedLod = readWriterParam._pLodNode;
		if (pLasPagedLod.IsNull())
			continue;

		// 560909 自动分类：使用咸梦II回点云文件，进行自动分类，杆塔的分类效果不正确 附图V1.6.0.31
		pc::data::tagPagedLodFile strModelPath =
			CPCQueryWrapperToolkit::GetPagedLodModelPath(readWriterParam._pLodNode);
		if (strModelPath.strPointInfoFile.IsEmpty() ||
			!CFileToolkit::FileExist(strModelPath.strPointInfoFile))
		{
			d3s::CLog::Error(L"读取 （%s） 文件节点为空!", (LPCTSTR)strModelPath.strPointInfoFile);
			continue;
		}
		readWriterParam._nBeginIndex = nPointSize;
		CPointReadWriter* pFilterThread = new CPointReadWriter(readWriterParam);
		nPointSize += pLasPagedLod.GetPointsNum();
		threadList.emplace_back(pFilterThread);
	}

#ifndef USE_MULTI_THREAD
	// CTbbReadVec parallel(threadList);
	// CTBBParallel::For(0, threadList.size(), parallel);
	for (auto pThread : threadList)
	{
		pThread->Read();
	}
#else
	tbb::parallel_for(tbb::blocked_range<size_t>(0, threadList.size()),
					  [&](const tbb::blocked_range<size_t>& r) {
						  for (size_t i = r.begin(); i != r.end(); ++i)
							  threadList.at(i)->Read();
					  });
#endif

	// 获取线程队列数据,并同步清除线程，减少内存峰值开销
	for (auto& pThread : threadList)
	{
		SAFE_DELETE(pThread);
	}
	threadList.clear();

	nNodeCnt = param.validNodeList.size();
	for (size_t i = 0; i < nNodeCnt; ++i)
	{
		// 获取pagedlod对应的模型文件路径
		readWriterParam._pLodNode = param.validNodeList[i];
		readWriterParam._nBeginIndex = nPointSize;
		CPointReadWriter pFilterThread(readWriterParam, param.boundBox);
		pFilterThread.Read();
		nPointSize += pFilterThread.GetPointsNum();
	}
	param.pPointCloud->SetSize(nPointSize);
}

void CPointCloudBoxQuery::MtSetPointsCloudToPoints(SMtSetParam& param)
{
	if (nullptr == param.pPointCloud)
		return;

	int nPointSize = 0;

	// 降噪
	std::set<int> outlierSet;
	auto denoiseLevels = CPCQueryWrapperToolkit::GetDenoiseParam();
	if (param.nDenoiseLevel > 0 && param.nDenoiseLevel <= denoiseLevels.params.size())
	{
		std::vector<int> inliers;
		std::vector<int> outliers;
		auto denoiseParam = denoiseLevels.params.at(param.nDenoiseLevel - 1);
		param.pPointCloud->Denoise(denoiseParam._dRadius,
								   denoiseParam._nMinPtSize,
								   inliers,
								   outliers);
		for (const auto& iter : outliers)
		{
			outlierSet.insert(iter);
		}
	}

	// 过滤筛选线程
	CFilterPointThreadVec threadList;
	size_t nNodeCnt = param.validAllInNodeList.size();
	pc::share_ptr<SClusterBoxMap> pClusterBoxMap = new SClusterBoxMap(param.clusterBoxMap);
	pc::share_ptr<STreeClusterMap> pTreeClusterMap = new STreeClusterMap;
	pc::share_ptr<SClearCluster> pClearCluster = new SClearCluster(param.clearCluster);
	SPointReadWriterParam readWriterParam;
	readWriterParam._pPointCloud = param.pPointCloud;
	readWriterParam._mapConvertType = param.mapConvertType;
	readWriterParam._vecSegShowTypep = param.segShowTypeMap;
	readWriterParam._eSegment = param.eSegment;
	readWriterParam._pClusterBoxMap = pClusterBoxMap;
	readWriterParam._strPrjId = param.strPrjId;
	readWriterParam._pTreeClusterMap = pTreeClusterMap;
	readWriterParam._denoiseIndexSet = outlierSet;
	readWriterParam._clearTypeCluster = param.clearTypeCluster;
	readWriterParam._pClearCluster = pClearCluster;
	for (size_t i = 0; i < nNodeCnt; ++i)
	{
		// 获取pagedlod对应的模型文件路径
		readWriterParam._pLodNode = param.validAllInNodeList[i];
		CBnsPointCloudNode pLasPagedLod = readWriterParam._pLodNode;
		if (pLasPagedLod.IsNull())
			continue;

		// 560909 自动分类：使用咸梦II回点云文件，进行自动分类，杆塔的分类效果不正确 附图V1.6.0.31
		pc::data::tagPagedLodFile strModelPath =
			CPCQueryWrapperToolkit::GetPagedLodModelPath(readWriterParam._pLodNode);
		if (strModelPath.strPointInfoFile.IsEmpty() ||
			!CFileToolkit::FileExist(strModelPath.strPointInfoFile))
		{
			d3s::CLog::Error(L"读取 （%s） 文件节点为空!", (LPCTSTR)strModelPath.strPointInfoFile);
			continue;
		}
		readWriterParam._nBeginIndex = nPointSize;
		CPointReadWriter* pFilterThread = new CPointReadWriter(readWriterParam);
		pFilterThread->SetPolygonParam(param.vecSelectPonits, param.boundingBox, param.vpwMatrix);
		nPointSize += pLasPagedLod.GetPointsNum();
		threadList.emplace_back(pFilterThread);
	}

#ifndef USE_MULTI_THREAD
	// CTbbWriteVec parallel(threadList);
	// CTBBParallel::For(0, threadList.size(), parallel);
	for (auto pThread : threadList)
	{
		pThread->Write();
	}
#else
	tbb::parallel_for(tbb::blocked_range<size_t>(0, threadList.size()),
					  [&](const tbb::blocked_range<size_t>& r) {
						  for (size_t i = r.begin(); i != r.end(); ++i)
							  threadList.at(i)->Write();
					  });
#endif

	// 获取线程队列数据,并同步清除线程，减少内存峰值开销
	for (auto& pThread : threadList)
	{
		SAFE_DELETE(pThread);
	}
	threadList.clear();

	nNodeCnt = param.validNodeList.size();
	for (size_t i = 0; i < nNodeCnt; ++i)
	{
		// 获取pagedlod对应的模型文件路径
		readWriterParam._pLodNode = param.validNodeList[i];
		// 560909 自动分类：使用咸梦II回点云文件，进行自动分类，杆塔的分类效果不正确 附图V1.6.0.31
		pc::data::tagPagedLodFile strModelPath =
			CPCQueryWrapperToolkit::GetPagedLodModelPath(readWriterParam._pLodNode);
		if (strModelPath.strPointInfoFile.IsEmpty() ||
			!CFileToolkit::FileExist(strModelPath.strPointInfoFile))
		{
			d3s::CLog::Error(L"读取 （%s） 文件节点为空!", (LPCTSTR)strModelPath.strPointInfoFile);
			continue;
		}
		readWriterParam._nBeginIndex = nPointSize;
		CPointReadWriter pFilterThread(readWriterParam, param.boundBox);
		pFilterThread.SetPolygonParam(param.vecSelectPonits, param.boundingBox, param.vpwMatrix);
		pFilterThread.Write();
		nPointSize += pFilterThread.GetPointsNum();
	}
}