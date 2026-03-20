
#include <Segment/AutoSegmentFileLoadSaveThread.h>
#include <Segment/PointCloudBoxQuery.h>
#include <Segment/PointCloudBoxQuery.h>
#include <BusinessNode/PCNodeType.h>

#include <include/PointCloudSegAPI.h>
#include <include/Log.h>

//////////////////////////////自动分类文件处理线程/////////////////////////////
// 等待时间
const int INT_WAIT_TIME = 100;

CAutoSegmentFileLoadSaveThread::CAutoSegmentFileLoadSaveThread(SSegmentThreadParam& param)
	: _nCompleteQueryCount(0),
	  _nCompleteModifyCount(0),
	  _bCompleteQuery(false),
	  _bModifyPointCloud(false),
	  _bStop(false),
	  _bPolygon(false),
	  _param(param),
	  _bReadTinyPagedLods(false)
{
}

CAutoSegmentFileLoadSaveThread::~CAutoSegmentFileLoadSaveThread(void) {}

/*---------------线程执行---------------*/
void CAutoSegmentFileLoadSaveThread::run(void)
{
	try
	{
		int nCount = _param._vBoundingBoxs.size();
		while (_nCompleteQueryCount != nCount || _nCompleteModifyCount != nCount)
		{
			// 查询结果获取后清空，查询结果为空时执行一次查询
			if (!_bCompleteQuery && _nCompleteQueryCount != nCount)
			{
				// d3s::CTimeLog timeRecord(L"查询（第%d个包围盒）点耗时:", _nCompleteQueryCount);
				osg::BoundingBox boundBox = _param._vBoundingBoxs[_nCompleteQueryCount];
				_queryBoundBox = pc::data::PointCloudBoundBox2D(boundBox.xMin(),
																boundBox.xMax(),
																boundBox.yMin(),
																boundBox.yMax());
				_pReadedPoints = QueryPoints(boundBox);
				++_nCompleteQueryCount;
				_bCompleteQuery = true;
			}
			// 修改点集分类（包括第零层级的第一层级）
			if (_bModifyPointCloud)
			{
				// d3s::CTimeLog timeRecord(L"修改（第%d块）第零层级和第一层级耗时：",
				// _nCompleteModifyCount);
				ModifyPointsClassify();
				++_nCompleteModifyCount;
				_bModifyPointCloud = false;
			}
			if (_bStop)
			{
				break;
			}
		}
	}
	catch (...)
	{
	}
}

void CAutoSegmentFileLoadSaveThread::Stop(void)
{
	_bStop = true;
	join();
}

/*-------------------------------------------------------------------------查询-------------------------------------------------------------------------*/
IPointCloudPtr CAutoSegmentFileLoadSaveThread::QueryPoints(osg::BoundingBox boundBox)
{
	if (_param._vPointCloudElements.empty())
	{
		d3s::CLog::Warn("CPointCloudQueryService::QueryPoints 点云元素列表为空！");
		return NULL;
	}
	if (!_bReadTinyPagedLods)
	{
		// 获取所有层级pagedlod节点
		for (auto& pPointCloudElement : _param._vPointCloudElements)
		{
			CPointCloudBoxQuery::GetLevelPagedLodList(pPointCloudElement,
													  CPointCloudBoxQuery::nAllLevel,
													  _vTinyPagedLods);
		}
		_bReadTinyPagedLods = true;
	}
	if (_vTinyPagedLods.empty())
	{
		d3s::CLog::Error("CPointCloudQueryService::QueryPoints 未查询到PagedLod节点!");
		return NULL;
	}

	CPointCloudBoxQuery::CalcValidNodeList(_vTinyPagedLods,
										   _vReadValidLods,
										   _vReadAllInValidLods,
										   _queryBoundBox);


	IPointCloudPtr pPointCloud = d3s::pcs::CreatePointCloud();
	CPointCloudBoxQuery::SMtReadParam param;
	param.pPointCloud = pPointCloud;
	param.validNodeList = _vReadValidLods;
	param.validAllInNodeList = _vReadAllInValidLods;
	param.mapConvertType = _param._mapConvertType;
	param.segShowTypeMap = _param._showTypeMap;
	param.boundBox = _queryBoundBox;
	param.eSegment = _param._eSegment;
	param._vegetationTypes = _param._vegetationTypes;
	param.nGroundType = _param._nGroundType;
	param.boxList = boundBox;
	CPointCloudBoxQuery::MtReadPointsToPointCloud(param);
	return pPointCloud;
}

IPointCloudPtr CAutoSegmentFileLoadSaveThread::GetQueryPoints(
	const pc::data::PointCloudBoundBox2D& boundBox)
{
	// d3s::CTimeLog timeRecord(L"");
	while (!_bCompleteQuery || _vWriteValidLods.size() != 0 || _pModifyPoints != NULL)
	{
		//::Sleep(INT_WAIT_TIME);
		usleep(INT_WAIT_TIME * 1000);
	}
	// timeRecord._strTip.Format(L"获取（第%d个包围盒）点耗时:", _nCompleteQueryCount - 1);
	if (boundBox != _queryBoundBox)
	{
		d3s::CLog::Error("CAutoSegmentFileLoadSaveThread::GetQueryPoints 已查询节点的包围盒和需要查询的包围盒不一致!");
		//  增加容错防止线程卡死
		++_nCompleteModifyCount;
		return NULL;
	}
	_pModifyPoints = _pReadedPoints;
	_pReadedPoints = NULL;
	_vWriteValidLods = _vReadValidLods;
	_vWriteAllInValidLods = _vReadAllInValidLods;
	_vReadValidLods.clear();
	_vReadAllInValidLods.clear();
	_bCompleteQuery = false;
	return _pModifyPoints;
}
/*-------------------------------------------------------------------------修改-------------------------------------------------------------------------*/

void CAutoSegmentFileLoadSaveThread::SetModifyPoints(const pc::data::PointCloudBoundBox2D& boundBox)
{
	while (_bModifyPointCloud)
	{
		//::Sleep(INT_WAIT_TIME);
		usleep(INT_WAIT_TIME * 1000);
	}
	_modifyBoundBox = boundBox;
	_bModifyPointCloud = true;
}


void CAutoSegmentFileLoadSaveThread::SetPolygonParam(const std::vector<osg::Vec3d>& vecSelectPonits,
													 const osg::BoundingBox& boundingBox,
													 const osg::Matrix& vpwMatrix)
{
	if (!vecSelectPonits.empty() || boundingBox.valid())
	{
		_bPolygon = true;
		_vecSelectPoints = vecSelectPonits;
		_boundingBox = boundingBox;
		_vpwMatrix = vpwMatrix;
	}
}

/*-------------------------------------------------------------------------文件-------------------------------------------------------------------------*/
void CAutoSegmentFileLoadSaveThread::ModifyPointsClassify(void)
{
	if (_param._vPointCloudElements.empty())
	{
		d3s::CLog::Warn("ModifyPointsClassify 点云为空");
		return;
	}
	
	std::vector<pc::data::CModelNodePtr> tempLods = _vWriteValidLods;
	std::vector<pc::data::CModelNodePtr> tempAllInLods = _vWriteAllInValidLods;
	IPointCloudPtr tempPoint = _pModifyPoints;
	_vWriteValidLods.clear();
	_vWriteAllInValidLods.clear();
	_pModifyPoints = NULL;
	LPCTSTR strPrjId = nullptr;
	if (nullptr != _param._vPointCloudElements.front())
	{
		auto pProjectNode =
			_param._vPointCloudElements.front()->GetTypeParent((int)eBnsProjectRoot);
		if (pProjectNode)
		{
			strPrjId = pProjectNode->GetId();
		}
	}

	CPointCloudBoxQuery::SMtSetParam param(_param._clusterBoxMap);
	param.pPointCloud = tempPoint;
	param.validNodeList = tempLods;
	param.validAllInNodeList = tempAllInLods;
	param.vecSelectPonits = _vecSelectPoints;
	param.boundingBox = _boundingBox;
	param.vpwMatrix = _vpwMatrix;
	param.mapConvertType = _param._mapConvertType;
	param.segShowTypeMap = _param._showTypeMap;
	param.boundBox = _modifyBoundBox;
	param.nDenoiseLevel = _param._nDenoiseLevel;
	param.eSegment = _param._eSegment;
	param.clusterBoxMap = _param._clusterBoxMap;
	param.strPrjId = strPrjId;
	param.clearTypeCluster = _param._clearTypeCluster;
	CPointCloudBoxQuery::MtSetPointsCloudToPoints(param);
	_clearCluter.insert(param.clearCluster.begin(), param.clearCluster.end());
}
