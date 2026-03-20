#include <LasFile/ExportLasTool.h>
#include <LasFile/PointCloudPageLod.h>
#include <LasFile/PointCloudPropertyTool.h>
#include <LasFile/PointCloudToolkit.h>
#include <LasFile/PointCloudTool.h>

#include <BusinessNode/PCNodeType.h>
#include <BusinessNode/BnsPointCloudNode.h>
#include <Segment/PointCloudBoxQuery.h>
#include <Segment/PCQueryWrapperToolkit.h>

#include <include/IGeoProjectionConvertor.h>
#include <include/PointCloudSegAPI.h>

#include <Tool/FileToolkit.h>


#define SCALE_FACTOR 0.0001
#define DATA_FORMAT 2
#define DATA_RECORD_LENGTH 26
#define EPSG_LOCATION 0
#define EPSG_KEY_COUNT 1
#define LAS_TINY_RIGHT_MOVE 8
#define	INT_SAVE_EPAG_KEY 3072
#define EXPORT_ALL INT16_MAX


// 遍历获取顶点及颜色
class CDataGetter : public osg::NodeVisitor
{
public:
	struct SData
	{
		osg::ref_ptr<const osg::Vec3Array>		_pVertexArray;
		osg::ref_ptr<const osg::Vec4ubArray>	_pColorArray;
		osg::ref_ptr<osg::Vec2Array>				_pTexCoordArray;
		osg::Matrix								_matrix;
	};
	typedef std::map<osg::ref_ptr<const osg::Vec3Array>, std::pair<osg::ref_ptr<const osg::Vec4ubArray>, osg::Matrix>> DataMap;
	CDataGetter(osg::NodeVisitor::TraversalMode traversalMode = osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) :osg::NodeVisitor(traversalMode)
	{
	}

public:
	void apply(osg::Geometry& geometry)
	{
		const osg::ref_ptr<osg::Vec3Array> pVertexArray = dynamic_cast<osg::Vec3Array*>(geometry.getVertexArray());
		const osg::ref_ptr<osg::Vec4ubArray> pColorArray = dynamic_cast<osg::Vec4ubArray*>(geometry.getColorArray());
		osg::ref_ptr<osg::Vec2Array> pTexCoordArray = dynamic_cast<osg::Vec2Array*>(geometry.getTexCoordArray(0));
		if (nullptr == pVertexArray || nullptr == pColorArray || nullptr == pTexCoordArray)
			return;

		_pDataMap.push_back(SData{ pVertexArray , pColorArray , pTexCoordArray , osg::computeLocalToWorld(getNodePath()) });
	}

	std::vector<SData> GetDataMap() { return _pDataMap; }

private:
	std::vector<SData> _pDataMap;
};

CExportLasTool::CExportLasTool(pc::data::CModelNodePtr pProjectNode, const CString& strOutPath)
	: _pProjectNode(pProjectNode), _strOutPath(strOutPath)
{

}

CExportLasTool::~CExportLasTool()
{
}

void CExportLasTool::ExportLas()
{
	CBnsProjectNode bnsProject = _pProjectNode;
	if (bnsProject.IsNull())
		return;

	pc::data::CModelNodeVector nodes = bnsProject.GetPointCloudNodeList();


	int nEpsg = bnsProject.GetEpsg();
	osg::Vec3d offsetXyz = bnsProject.getBasePos();
	CString strScanTime = bnsProject.GetScanTime();

	std::vector<pc::data::CModelNodePtr> vAllPagedLods;
	CPointCloudBoxQuery::GetLevelPagedLodList(_pProjectNode, CPointCloudBoxQuery::nAllLevel, vAllPagedLods);

	CString strFileName = _strOutPath + L"/分类点云.las";
	std::map<int, bool> errorMap;
	ExportLasFileWithTime(vAllPagedLods, nEpsg, offsetXyz, strFileName, errorMap, strScanTime);

}

bool CExportLasTool::ExportLasFileWithTime(const std::vector<pc::data::CModelNodePtr>& pageLodList, 
	const int& nEpsg, const osg::Vec3d& offsetXyz, const CString& strFileName, std::map<int, bool>& errorMap, 
	CString& strScanTime, const bool& bByType /*= false*/, const std::map<int, CString> typeFileMap /*= std::map<int, CString>()*/, 
	const std::vector<int>& vecType /*= std::vector<int>()*/)
{
	//d3s::CTimeLog timeRecord(_T("导出las文件耗时"));
	if (pageLodList.empty() || (bByType && typeFileMap.empty()))
	{
		//d3s::CLog::Error(_T("参数传递为空"));
		return false;
	}

	// 获取pagedLod所有文件、计算包围盒
	std::vector<pc::data::tagPagedLodFile> pageLodFileList;
	osg::BoundingBox boundingBox;
	for (auto pPageLod : pageLodList)
	{
		CBnsPointCloudNode bnsPc = pPageLod;
		if (bnsPc.IsNull() || pPageLod->getNodeTypeId() != EProjectNodeType::eBnsPointCloudLodNode)
			continue;

		boundingBox.expandBy(bnsPc.GetModelBoundingBox());
		pageLodFileList.push_back(CPCQueryWrapperToolkit::GetPagedLodModelPath(pPageLod));
	}

	// 创建写las文件操作器
	std::map<int, SLasWriterData> typeLasWriterMap;
	if (!CreateTypeLasWriterWithTime(typeLasWriterMap, strFileName, bByType, typeFileMap, boundingBox, nEpsg, offsetXyz, strScanTime))
		return false;

	// 读取pagedLod文件、写las文件
	for (const auto& strPageLodFile : pageLodFileList)
	{
		std::string sPointInfoFile = CStringToolkit::CStringToUTF8(strPageLodFile.strPointInfoFile);
		std::string sPointTexFile = CStringToolkit::CStringToUTF8(strPageLodFile.strPointTexFile);
		osg::ref_ptr<osg::Node> pNode = CPointCloudToolkit::ReadNode(sPointInfoFile, sPointTexFile);
		if (nullptr == pNode)
		{
			//d3s::CLog::Error(_T("%s读取失败"), strPageLodFile);
			continue;
		}

		WriteLas(pNode, bByType, typeLasWriterMap, vecType, errorMap, offsetXyz);
	}

	// 释放内存
	for (auto& iter : typeLasWriterMap)
	{
		if (nullptr == iter.second._pLaswriter)
			continue;
		iter.second._pLaswriter->update_header(&iter.second._header, true);
		iter.second._pLaswriter->close();
		delete iter.second._pLaswriter;
	}

	return true;
}


bool CExportLasTool::CreateTypeLasWriterWithTime(std::map<int, SLasWriterData>& typeLasWriterMap, 
	const CString& strFileName, const bool& bByType, const std::map<int, CString> typeFileMap, 
	const osg::BoundingBox& boundingBox, const int& nEpsg, const osg::Vec3d& offsetXyz, CString& strScanTime)
{
	if (!bByType)
	{
		auto strFilePath = CFileToolkit::GetFileDirectory(strFileName);
		if (!CFileToolkit::DirectoryExist(strFilePath))
			CFileToolkit::CreateDirectory(strFilePath);
		CreateFileHandleWithTime(typeLasWriterMap[EXPORT_ALL]._lasWriteOpener, typeLasWriterMap[EXPORT_ALL]._header, boundingBox, strFileName, nEpsg, offsetXyz, strScanTime);
		if (!CreateWriteOperator(typeLasWriterMap[EXPORT_ALL]._lasWriteOpener, typeLasWriterMap[EXPORT_ALL]._header, typeLasWriterMap[EXPORT_ALL]._pLaswriter
			, typeLasWriterMap[EXPORT_ALL]._laspoint))
		{
			//d3s::CLog::Error(_T("写las文件操作器创建失败"));
			return false;
		}
		return true;
	}

	for (auto iter : typeFileMap)
	{
		auto strFilePath = CFileToolkit::GetFileDirectory(iter.second);
		if (!CFileToolkit::DirectoryExist(strFilePath))
			CFileToolkit::CreateDirectory(strFilePath);
		CreateFileHandleWithTime(typeLasWriterMap[iter.first]._lasWriteOpener, typeLasWriterMap[iter.first]._header, boundingBox, iter.second, nEpsg, offsetXyz, strScanTime);
		if (!CreateWriteOperator(typeLasWriterMap[iter.first]._lasWriteOpener, typeLasWriterMap[iter.first]._header, typeLasWriterMap[iter.first]._pLaswriter
			, typeLasWriterMap[iter.first]._laspoint))
		{
			//d3s::CLog::Error(_T("写las文件操作器创建失败"));
			return false;
		}
	}
	return true;
}


void CExportLasTool::CreateFileHandleWithTime(LASwriteOpener& lasWriteOpener, LASheader& header,
	const osg::BoundingBox& boundingBox, const CString& strFileName, const int nEpsg, const osg::Vec3d& offsetXyz, CString& strScanTime)
{
	lasWriteOpener.set_file_name(CStringToolkit::CStringToUTF8(strFileName).c_str());
	header.x_scale_factor = SCALE_FACTOR;
	header.y_scale_factor = SCALE_FACTOR;
	header.z_scale_factor = SCALE_FACTOR;
	header.x_offset = offsetXyz.x();
	header.y_offset = offsetXyz.y();
	header.z_offset = offsetXyz.z();
	header.point_data_format = DATA_FORMAT;
	header.point_data_record_length = DATA_RECORD_LENGTH;

	// 导出点云记录公司名称
	strcpy(header.generating_software, "BOOWAY");
	header.set_bounding_box(boundingBox.xMin() + offsetXyz.x(), boundingBox.yMin() + offsetXyz.y(), boundingBox.zMin() + offsetXyz.z(),
		boundingBox.xMax() + offsetXyz.x(), boundingBox.yMax() + offsetXyz.y(), boundingBox.zMax() + offsetXyz.z());
	SetGeotiff(&header, nEpsg);
	//导出点云扫描时间
	SetSacnTime(&header, strScanTime);
}

bool CExportLasTool::CreateWriteOperator(LASwriteOpener& lasWriteOpener, LASheader& header, LASwriter*& pLaswriter, LASpoint& laspoint)
{
	pLaswriter = lasWriteOpener.open(&header);
	if (nullptr == pLaswriter)
	{
		//d3s::CLog::Warn(_T("laswriteopener.open()失败"));
		return false;
	}
	return BOOL(0) != laspoint.init(&header, header.point_data_format, header.point_data_record_length);
}

void CExportLasTool::SetGeotiff(LASheader* pHeader, const int nEpsg)
{
	if (nullptr == pHeader || nEpsg <= 0)
		return;

	IGeoProjectionConvertorPtr convertor = d3s::pcs::CreateGeoProjectionConvertor(nEpsg);
	if (nullptr == convertor)
		return;

	int len = 0;
	char* wkt = nullptr;

	if (convertor->GetWktFromProjection(len, &wkt) && nullptr != wkt)
	{
		pHeader->set_geo_ogc_wkt(len, wkt);

		if ((pHeader->version_minor >= 4) && (pHeader->point_data_format >= 6))
			pHeader->set_global_encoding_bit(LAS_TOOLS_GLOBAL_ENCODING_BIT_OGC_WKT_CRS);

		free(wkt);
	}

	int number_of_keys = 0;
	GeoProjectionGeoKeys* geo_keys = nullptr;
	int num_geo_double_params = 0;
	double* geo_double_params = 0;
	bool projection_was_set = false;

	if (convertor->HasProjection())
	{

		projection_was_set = convertor->GetGeoKeysFromProjection(number_of_keys,
			&geo_keys,
			num_geo_double_params,
			&geo_double_params);
	}

	if (!projection_was_set)
		return;

	pHeader->set_geo_keys(number_of_keys, (LASvlr_key_entry*)geo_keys);
	free(geo_keys);

	if (geo_double_params)
	{
		pHeader->set_geo_double_params(num_geo_double_params, geo_double_params);
		free(geo_double_params);
	}
	else
	{
		pHeader->del_geo_double_params();
	}

	pHeader->del_geo_ascii_params();
}

void CExportLasTool::SetSacnTime(LASheader* pHeader, CString& strScanTime)
{
	// 解析日期字符串
	int y = 0, m = 0, d = 0;
	// 使用 swscanf 解析宽字符串
	if (swscanf(strScanTime, L"%d-%d-%d", &y, &m, &d) != 3)
	{
		// 解析失败，使用系统当前时间（本地时间）
		auto now = std::chrono::system_clock::now();
		std::time_t t = std::chrono::system_clock::to_time_t(now);
		std::tm* local = std::localtime(&t);
		if (local) {
			y = local->tm_year + 1900;
			m = local->tm_mon + 1;
			d = local->tm_mday;
		}
		else {
			// 如果 localtime 失败，fallback 到默认值
			y = 1970; m = 1; d = 1;
		}
	}


	// 填充 tm 结构计算年中天数
	std::tm date_tm = { 0 };
	date_tm.tm_year = y - 1900;
	date_tm.tm_mon = m - 1;
	date_tm.tm_mday = d;
	// 调用 mktime 会填充 tm_yday
	std::mktime(&date_tm);
	int dayOfYear = date_tm.tm_yday + 1; // tm_yday 从 0 开始

	pHeader->file_creation_day = dayOfYear;
	pHeader->file_creation_year = y;
}



void CExportLasTool::WriteLas(osg::Node* pNode, const bool& bByType, std::map<int, SLasWriterData>& typeLasWriterMap,
	const std::vector<int>& vecType, std::map<int, bool>& errorMap, const osg::Vec3d& offsetXyz)
{
	if (nullptr == pNode)
		return;

	LASwriter* pLaswriter = nullptr;
	LASpoint* pLaspoint = nullptr;
	if (!bByType)
	{
		pLaswriter = typeLasWriterMap[EXPORT_ALL]._pLaswriter;
		pLaspoint = &(typeLasWriterMap[EXPORT_ALL]._laspoint);
		if (nullptr == pLaswriter || nullptr == pLaspoint)
			return;
	}

	CDataGetter dataGetter;
	pNode->accept(dataGetter);
	std::vector<CDataGetter::SData> dataList = dataGetter.GetDataMap();
	for (auto iter : dataList)
	{
		const osg::ref_ptr<const osg::Vec3Array>& pVertexArray = iter._pVertexArray;
		const osg::ref_ptr<const osg::Vec4ubArray>& pColorArray = iter._pColorArray;
		osg::ref_ptr<osg::Vec2Array> pTexCoordArray = iter._pTexCoordArray;
		const osg::Matrix& matrix = iter._matrix;
		if (nullptr == pVertexArray || nullptr == pColorArray || nullptr == pTexCoordArray)
		{
			//d3s::CLog::Warn(_T("顶点、颜色或者纹理指针为空"));
			continue;
		}
		size_t nCount = pVertexArray->size();
		if (nCount != pColorArray->size() || nCount != pTexCoordArray->size())
		{
			//d3s::CLog::Warn(_T("顶点数量与颜色数量或者分类数量不同"));
			continue;
		}

		for (size_t i = 0; i < nCount; ++i)
		{
			// 导出分类，判断点云是否删除
			uint32_t nType = 0;
			if (!CPointCloudPropertyTool::GetSegmentProperty(pTexCoordArray, i, nType, true))
				continue;

			if (bByType)
			{
				auto iter = typeLasWriterMap.find(nType);
				if (typeLasWriterMap.end() == iter || nullptr == iter->second._pLaswriter)
					continue;
				pLaswriter = iter->second._pLaswriter;
				pLaspoint = &(iter->second._laspoint);
				errorMap[nType] = true;
			}
			else if (!vecType.empty())	//如果外部传入类型则检测
			{
				auto iter = std::find(vecType.begin(), vecType.end(), nType);
				if (vecType.end() == iter)
					continue;
			}

			osg::Vec3d point = (*pVertexArray)[i] * matrix;
			const osg::Vec4ub& color = (*pColorArray)[i];
			if (!WritePointInfo(point, color, offsetXyz, nType, pLaswriter, *pLaspoint))
			{
				//d3s::CLog::Warn(_T("[PointCloudWriter::PointCloudWriter] 写入第 %d 个点时，发生错误"), i);
				break;
			}
		}
	}
}

bool CExportLasTool::WritePointInfo(const osg::Vec3d& point, const osg::Vec4ub& color, const osg::Vec3d& offsetXyz, uint32_t nType
	, LASwriter* pLaswriter, LASpoint& laspoint)
{
	if (nullptr == pLaswriter)
		return false;

	static std::map<unsigned, unsigned> typeMapping = CPointCloudTool::ReadTypeMapping();

	laspoint.set_x(point.x() + offsetXyz.x());
	laspoint.set_y(point.y() + offsetXyz.y());
	laspoint.set_z(point.z());
	// 558136 类别设置：新增的类别超过3种，第3种及以上的类别无法单独显示在视图中 附图V1.6.0.19
	laspoint.set_user_data(nType);
	{
		// 将分类写入到规范的字段里（1.内部只能存储小于32的分类值；2.提供给其他软件使用）
		auto iter = typeMapping.find(nType);
		if (typeMapping.end() != iter)
			nType = iter->second;
		laspoint.set_classification(nType);
	}
	laspoint.set_R((unsigned short)color.r() << LAS_TINY_RIGHT_MOVE);
	laspoint.set_G((unsigned short)color.g() << LAS_TINY_RIGHT_MOVE);
	laspoint.set_B((unsigned short)color.b() << LAS_TINY_RIGHT_MOVE);

	if (!pLaswriter->write_point(&laspoint))
		return false;
	pLaswriter->update_inventory(&laspoint);
	return true;
}