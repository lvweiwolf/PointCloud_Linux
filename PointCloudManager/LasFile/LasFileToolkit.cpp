#include <LasFile/LasFileToolkit.h>
#include <LasFile/PointCloudTool.h>
#include <LasFile/ExportLasTool.h>

#include <include/BnsWrapper.h>


// 存EPSG信息的Key值
const U16 INT_SAVE_EPAG_KEY = 3072;

void CLasFileToolkit::ImportLasFile(const std::vector<CString>& lasPathList,
									pc::data::CModelNodePtr pProjectNode,
									const CString& strOutPath)
{
	if (lasPathList.empty())
		return;

	int nEpsg = 32650;
	GetPointCloudEpsg(nEpsg, lasPathList.at(0));

	for (const CString& lasFile : lasPathList)
	{
		CPointCloudTool tool(lasFile, pProjectNode, strOutPath);
		tool.LoadLas();
	}

	CBnsProjectNode bnsProject = pProjectNode;
	if (!bnsProject.IsNull())
	{
		bnsProject.SetEpsg(nEpsg);
	}
}


bool CLasFileToolkit::GetPointCloudEpsg(int& nOutEpsg, const CString& strLasFile)
{
	bool bReturn = false;
	std::string strFilePath = CStringToolkit::CStringToUTF8(strLasFile);
	LASreadOpener lasReadOpener;
	lasReadOpener.set_file_name(strFilePath.c_str());
	LASreader* pLasReader = lasReadOpener.open();
	if (nullptr == pLasReader)
	{
		return bReturn;
	}
	// 获取点云度带信息
	LASvlr_geo_keys* pLasClrGeoKeys = pLasReader->header.vlr_geo_keys;
	if (nullptr == pLasClrGeoKeys)
	{
		// d3s::CLog::Error(L"（%s）文件LASvlr_geo_keys为空!", strLasFile);
	}
	else
	{
		int nKeySize = pLasClrGeoKeys->number_of_keys;
		for (int i = 0; i < nKeySize; ++i)
		{
			U16 nKeyId = (pLasReader->header.vlr_geo_key_entries + i)->key_id;
			if (INT_SAVE_EPAG_KEY != nKeyId)
				continue;
			nOutEpsg = (pLasReader->header.vlr_geo_key_entries + i)->value_offset;
			bReturn = true;
			break;
		}
	}
	pLasReader->close();
	delete pLasReader;
	return bReturn;
}

void CLasFileToolkit::ExportLasFile(pc::data::CModelNodePtr pProjectNode, const CString& strOutPath)
{
	CExportLasTool exportTool(pProjectNode, strOutPath);
	exportTool.ExportLas();
}
