#include "../stdafx.h"
#include "DataManager.h"

// 属性名（Type）
const	CString			STRING_TYPE_INDEX = L"Type";
// 属性名（Type）
const	CString			STRING_SEG_TYPE_INDEX = L"SegType";


CString GetTypeConvertFile()
{
	CString strPath = L"/TypeConvert.xml";
	return strPath;
}

CDataManager::CDataManager()
{

}

CDataManager::~CDataManager()
{

}

std::map<int, int> CDataManager::GetTypeConvertInfo()
{
	std::map<int, int> mapConvertType;

	CString strAppPath = CLibToolkit::GetAppModuleDir(nullptr);
	CString strCfgPath = strAppPath + GetTypeConvertFile();

	toolkit::CXmlDocument doc;
	if (!doc.LoadFile(strCfgPath, toolkit::fmtXMLUTF8))
	{
		return mapConvertType;
	}
	toolkit::CXmlElement* pRoot = doc.GetElementRoot();
	if (nullptr == pRoot)
	{
		return mapConvertType;
	}
	toolkit::CXmlElements* pElements = pRoot->GetChildElements(FALSE);
	if (nullptr == pElements)
	{
		return mapConvertType;
	}
	int nCount = pElements->GetCount();
	toolkit::CXmlElement* pElement = nullptr;
	int nPropIndex = -1;
	for (int i = 0; i < nCount; ++i)
	{
		pElement = pElements->GetAt(i);
		
		if (!pElement->FindAttribute(STRING_SEG_TYPE_INDEX, nPropIndex) ||
			!pElement->FindAttribute(STRING_TYPE_INDEX, nPropIndex))
			continue;

		int nSegTypeIndex = CStringToolkit::StrToInt(pElement->GetAttrValue(STRING_SEG_TYPE_INDEX));
		int nPointCloudType = CStringToolkit::StrToInt(pElement->GetAttrValue(STRING_TYPE_INDEX));
		auto iter = mapConvertType.find(nSegTypeIndex);
		mapConvertType.insert(std::make_pair(nSegTypeIndex, nPointCloudType));
	}

	return mapConvertType;
}
