#include <ProjectManager/ClusterFileLoader.h>
#include <include/ClusterManagerSet.h>

#include <Tool/XmlDocument.h>
#include <Tool/FileToolkit.h>

void CClusterFileLoader::LoadFile(const CString& strProjectID, const CString& strFileName)
{
	using namespace toolkit;

	CXmlDocument xmlDoc;

	if (!xmlDoc.LoadFile(strFileName, fmtXML))
		return;

	auto pClusterMgr = CClusterManagerSet::GetInst()->GetClusterManager(strProjectID);
	CXmlElement* pRoot = xmlDoc.GetElementRoot();

	// 簇对象属性
	CXmlElement* pClusterItemRoot = pRoot->GetChildElementAt(L"ClusterItem");
	CXmlElements* pClusterList = pClusterItemRoot->GetChildElements();

	size_t nCount = pClusterList->GetCount();
	for (size_t i = 0; i < nCount; ++i)
	{
		CXmlElement* pClusterEle = pClusterList->GetAt(i);
		d3s::share_ptr<CClusterItem> pCluster = CClusterItem::Load(pClusterEle);
		pClusterMgr->AddClusterItem(pCluster);
	}
}

void CClusterFileLoader::SaveFile(const CString& strProjectID, const CString& strFileName)
{
	using namespace toolkit;

	CString strRealFile = strFileName;

	// if (pDoc->GetDocument()->GetChildFileName(CLUSTER_LOAD_KEY).IsEmpty())
	// {
	// 	pDoc->GetDocument()->SetChildFileName(CLUSTER_LOAD_KEY, CLUSTER_FILE_NAME);
	// 	strRealFile = pDoc->GetDocument()->GetChildFileName(CLUSTER_LOAD_KEY);
	// }

	auto pClusterMgr = CClusterManagerSet::GetInst()->GetClusterManager(strProjectID);
	std::map<int, d3s::share_ptr<CClusterItem>> clusterItemMap = pClusterMgr->GetClusterMap();

	toolkit::CXmlDocument xmlDoc;
	CXmlElement* pRoot = xmlDoc.GetElementRoot();

	// 簇对象属性
	CXmlElement* pClusterItemRoot = pRoot->GetChildElements(TRUE)->Add();
	pClusterItemRoot->SetElementName(L"ClusterItem");

	for (auto pIter : clusterItemMap)
	{
		CClusterItem::Save(pIter.second, pClusterItemRoot->GetChildElements(TRUE)->Add());
	}

	xmlDoc.SaveFile(strRealFile, fmtXMLUTF8);
}


void CClusterFileLoader::CloseFile(const CString& strProjectID)
{
	CClusterManagerSet::GetInst()->RemoveClusterManager(strProjectID);
}