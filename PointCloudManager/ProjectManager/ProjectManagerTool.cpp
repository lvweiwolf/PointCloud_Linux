#include <ProjectManager/ProjectManagerTool.h>
#include <ProjectManager/ClusterFileLoader.h>
#include <BusinessNode/PCNodeType.h>

#include <include/Log.h>

#include <Tool/ModelNodeHelper.h>
#include <Tool/FileToolkit.h>


CBnsProjectNode CProjectManagerTool::LoadProject(const CString& strFile)
{
	d3s::CLog::Info("加载proje");
	pc::data::CModelNodePtr pProjectNode = nullptr;
	pc::data::CModelNodeXmlHelper::LoadFromFile(strFile, pProjectNode);
	if (nullptr == pProjectNode)
		pProjectNode = new pc::data::CModelNode(eBnsProjectRoot);

	// 加载簇管理数据
	if (CFileToolkit::FileExist(strFile))
	{
		const CString strDirectory = CFileToolkit::GetFileDirectory(strFile);
		const CString strClusterFileName = strDirectory + _T("/ClusterDataFile.xml");

		d3s::share_ptr<CClusterFileLoader> loader = new CClusterFileLoader();
		loader->LoadFile(pProjectNode->GetId(), strClusterFileName);
	}

	return pProjectNode;
}

bool CProjectManagerTool::SaveProject(const CString& strFile, CBnsProjectNode bnsProject)
{
	bool bProjectSave = pc::data::CModelNodeXmlHelper::SaveToFile(strFile, bnsProject);

	// 写入簇管理数据
	if (CFileToolkit::FileExist(strFile))
	{
		const CString strDirectory = CFileToolkit::GetFileDirectory(strFile);
		const CString strClusterFileName = strDirectory + _T("/ClusterDataFile.xml");

		d3s::share_ptr<CClusterFileLoader> saver = new CClusterFileLoader();
		saver->SaveFile(bnsProject.GetID(), strClusterFileName);
	}

	return bProjectSave;
}
