#include <ProjectManager/ProjectManagerTool.h>
#include <BusinessNode/PCNodeType.h>

#include <Tool/ModelNodeHelper.h>

CBnsProjectNode CProjectManagerTool::LoadProject(const CString& strFile)
{
	pc::data::CModelNodePtr pProjectNode = nullptr;
	pc::data::CModelNodeXmlHelper::LoadFromFile(strFile, pProjectNode);
	if (nullptr == pProjectNode)
		pProjectNode = new pc::data::CModelNode(eBnsProjectRoot);

	return pProjectNode;
}

bool CProjectManagerTool::SaveProject(const CString& strFile, CBnsProjectNode bnsProject)
{
	return pc::data::CModelNodeXmlHelper::SaveToFile(strFile, bnsProject);
}
