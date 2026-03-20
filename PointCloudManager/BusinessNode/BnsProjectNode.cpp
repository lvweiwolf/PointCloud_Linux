#include <BusinessNode/BnsProjectNode.h>
#include <BusinessNode/PCNodeType.h>

const CString STRING_PROJECT_NNAME = L"Project_Name";
const CString STRING_BASEPOS_NAME = L"BasePostion";
const CString STRING_POINT_EPSG = L"PointEpsg";
const CString STRING_POINT_SCANTIME = L"PCScanTime";

CBnsProjectNode::~CBnsProjectNode() {}

void CBnsProjectNode::SetProjectName(const CString& strName)
{
	if (nullptr != _pNode)
	{
		_pNode->SetData(STRING_PROJECT_NNAME, strName);
	}
}

CString CBnsProjectNode::GetProjectName()
{
	return nullptr == _pNode ? L"" : _pNode->GetData(STRING_PROJECT_NNAME);
}

CBnsPointCloudNode CBnsProjectNode::AddPointCloudNode()
{
	if (nullptr == _pNode)
		return CBnsPointCloudNode();

	pc::data::CModelNodePtr pPCArr =
		_pNode->FindSubNode(EProjectNodeType::eBnsPointCloudLodNodeArr, true);
	if (nullptr == pPCArr)
		return CBnsPointCloudNode();

	return pPCArr->InsertNode(-1, eBnsPointCloudLodNode);
}

bool CBnsProjectNode::InsertPointCloudNode(pc::data::CModelNodePtr pPointNode)
{
	if (nullptr == _pNode)
		return false;

	pc::data::CModelNodePtr pPCArr =
		_pNode->FindSubNode(EProjectNodeType::eBnsPointCloudLodNodeArr, true);
	if (nullptr == pPCArr)
		return false;

	pPCArr->InsertNode(pPointNode, -1);

	return true;
}

pc::data::CModelNodeVector CBnsProjectNode::GetPointCloudNodeList()
{
	pc::data::CModelNodeVector nodes;
	if (nullptr == _pNode)
		return nodes;

	pc::data::CModelNodePtr pPCArr =
		_pNode->FindSubNode(EProjectNodeType::eBnsPointCloudLodNodeArr, false);
	if (nullptr == pPCArr)
		return nodes;

	nodes = pPCArr->GetSubNodes();

	return nodes;
}


void CBnsProjectNode::setBasePos(const osg::Vec3d& pos)
{
	if (nullptr != _pNode)
	{
		_pNode->SetData(STRING_BASEPOS_NAME, pos);
	}
}

osg::Vec3d CBnsProjectNode::getBasePos()
{
	return nullptr == _pNode ? L"" : _pNode->GetData(STRING_BASEPOS_NAME);
}


void CBnsProjectNode::SetEpsg(int nEpsg)
{
	if (nullptr != _pNode)
	{
		_pNode->SetData(STRING_POINT_EPSG, nEpsg);
	}
}

int CBnsProjectNode::GetEpsg()
{
	return nullptr == _pNode ? L"" : _pNode->GetData(STRING_POINT_EPSG);
}

void CBnsProjectNode::SetScanTime(const CString& strScanTime)
{
	if (nullptr != _pNode)
	{
		_pNode->SetData(STRING_POINT_SCANTIME, strScanTime);
	}
}

CString CBnsProjectNode::GetScanTime()
{
	return nullptr == _pNode ? L"" : _pNode->GetData(STRING_POINT_SCANTIME);
}
