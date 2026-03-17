#include <BusinessNode/BnsPointCloudNode.h>

const CString STRING_FILE_NAME = L"FileName";
const CString STRING_DATABASE_PATH = L"DatabasePath";
const CString STRING_OFFSET_NAME = L"Offset";
const CString STRING_CENTER_NAME = L"CenterPos";
const CString STRING_BOUNDING_BOX_KEY = L"ModelBoundingBox";	 // 包围盒属性key
const CString STRING_REAL_BOUNDING_BOX_KEY = L"RealBoundingBox"; // 包围盒属性key
const CString STRING_RADIUS_KEY = L"Radius";					 // 包围盒属性key
const CString STRING_POINTNUM_KEY = L"PointNum";				 // 包围盒属性key
const CString STRING_POINT_LEVEL = L"PointLevel";				 // 层级

CBnsPointCloudNode::~CBnsPointCloudNode() {}

void CBnsPointCloudNode::setFileName(const CString& strName)
{
	if (nullptr != _pNode)
	{
		_pNode->SetData(STRING_FILE_NAME, strName);
	}
}

CString CBnsPointCloudNode::getFileName()
{
	return nullptr == _pNode ? L"" : _pNode->GetData(STRING_FILE_NAME);
}

void CBnsPointCloudNode::setDatabasePath(const CString& strName)
{
	if (nullptr != _pNode)
	{
		_pNode->SetData(STRING_DATABASE_PATH, strName);
	}
}

CString CBnsPointCloudNode::getDatabasePath()
{
	return nullptr == _pNode ? L"" : _pNode->GetData(STRING_DATABASE_PATH);
}

void CBnsPointCloudNode::setOffset(const osg::Vec3d& pos)
{
	if (nullptr != _pNode)
	{
		_pNode->SetData(STRING_OFFSET_NAME, pos);
	}
}

osg::Vec3d CBnsPointCloudNode::getOffset()
{
	return nullptr == _pNode ? L"" : _pNode->GetData(STRING_OFFSET_NAME);
}

void CBnsPointCloudNode::setCenter(const osg::Vec3d& pos)
{
	if (nullptr != _pNode)
	{
		_pNode->SetData(STRING_CENTER_NAME, pos);
	}
}

osg::Vec3d CBnsPointCloudNode::getCenter()
{
	return nullptr == _pNode ? L"" : _pNode->GetData(STRING_CENTER_NAME);
}

void CBnsPointCloudNode::SetModelBoundingBox(const osg::BoundingBox& boundingBox)
{
	if (nullptr != _pNode)
	{
		CString strBox;
		strBox.Format(L"%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
					  boundingBox.xMin(),
					  boundingBox.yMin(),
					  boundingBox.zMin(),
					  boundingBox.xMax(),
					  boundingBox.yMax(),
					  boundingBox.zMax());
		_pNode->SetData(STRING_BOUNDING_BOX_KEY, strBox);
	}
}

osg::BoundingBox CBnsPointCloudNode::GetModelBoundingBox()
{
	CString strBox = (nullptr == _pNode ? L"" : _pNode->GetData(STRING_BOUNDING_BOX_KEY));
	std::vector<CString> strSplit;
	CStringToolkit::SplitString(strBox, strSplit, L",");
	if (strSplit.size() != 6)
		return osg::BoundingBox();
	double dXMin = CStringToolkit::StrToDouble(strSplit[0]);
	double dYMin = CStringToolkit::StrToDouble(strSplit[1]);
	double dZMin = CStringToolkit::StrToDouble(strSplit[2]);
	double dXMax = CStringToolkit::StrToDouble(strSplit[3]);
	double dYMax = CStringToolkit::StrToDouble(strSplit[4]);
	double dZMax = CStringToolkit::StrToDouble(strSplit[5]);
	return osg::BoundingBox(dXMin, dYMin, dZMin, dXMax, dYMax, dZMax);
}

void CBnsPointCloudNode::SetRealBoundingBox(const osg::BoundingBox& boundingBox)
{
	if (nullptr != _pNode)
	{
		CString strBox;
		strBox.Format(L"%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
					  boundingBox.xMin(),
					  boundingBox.yMin(),
					  boundingBox.zMin(),
					  boundingBox.xMax(),
					  boundingBox.yMax(),
					  boundingBox.zMax());
		_pNode->SetData(STRING_REAL_BOUNDING_BOX_KEY, strBox);
	}
}

osg::BoundingBox CBnsPointCloudNode::GetRealBoundingBox()
{
	CString strBox = (nullptr == _pNode ? L"" : _pNode->GetData(STRING_REAL_BOUNDING_BOX_KEY));
	std::vector<CString> strSplit;
	CStringToolkit::SplitString(strBox, strSplit, L",");
	if (strSplit.size() != 6)
		return osg::BoundingBox();
	double dXMin = CStringToolkit::StrToDouble(strSplit[0]);
	double dYMin = CStringToolkit::StrToDouble(strSplit[1]);
	double dZMin = CStringToolkit::StrToDouble(strSplit[2]);
	double dXMax = CStringToolkit::StrToDouble(strSplit[3]);
	double dYMax = CStringToolkit::StrToDouble(strSplit[4]);
	double dZMax = CStringToolkit::StrToDouble(strSplit[5]);
	return osg::BoundingBox(dXMin, dYMin, dZMin, dXMax, dYMax, dZMax);
}

void CBnsPointCloudNode::setRadius(double Radius)
{
	if (nullptr != _pNode)
	{
		_pNode->SetData(STRING_RADIUS_KEY, Radius);
	}
}

double CBnsPointCloudNode::getRadius()
{
	return nullptr == _pNode ? L"" : _pNode->GetData(STRING_RADIUS_KEY);
}

void CBnsPointCloudNode::SetPointsNum(int nPointNum)
{
	if (nullptr != _pNode)
	{
		_pNode->SetData(STRING_POINTNUM_KEY, nPointNum);
	}
}

int CBnsPointCloudNode::GetPointsNum(void)
{
	return nullptr == _pNode ? L"" : _pNode->GetData(STRING_POINTNUM_KEY);
}

void CBnsPointCloudNode::SetPcLevel(int nLevel)
{
	if (nullptr != _pNode)
	{
		_pNode->SetData(STRING_POINT_LEVEL, nLevel);
	}
}

int CBnsPointCloudNode::GetPcLevel(void)
{
	return nullptr == _pNode ? L"" : _pNode->GetData(STRING_POINT_LEVEL);
}
