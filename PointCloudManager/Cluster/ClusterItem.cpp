#include <include/ClusterItem.h>
#include <include/ClusterManager.h>
#include <include/StringToolkit.h>

#include <Segment/PointCloudManagerDef.h>

const CString KeyID = _T("ID");						// id属性名
const CString KeyClassifyType = _T("ClassifyType"); // 分类类型属性名
const CString KeyBoundingBox = _T("BoundingBox");	// 包围盒属性名
const CString KeyPolygons = _T("Polygons");			// 多边形属性名

// CClusterItem
//////////////////////////////////////////////////////////////////////////
CClusterItem::CClusterItem(int nClassifyType,
						   std::map<CString, CString> propMap /* = std::map<CString, CString>{}*/)
	: _propMap(propMap)
{
	SetId(INVALID_CLUSTER_ID);
	SetClassifyType(nClassifyType);
}

CClusterItem::CClusterItem(const CClusterItem& copy)
	: _propMap(copy._propMap), _pageLodIdList(copy._pageLodIdList)
{
}

void CClusterItem::Save(d3s::share_ptr<CClusterItem> pCluster, toolkit::CXmlElement* pClusterEle)
{
	if (nullptr == pCluster || nullptr == pClusterEle)
		return;

	// 属性
	const std::map<CString, CString>& propMap = pCluster->GetProp();
	for (auto pPropIter : propMap)
		pClusterEle->AddAttrValue(pPropIter.first, pPropIter.second);

	// 关联pageLog标识ID
	const std::set<CString>& pageLodIdList = pCluster->GetPagedLodID();
	CString strPageLodIdText;

	for (auto strPageLodId : pageLodIdList)
	{
		if (!strPageLodIdText.IsEmpty())
			strPageLodIdText.Format(_T("%s;%s"), (LPCTSTR)strPageLodIdText, (LPCTSTR)strPageLodId);
		else
			strPageLodIdText = strPageLodId;
	}

	pClusterEle->SetElementText(strPageLodIdText);
}

d3s::share_ptr<CClusterItem> CClusterItem::Load(toolkit::CXmlElement* pClusterEle)
{
	if (nullptr == pClusterEle)
		return nullptr;

	// 属性
	std::map<CString, CString> propMap;
	toolkit::CXmlAttributes* pAttrs = pClusterEle->GetAttributes();
	size_t nAttrCount = pAttrs->GetCount();

	for (size_t j = 0; j < nAttrCount; ++j)
	{
		toolkit::CXmlAttribute* pAttr = pAttrs->GetAt(j);
		propMap[pAttr->GetAttrName()] = pAttr->GetStrValue();
	}

	// 关联pageLog标识ID
	CString strPageLodIdText = pClusterEle->GetElementText();
	std::vector<CString> pageLodIdList;
	CStringToolkit::SplitString(strPageLodIdText, pageLodIdList, L";");

	auto iter = propMap.find(_T("ClassifyType"));
	if (iter == propMap.end())
	{
		PC_ERROR(_T("从文件读取簇属性出错!"));
		return nullptr;
	}

	d3s::share_ptr<CClusterItem> pClusterItem =
		new CClusterItem(CStringToolkit::StrToInt(iter->second));
	pClusterItem->AddPagedLodID(pageLodIdList);
	pClusterItem->GetProp() = propMap;

	return pClusterItem;
}

int CClusterItem::GetId() { return CStringToolkit::StrToInt(_propMap[KeyID]); }

void CClusterItem::SetId(int nId) { _propMap[KeyID] = CStringToolkit::IntToStr(nId); }

int CClusterItem::GetClassifyType() { return CStringToolkit::StrToInt(_propMap[KeyClassifyType]); }

void CClusterItem::SetClassifyType(int nClassifyType)
{
	_propMap[KeyClassifyType] = CStringToolkit::IntToStr(nClassifyType);
}

CString CClusterItem::GetProp(const CString& strKey) { return _propMap[strKey]; }

std::map<CString, CString>& CClusterItem::GetProp() { return _propMap; }

void CClusterItem::SetProp(const CString& strKey, const CString& strValue)
{
	_propMap[strKey] = strValue;
}

void CClusterItem::AddPagedLodID(const CString& strPageLodId)
{
	_pageLodIdList.insert(strPageLodId);
}

void CClusterItem::AddPagedLodID(const std::vector<CString>& pageLodIdList)
{
	for (auto strPageLodId : pageLodIdList)
	{
		AddPagedLodID(strPageLodId);
	}
}

void CClusterItem::RemovePagedLodID(const CString& strPageLodId)
{
	_pageLodIdList.erase(strPageLodId);
}

const std::set<CString>& CClusterItem::GetPagedLodID() { return _pageLodIdList; }

osg::BoundingBox CClusterItem::GetBoundingBox()
{
	CString strBox = _propMap[KeyBoundingBox];
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

void CClusterItem::SetBoundingBox(const osg::BoundingBox& boundingBox)
{
	CString strBox;
	strBox.Format(L"%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
				  boundingBox.xMin(),
				  boundingBox.yMin(),
				  boundingBox.zMin(),
				  boundingBox.xMax(),
				  boundingBox.yMax(),
				  boundingBox.zMax());

	_propMap[KeyBoundingBox] = strBox;
}

CClusterItem::PolygonList CClusterItem::GetPolygons()
{
	PolygonList polygons;

	do
	{
		CString strPolygons = _propMap[KeyPolygons];
		std::vector<CString> polygonTokens;
		CStringToolkit::SplitString(strPolygons, polygonTokens, _T(" "));

		// 未能解析到点数组
		if (polygonTokens.empty())
			break;

		for (size_t i = 0; i < polygonTokens.size(); ++i)
		{
			const auto& strPolygon = polygonTokens.at(i);
			std::vector<CString> pointTokens;
			CStringToolkit::SplitString(strPolygon, pointTokens, _T(";"));

			if (pointTokens.empty())
				continue;

			Polygon polygon;
			for (size_t j = 0; j < pointTokens.size(); ++j)
			{
				const auto& strPoint = pointTokens.at(j);
				std::vector<CString> pointCoords;
				CStringToolkit::SplitString(strPoint, pointCoords, _T(","));

				if (pointCoords.size() != 3)
					continue;

				double x = CStringToolkit::StrToDouble(pointCoords[0]);
				double y = CStringToolkit::StrToDouble(pointCoords[1]);
				double z = CStringToolkit::StrToDouble(pointCoords[2]);

				polygon.push_back(osg::Vec3d(x, y, z));
			}

			if (!polygon.empty())
				polygons.push_back(polygon);
		}
	} while (false);

	return polygons;
}

void CClusterItem::SetPolygons(const PolygonList& polygons)
{
	CString strPolygons;

	for (size_t i = 0; i < polygons.size(); ++i)
	{
		const auto& polygon = polygons.at(i);

		if (i != 0)
			strPolygons += _T(" ");

		CString strPolygon;

		for (size_t j = 0; j < polygon.size(); ++j)
		{
			const auto& point = polygon.at(j);
			CString strPt;
			strPt.Format(_T("%.6f,%.6f,%.6f"), point.x(), point.y(), point.z());

			if (j != 0)
				strPolygon += _T(";");

			strPolygon += strPt;
		}

		strPolygons += strPolygon;
	}

	_propMap[KeyPolygons] = strPolygons;
}
