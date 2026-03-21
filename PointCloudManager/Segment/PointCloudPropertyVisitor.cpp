#include <Segment/PointCloudPropertyVisitor.h>
#include <LasFile/PointCloudToolkit.h>
#include <LasFile/PointCloudPropertyTool.h>

#include <osg/Geometry>


CPointCloudSetPropVisitor::CPointCloudSetPropVisitor(
	const pc::data::SVisitorInfos& visitorInfos,
	const CString& strPageNodeId,
	osg::NodeVisitor::TraversalMode traversalMode)
	: osg::NodeVisitor(traversalMode), _visitorInfos(visitorInfos), _strPageNodeId(strPageNodeId)
{
	_bExitTraversal = false;
	_bEffectiveTraversal = false;
	_bSliceSelect = false;

	if (_visitorInfos._visitorType == pc::data::SVisitorInfos::ePolygon)
	{
		InitInfo(_visitorInfos._visitorInfo);
		if (!visitorInfos._sliceVisitorInfo._coarsePolygonPoints.empty())
		{
			InitInfo(_visitorInfos._sliceVisitorInfo);
			_bSliceSelect = true;
		}
	}
}

CPointCloudSetPropVisitor::~CPointCloudSetPropVisitor() {}

void CPointCloudSetPropVisitor::apply(osg::Geometry& geometry)
{
	if (_bExitTraversal || nullptr == _visitorInfos._pPropVisitorCallback)
		return;

	osg::ref_ptr<osg::Vec3Array> pVertexArray =
		dynamic_cast<osg::Vec3Array*>(geometry.getVertexArray());

	osg::ref_ptr<osg::Vec2Array> pTexCoordArray =
		dynamic_cast<osg::Vec2Array*>(geometry.getTexCoordArray(0));

	if (!pVertexArray.valid() || !pTexCoordArray.valid())
		return;

	pc::data::SVisitorCallbackParam callbackParam;
	callbackParam._nodePath = getNodePath();
	callbackParam._pTexCoordArray = pTexCoordArray;
	callbackParam._pAdditionalParam = _visitorInfos._pAdditionalParam;
	callbackParam._strPageNodeId = _strPageNodeId;
	size_t nVertArraySize = pVertexArray->size();

	for (size_t nVertexIndex = 0; nVertexIndex < nVertArraySize; ++nVertexIndex)
	{
		if (_bExitTraversal)
			break;

		osg::Matrix matrix = osg::computeLocalToWorld(getNodePath());
		osg::Vec3d VertexPoint = matrix.preMult((*pVertexArray)[nVertexIndex]);

		if (!_visitorInfos._bAllTraversal && !ValildData(VertexPoint, pTexCoordArray, nVertexIndex))
			continue;

		callbackParam._nIndex = nVertexIndex;
		callbackParam._point = VertexPoint;
		_visitorInfos.ExpandBoundingBox(VertexPoint);

		if (_visitorInfos._pPropVisitorCallback(callbackParam))
			_bEffectiveTraversal = true;

		_bExitTraversal = callbackParam._bExitTraversal;
	}
}

void CPointCloudSetPropVisitor::apply(osg::Geode& node)
{
	if (_bExitTraversal)
		return;

	if (!_visitorInfos._bAllTraversal)
	{
		osg::Matrix matrix = osg::computeLocalToWorld(getNodePath());
		osg::BoundingSphere boundShere = node.getBound();
		boundShere._center = boundShere.center() * matrix;
		osg::BoundingBox boundBox;
		boundBox.expandBy(boundShere);

		switch (_visitorInfos._visitorType)
		{
		case pc::data::SVisitorInfos::ePolygon:
			if (!CPointCloudToolkit::JudgePolygonInModel(
					boundBox,
					_visitorInfos._visitorInfo._coarseVPW,
					_visitorInfos._visitorInfo._coarsePolygonPoints))
			{
				return;
			}

			break;
		case pc::data::SVisitorInfos::eBoundingBoxXY:
			boundBox.zMin() = _visitorInfos._visitorInfo._boundingBox.zMin();
			boundBox.zMax() = _visitorInfos._visitorInfo._boundingBox.zMax();
			if (!_visitorInfos._visitorInfo._boundingBox.intersects(boundBox))
				return;

			break;
		case pc::data::SVisitorInfos::eBoundingBox:
			if (!_visitorInfos._visitorInfo._boundingBox.intersects(boundBox))
				return;

			break;
		case pc::data::SVisitorInfos::ePolygonXY:
		{
			osg::BoundingBox boundBoxXY;

			for (const auto& iter : _visitorInfos._visitorInfo._coarsePolygonPoints)
				boundBoxXY.expandBy(iter);

			boundBoxXY.zMin() = boundBoxXY.zMax() = boundBox.zMin();

			if (!boundBoxXY.intersects(boundBox))
				return;

			break;
		}
		case pc::data::SVisitorInfos::eCircular:
		{
			osg::BoundingBox boundBoxCircularXY;
			const auto& center = _visitorInfos._visitorInfo._circularParam._center;
			const auto& dR = _visitorInfos._visitorInfo._circularParam._dR;
			boundBoxCircularXY.expandBy(osg::Vec3d(center.x() + dR, center.y(), 0.0));
			boundBoxCircularXY.expandBy(osg::Vec3d(center.x() - dR, center.y(), 0.0));
			boundBoxCircularXY.expandBy(osg::Vec3d(center.x(), center.y() + dR, 0.0));
			boundBoxCircularXY.expandBy(osg::Vec3d(center.x(), center.y() - dR, 0.0));
			boundBoxCircularXY.zMin() = boundBoxCircularXY.zMax() = boundBox.zMin();

			if (!boundBoxCircularXY.intersects(boundBox))
				return;

			break;
		}
		default:
			break;
		}
	}

	osg::NodeVisitor::apply(node);
}

void CPointCloudSetPropVisitor::InitInfo(pc::data::SVisitorInfo& info)
{
	size_t nPointVecSize = info._coarsePolygonPoints.size();

	for (size_t i = 0; i < nPointVecSize; ++i)
	{
		osg::Vec3d point = info._coarsePolygonPoints[i];

		if (i == 0)
		{
			info._coarsePolygonLimit._nMaxX = ceil(point.x());
			info._coarsePolygonLimit._nMinX = ceil(point.x());
			info._coarsePolygonLimit._nMaxY = ceil(point.y());
			info._coarsePolygonLimit._nMinY = ceil(point.y());
		}
		else
		{
			info._coarsePolygonLimit._nMaxX =
				std::max<int>(info._coarsePolygonLimit._nMaxX, ceil(point.x()));
			info._coarsePolygonLimit._nMinX =
				std::min<int>(info._coarsePolygonLimit._nMinX, ceil(point.x()));
			info._coarsePolygonLimit._nMaxY =
				std::max<int>(info._coarsePolygonLimit._nMaxY, ceil(point.y()));
			info._coarsePolygonLimit._nMinY =
				std::min<int>(info._coarsePolygonLimit._nMinY, ceil(point.y()));
		}
	}
}

bool CPointCloudSetPropVisitor::PointInPolygonXY(const osg::Vec3d& point,
												 const std::vector<osg::Vec3d>& polygon)
{
	return CPointCloudToolkit::PtInPolygon(point, polygon);
	/*int nIntersectionCount = 0;
	size_t nCount = polygon.size();
	for (size_t i = 0; i < nCount; ++i)
	{
		const osg::Vec3d& p1 = polygon[i];
		const osg::Vec3d& p2 = polygon[(i + 1) % nCount];

		if ((p1.y() > point.y()) != (p2.y() > point.y())
			&& point.x() < (p2.x() - p1.x()) * (point.y() - p1.y()) / (p2.y() - p1.y()) + p1.x())
			++nIntersectionCount;
	}
	return nIntersectionCount % 2 == 1;*/
}

bool CPointCloudSetPropVisitor::GetPointInPolygonIndex(const osg::Vec3d& VertexPoint,
													   const osg::Matrix& matrixVPW,
													   const pc::data::SLimitValue& polygonLimit,
													   int& nIndex)
{
	osg::Vec3d screenPoint = VertexPoint * matrixVPW;
	int nX = ceil(screenPoint.x());
	int nY = ceil(screenPoint.y());

	if (nX > polygonLimit._nMaxX || nX < polygonLimit._nMinX || nY < polygonLimit._nMinY ||
		nY > polygonLimit._nMaxY)
	{
		return false;
	}

	nIndex = (nX - polygonLimit._nMinX) * (polygonLimit._nMaxY - polygonLimit._nMinY + 1) +
			 (nY - polygonLimit._nMinY);
	return true;
}

bool CPointCloudSetPropVisitor::ValildData(const osg::Vec3d& VertexPoint,
										   pc::data::SVisitorInfo& info)
{
	int nCoarseIndex = 0;

	if (!GetPointInPolygonIndex(VertexPoint,
								info._coarseVPW,
								info._coarsePolygonLimit,
								nCoarseIndex))
	{
		return false;
	}

	if (nCoarseIndex >= (int)info._coarsePolygon.size() || nCoarseIndex < 0)
		return false;

	char cValue = info._coarsePolygon[nCoarseIndex];

	if (pc::data::SVisitorInfo::g_cOutPolygon == cValue)
	{
		return false;
	}
	else if (pc::data::SVisitorInfo::g_cInPolygon == cValue || info._finePolygonMap.empty())
	{
		return true;
	}

	int nFineIndex = 0;

	if (!GetPointInPolygonIndex(VertexPoint,
								info._fineVPW,
								info._finePolygonMap[nCoarseIndex].first,
								nFineIndex))
	{
		return false;
	}

	if (nFineIndex >= (int)info._finePolygonMap[nCoarseIndex].second.size() || nFineIndex < 0)
		return false;

	return info._finePolygonMap[nCoarseIndex].second[nFineIndex];
}

bool CPointCloudSetPropVisitor::ValildData(const osg::Vec3d& VertexPoint,
										   osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
										   const size_t& nIndex)
{
	switch (_visitorInfos._visitorType)
	{
	case pc::data::SVisitorInfos::ePolygon:
	{
		if (!ValildData(VertexPoint, _visitorInfos._visitorInfo))
			return false;

		if (_bSliceSelect && !ValildData(VertexPoint, _visitorInfos._sliceVisitorInfo))
			return false;

		break;
	}
	case pc::data::SVisitorInfos::eBoundingBoxXY:
	{
		osg::Vec3d point = VertexPoint;
		point.z() = _visitorInfos._visitorInfo._boundingBox.center().z();

		if (!_visitorInfos._visitorInfo._boundingBox.contains(point))
			return false;

		break;
	}
	case pc::data::SVisitorInfos::eBoundingBox:
	{
		osg::Vec3d point = VertexPoint;

		if (!_visitorInfos._visitorInfo._boundingBox.contains(point))
			return false;

		break;
	}
	case pc::data::SVisitorInfos::ePolygonXY:
	{
		osg::Vec3d point = VertexPoint;

		if (!PointInPolygonXY(point, _visitorInfos._visitorInfo._coarsePolygonPoints))
			return false;

		break;
	}
	case pc::data::SVisitorInfos::eCircular:
	{
		osg::Vec3d point = VertexPoint;
		const auto& center = _visitorInfos._visitorInfo._circularParam._center;
		const auto& dR = _visitorInfos._visitorInfo._circularParam._dR;
		point.z() = center.z();

		if ((point - center).length() > dR)
			return false;

		break;
	}
	default:
		break;
	}

	if (CPointCloudPropertyTool::IsHide(pTexCoordArray, nIndex, _visitorInfos._hideSegmentList))
		return false;

	return true;
}