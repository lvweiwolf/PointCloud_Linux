#include <src/algorithm/geometry2d_op.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Gps_circle_segment_traits_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include <osg/Vec3d>
#include <osg/Vec2>
#include <osg/Vec3>
#include <osg/BoundingBox>

#include <algorithm>
// #pragma comment(lib, "libmpfr-4.lib") // CGAL 依赖库
// #pragma comment(lib, "libgmp-10.lib") // CGAL 依赖库


typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;
typedef Kernel::Circle_2 Circle_2;
typedef Kernel::Iso_rectangle_2 Rectangle_2;
typedef CGAL::Gps_circle_segment_traits_2<Kernel> Traits_2;

typedef CGAL::Polygon_2<Kernel> Polygon_2;

namespace d3s {
	namespace pcs {

		bool rectIntersect(const Rect& r1, const Rect& r2)
		{
			// 如果r1右侧在r2左侧 或 r2右侧在r1左侧
			if (r1.xmax < r2.xmin || r2.xmax < r1.xmin)
				return false;

			// 如果r1下侧在r2上侧 或 r2下侧在r1上侧
			if (r1.ymax < r2.ymin || r2.ymax < r1.ymin)
				return false;

			return true;
		}

		double rectIntersectionArea(const Rect& r1, const Rect& r2)
		{
			if (!rectIntersect(r1, r2))
			{
				return 0.0;
			}

			double x1 = std::max(r1.xmin, r2.xmin);
			double y1 = std::max(r1.ymin, r2.ymin);
			double x2 = std::min(r1.xmax, r2.xmax);
			double y2 = std::min(r1.ymax, r2.ymax);

			return (x2 - x1) * (y2 - y1);
		}

		bool pointInPolygon(const std::vector<osg::Vec3d>& polygon, double x, double y)
		{
			int j = polygon.size() - 1;
			bool oddNodes = false;

			for (int i = 0; i < (int)polygon.size(); i++)
			{
				if (((polygon[i].y() < y && polygon[j].y() >= y) ||
					 (polygon[j].y() < y && polygon[i].y() >= y)) &&
					(polygon[i].x() <= x || polygon[j].x() <= x))
				{
					oddNodes ^=
						(polygon[i].x() + (y - polygon[i].y()) / (polygon[j].y() - polygon[i].y()) *
											  (polygon[j].x() - polygon[i].x()) <
						 x);
				}

				j = i;
			}

			return oddNodes;
		}

		bool polygonIntersect2D(const std::vector<osg::Vec3d>& polygon1,
								const std::vector<osg::Vec3d>& polygon2)
		{

			Polygon_2 P, Q;

			for (size_t i = 0; i < polygon1.size(); ++i)
			{
				const osg::Vec3d& v = polygon1[i];
				P.push_back(Point_2(v.x(), v.y()));
			}

			for (size_t i = 0; i < polygon2.size(); ++i)
			{
				const osg::Vec3d& v = polygon2[i];
				Q.push_back(Point_2(v.x(), v.y()));
			}

			return CGAL::do_intersect(P, Q);
		}

		bool bboxCircleIntersect2D(const osg::BoundingBox& bbox,
								   const osg::Vec3d& center,
								   double radius)
		{

			const osg::Vec3d& min = bbox._min;
			const osg::Vec3d& max = bbox._max;

			Rectangle_2 rectangle(Point_2(min.x(), min.y()), Point_2(max.x(), max.y()));
			Circle_2 circle(Point_2(center.x(), center.y()), radius * radius);

			return CGAL::do_intersect(rectangle, circle);
		}


		bool lineIntersection(double xA,
							  double yA,
							  double xB,
							  double yB,
							  double xC,
							  double yC,
							  double xD,
							  double yD,
							  double& x,
							  double& y)
		{
			// Line AB represented as a1x + b1y = c1
			double a1 = yB - yA;
			double b1 = xA - xB;
			double c1 = a1 * (xA) + b1 * (yA);

			// Line CD represented as a2x + b2y = c2
			double a2 = yD - yC;
			double b2 = xC - xD;
			double c2 = a2 * (xC) + b2 * (yC);

			double determinant = a1 * b2 - a2 * b1;

			if (determinant == 0)
			{
				// The lines are parallel. This is simplified
				// by returning a pair of FLT_MAX
				return false;
			}
			else
			{
				x = (b2 * c1 - b1 * c2) / determinant;
				y = (a1 * c2 - a2 * c1) / determinant;
				return true;
			}
		}

		bool lineSegmentIntersection(osg::Vec2 a,
									 osg::Vec2 b,
									 osg::Vec2 c,
									 osg::Vec2 d,
									 double& x,
									 double& y)
		{
			/*
			auto area_abc = (a.x() - c.x()) * (b.y() - c.y()) - (a.y() - c.y()) * (b.x() - c.x());
			auto area_abd = (a.x() - d.x()) * (b.y() - d.y()) - (a.y() - d.y()) * (b.x() - d.x());

			if (area_abc * area_abd >= 0)
				return false;

			auto area_cda = (c.x() - a.x()) * (d.y() - a.y()) - (c.y() - a.y()) * (d.x() - a.x());

			// 三角形cdb 面积的2倍
			auto area_cdb = area_cda + area_abc - area_abd;

			if (area_cda * area_cdb >= 0)
				return false;

			//计算交点坐标
			auto t = area_cda / (area_abd - area_abc);
			auto dx = t * (b.x() - a.x());
			auto dy = t * (b.y() - a.y());

			x = a.x() + dx;
			y = a.y() + dy;

			return true;
			*/
			double s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
			s10_x = b.x() - a.x();
			s10_y = b.y() - a.y();
			s32_x = d.x() - c.x();
			s32_y = d.y() - c.y();

			denom = s10_x * s32_y - s32_x * s10_y;
			if (denom == 0)
				return false; // Collinear

			bool denomPositive = denom > 0;

			s02_x = a.x() - c.x();
			s02_y = a.y() - c.y();
			s_numer = s10_x * s02_y - s10_y * s02_x;

			if ((s_numer < 0) == denomPositive)
				return false; // No collision

			t_numer = s32_x * s02_y - s32_y * s02_x;

			if ((t_numer < 0) == denomPositive)
				return false; // No collision

			if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
				return false; // No collision

			// Collision detected
			t = t_numer / denom;
			x = a.x() + (t * s10_x);
			y = a.y() + (t * s10_y);

			return true;
		}

		double distanceToSegment(const osg::Vec2d& point,
								 const std::array<osg::Vec2d, 2>& lineSegment)
		{
			Point_2 p(point.x(), point.y());
			Segment_2 seg(Point_2(lineSegment[0].x(), lineSegment[0].y()),
						  Point_2(lineSegment[1].x(), lineSegment[1].y()));

			return CGAL::to_double(CGAL::squared_distance(seg, p));
		}

		double getMaxProjection(const osg::Vec3d& p0,
								const osg::Vec3d& p1,
								const std::vector<osg::Vec3d>& polygons)
		{
			osg::Vec3d d1 = p1 - p0;
			d1.normalize();
			// auto d2 = d1 ^ osg::Z_AXIS;	 // 垂直向量
			double max_projection = 0.0; // 规避铁塔，沿线路方向收缩

			// 铁塔边界多边形顶点在 d1 上的最大投影点
			for (size_t i = 0; i < polygons.size(); ++i)
			{
				const osg::Vec3d& vert = polygons.at(i);

				osg::Vec3 u = vert - p0;
				double length = u.length();
				u.normalize();
				double projection = d1 * u * length;
				max_projection = std::max(projection, max_projection);
			}

			return max_projection;
		}

		osg::BoundingBox getPolygonBound(const std::vector<osg::Vec3d>& polygon)
		{
			osg::BoundingBox bbox;

			for (size_t i = 0; i < polygon.size(); ++i)
			{
				const osg::Vec3d& p = polygon.at(i);
				bbox.expandBy(p);
			}

			return bbox;
		}

	}
}