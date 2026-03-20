//////////////////////////////////////////////////////////////////////
// 文件名称：geometry2d_op.h
// 功能描述：2D 几何图像处理
// 创建标识：吕伟	2022/4/27
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef GEOMETRY2D_OP_H_
#define GEOMETRY2D_OP_H_

#include <osg/Vec3d>
#include <osg/Vec2>
#include <osg/BoundingBox>

#include <vector>

namespace d3s {
	namespace pcs {

		struct Rect
		{
			Rect(double xmin_, double ymin_, double xmax_, double ymax_)
				: xmin(xmin_), ymin(ymin_), xmax(xmax_), ymax(ymax_)
			{
			}

			double xmin, ymin, xmax, ymax;
		};

		bool rectIntersect(const Rect& r1, const Rect& r2);

		double rectIntersectionArea(const Rect& r1, const Rect& r2);

		/**
		 *  @brief    点是否在多边形内
		 *
		 *  @prarm	 const std::vector<osg::Vec3d> & polygon			多边形顶点
		 *  @prarm	 double x											点云 x坐标
		 *  @prarm	 double y											点云 y坐标
		 *
		 *  @return   bool
		 */
		bool pointInPolygon(const std::vector<osg::Vec3d>& polygon, double x, double y);


		bool polygonIntersect2D(const std::vector<osg::Vec3d>& polygon1,
								const std::vector<osg::Vec3d>& polygon2);


		bool bboxCircleIntersect2D(const osg::BoundingBox& bbox,
								   const osg::Vec3d& center,
								   double radius);

		/**
		 *  @brief    线段求交
		 *
		 *  @prarm	 double xA		线段AB A.x
		 *  @prarm	 double yA		线段AB A.y
		 *  @prarm	 double xB		线段AB B.x
		 *  @prarm	 double yB		线段AB B.x
		 *  @prarm	 double xC		线段CD C.x
		 *  @prarm	 double yC		线段CD C.y
		 *  @prarm	 double xD		线段CD D.x
		 *  @prarm	 double yD		线段CD D.y
		 *  @prarm	 double & x
		 *  @prarm	 double & y
		 *
		 *  @return   bool
		 */
		bool lineIntersection(double xA,
							  double yA,
							  double xB,
							  double yB,
							  double xC,
							  double yC,
							  double xD,
							  double yD,
							  double& x,
							  double& y);

		bool lineSegmentIntersection(osg::Vec2 a,
									 osg::Vec2 b,
									 osg::Vec2 c,
									 osg::Vec2 d,
									 double& x,
									 double& y);


		double distanceToSegment(const osg::Vec2d& point,
								 const std::array<osg::Vec2d, 2>& lineSegment);

		/**
		 *  @brief    获得边界顶点在方向上的最大投影
		 *
		 *  @prarm	 const osg::Vec3d & p0		向量起点
		 *  @prarm	 const osg::Vec3d & p1		向量终点
		 *  @prarm	 const std::vector<osg::Vec3d> & polygons	边界顶点
		 *
		 *  @return   double
		 */
		double getMaxProjection(const osg::Vec3d& p0,
								const osg::Vec3d& p1,
								const std::vector<osg::Vec3d>& polygons);


		osg::BoundingBox getPolygonBound(const std::vector<osg::Vec3d>& polygon);
	}
}


#endif // GEOMETRY2D_OP_H_