//////////////////////////////////////////////////////////////////////
// 文件名称：powerlineCurve.h
// 功能描述：电力线档内悬链线信息
// 创建标识：吕伟	2022/9/21
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once
#include <osg/Vec3d>
#include <osg/Matrix>
#include <vector>
#include <memory>
#include <float.h>
#include "../core/pointTypes.h"
namespace d3s {
	namespace pcs {

		// SpanResult
		//////////////////////////////////////////////////////////////////////////
		struct SpanResult
		{
			int id;
			osg::Vec3d direction;

			std::vector<std::vector<int>> sliceInidces;
			std::vector<std::vector<osg::Vec3d>> slicePolygons; // 档边界在档方向上的切分

			std::vector<int> powerlines;	 // 档内电力线点云索引
			std::vector<int> otherlines;	 // 档内其他线路点云索引
			std::vector<int> powerlinesFail; // 未被成功分类为电力线的点云索引
		};

		typedef std::vector<SpanResult> SpanResultList;

		// PowerlineCurve
		//////////////////////////////////////////////////////////////////////////
		struct PowerlineCurve
		{
			PowerlineCurve() : xmean(0.0), ymin(-DBL_MAX), ymax(DBL_MAX), a(0.0), b(0.0), c(0.0) {}

			PowerlineCurve(const osg::Matrix& mat,
						   const osg::Vec3d& o,
						   double x,
						   double min,
						   double max,
						   double ca,
						   double cb,
						   double cc)
				: rotate(mat), origin(o), xmean(x), ymin(min), ymax(max), a(ca), b(cb), c(cc)
			{
			}

			PowerlineCurve(const PowerlineCurve& rhs)
				: rotate(rhs.rotate),
				  origin(rhs.origin),
				  xmean(rhs.xmean),
				  ymin(rhs.ymin),
				  ymax(rhs.ymax),
				  a(rhs.a),
				  b(rhs.b),
				  c(rhs.c)
			{
			}

			double DistanceTo(const osg::Vec3d& point) const;

			double NolimitDistanceTo(const osg::Vec3d& point) const;

			osg::Vec3d GetMin() const;
		
			osg::Vec3d GetMax() const;

			osg::Matrix rotate; // 导线方向对齐到 Y 轴的旋转矩阵
			osg::Vec3d origin;	// 曲线方程的坐标原点
			double xmean;		// 曲线所在平面的 X 轴偏移
			double ymin;		// 曲线所在平面 Y 轴最小值
			double ymax;		// 曲线所在平面 Y 轴最大值
			double a, b, c;
		};

		typedef std::shared_ptr<PowerlineCurve> PowerlineCurvePtr;
		typedef std::vector<PowerlineCurvePtr> PowerlineSpan;

		// 导线连接点信息
		struct ConnectionInstance
		{
			ConnectionInstance(PowerlineCurvePtr forwardCurve,
							   PowerlineCurvePtr backwardCurve,
							   const osg::Vec3d& cp)
				: forwardSpanCurve(forwardCurve), backwardSpanCurve(backwardCurve), connectPoint(cp)
			{
			}

			PowerlineCurvePtr forwardSpanCurve;	 // 挂点连接前视档导线的拟合曲线
			PowerlineCurvePtr backwardSpanCurve; // 挂点连接后视档导线的拟合曲线
			osg::Vec3d connectPoint;			 // 挂点坐标
		};

		typedef std::vector<osg::Vec3d> PylonConnection;
		typedef std::vector<PylonConnection> PylonConnections;

		/**
		 *  @brief    电力线曲线拟合
		 *
		 *  @param    PointCloudViewPtr  input				点云数据
		 *  @param    const std::vector<int> & indices		单股导线点云缩影
		 *  @param    osg::Vec4 color						图形颜色
		 *
		 *  @return   std::vector<osg::Vec3d>				返回曲线顶点坐标
		 */
		PowerlineCurvePtr PowerlineCurveFitting(PointCloudViewPtr input,
												const std::vector<int>& indices,
												double expandLength,
												osg::Vec4 color, bool debug = false);

		PowerlineCurvePtr PowerlineCurveFitting(PointCloudViewPtr input,
												const std::vector<int>& indices,
												const osg::Vec3d& pos,
												const osg::Vec3d& dir,
												double expandLength,
												osg::Vec4 color,
												bool debug = false);
	}
}
