/////////////////////////////////////////////////////////////////////
// 文件名称：powerlineClassifier.h
// 功能描述：
// 创建标识：吕伟	2022/9/19
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef POWERLINE_CLASSIFIER_H_
#define POWERLINE_CLASSIFIER_H_

#include <src/segmentation/powerlineCurve.h>

namespace d3s {
	namespace pcs {

		class Grid2D;

		struct PowerlineClassifyOptions
		{
			double slice_step;		// 沿档方向切分的距离
			double slice_thickness; // 档垂直方向的宽度

			double cluster_radius; // 电力线聚类距离阈值
			double cluster_th;

			int cluster_min_points;				   // 聚类簇最小点数
			int curve_min_points;				   // 曲线拟合聚类最小点数
			double curve_expand_length;			   // 曲线延长线长度
			bool curve_fit_fast_clustering_enable; // 拟合曲线时是否启用快速聚类
		};

		class PowerlineClassifier
		{
		public:
			typedef std::vector<osg::Vec3d> Positions;
			typedef std::vector<osg::Vec3d> Vectors;
			typedef std::vector<std::vector<osg::Vec3d>> Polygons;

			PowerlineClassifier(const PowerlineClassifyOptions& options,
								PointCloudViewPtr input,
								const std::vector<int>& indices);

			~PowerlineClassifier();

			void SetPylonPositions(const Positions& pylonPositions);

			void SetPylonPolygons(const Polygons& pylonPolygons);

			void SetPylonSides(const Vectors& pylonSides);

			std::vector<PowerlineSpan> GetPowerlineSpans() const { return _powerlineSpans; }

			PowerlineSpan GetBeforeSpan() const { return _beforeSpan; }

			PowerlineSpan GetAfterSpan() const { return _afterSpan; }

			void Segment();

		private:
			struct SpanSpatial
			{
				int id;

				osg::Vec3d p0;
				osg::Vec3d p1;

				osg::Vec3d side0;
				osg::Vec3d side1;

				std::vector<osg::Vec3d> polygon0;
				std::vector<osg::Vec3d> polygon1;
			};

			/**
			 *  @brief    提取档内点云
			 *
			 *  @prarm	 const SpanSpatial & spatial	档空间信息
			 *  @prarm	 double step					档方向切分步长
			 *  @prarm	 double width					档垂直方向上的宽度
			 *
			 *  @return   SpanResult
			 */
			SpanResult ExtractPointsInSpan(const Grid2D& grid,
										   const SpanSpatial& spatial,
										   double step,
										   double width);

			/**
			 *  @brief    提取档边界范围之外的点
			 *
			 *  @param    const osg::Vec3d & p			杆塔坐标
			 *  @param    const osg::Vec3d & v			方向
			 *  @param    const osg::Vec3d & side		横担方向
			 *  @param    const std::vector<osg::Vec3d> & polygon	杆塔边界
			 *  @param    double step					切分步长
			 *  @param    double width					切分宽度
			 *  @param    int id						标号
			 *
			 *  @return   d3s::pcs::SpanResult
			 */
			SpanResult ExtractPointsOutOfSpan(const Grid2D& grid,
											  const osg::Vec3d& p,
											  const osg::Vec3d& v,
											  const osg::Vec3d& side,
											  const std::vector<osg::Vec3d>& polygon,
											  double step,
											  double width,
											  int id);

			/**
			 *  @brief    分类档内的电力线
			 *
			 *  @param    SpanResult & result
			 *
			 *  @return   void
			 */
			void ClassifyInSpan(SpanResult& result);


			/**
			 *  @brief    根据电力线拟合曲线重新分类电线点云
			 *
			 *  @return   void
			 */
			PowerlineSpan ReclassifyWithCurve(SpanResult& result, int curveMinPts);

			/**
			 *  @brief    排除与档方向不同向的电力线
			 *
			 *  @param    SpanResult & result
			 *
			 *  @return   void
			 */
			void ExcludeDifferentDirection(SpanResult& result);

			/**
			 *  @brief    获得所有档的边界
			 *
			 *  @param    const PylonPolygons & polygons	档信息列表
			 *
			 *  @return   osg::BoundingBox
			 */
			osg::BoundingBox GetBound(const Polygons& polygons) const;

		private:
			PowerlineClassifyOptions _options; // 铁塔点云提取参数
			PointCloudViewPtr _input;		   // 输入点云数据
			std::vector<int> _indices;		   // 处理点云的索引列表

			Positions _pylonPositions; // 杆塔坐标
			Polygons _pylonPolygons;   // 杆塔边界
			Vectors _pylonSides;	   // 杆塔横担方向

			PowerlineSpan _beforeSpan;					// 前方档
			PowerlineSpan _afterSpan;					// 后方档
			std::vector<PowerlineSpan> _powerlineSpans; // 电力线档内信息
		};

	}
}

#endif // POWERLINE_CLASSIFIER_H_