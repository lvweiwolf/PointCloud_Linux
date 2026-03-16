//////////////////////////////////////////////////////////////////////
// 文件名称：wireClassifier.h
// 功能描述：配电线路导线分类器
// 创建标识：吕伟	2023/8/23
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef WIRE_CLASSIFIER_H_
#define WIRE_CLASSIFIER_H_

#include <src/segmentation/powerlineCurve.h>

namespace d3s {
	namespace pcs {

		class Grid2D;

		struct Trunk;

		struct WireClassifyOptions
		{
			double slice_step;		// 沿档方向切分的距离
			double slice_thickness; // 档垂直方向的宽度

			double cluster_radius; // 电力线聚类距离阈值
			double cluster_th;

			int cluster_min_points;				   // 聚类簇最小点数
			int curve_min_points;				   // 曲线拟合聚类最小点数
			double curve_expand_length;			   // 曲线延长线长度
			bool curve_fit_fast_clustering_enable; // 拟合曲线时是否启用快速聚类


			double linearity_radius = 0.5;	// 计算点云线性特征半径
			double linearity_threshold = 0.9; // 计算点云线性特征阈值
		};

		class WireClassifier
		{
		public:
			typedef std::vector<osg::Vec3d> Positions;
			typedef std::vector<osg::Vec3d> Vectors;
			typedef std::vector<std::vector<osg::Vec3d>> Polygons;

			WireClassifier(const WireClassifyOptions& options,
						   PointCloudViewPtr input,
						   const std::vector<int>& indices);

			~WireClassifier();

			void SetPolePositions(const Positions& polePositions);

			void SetPolePolygons(const Polygons& polePolygons);

			void SetPoleSides(const Vectors& poleSides);

			std::vector<PowerlineSpan> GetWireSpans() const { return _wireSpans; }

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

			SpanResult ExtractSpanResult(const Grid2D& grid,
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
			SpanResult ExtractOutOfSpanResult(const Grid2D& grid,
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



			std::vector<bool> ComputeLinearity(PointCloudViewPtr input,
											   const std::vector<int>& indices);

			/**
			 *  @brief    根据点云特征修复导线
			 *
			 *  @param    SpanResult & result				档内数据
			 *  @param    std::vector<bool> & linearity		点云线性特征
			 *
			 *  @return   void
			 */
			void ReclassifyWithLinearity(const std::vector<bool>& linearity, SpanResult& result);

			/**
			 *  @brief    根据电力线拟合曲线重新分类电线点云
			 *
			 *  @return   void
			 */
			PowerlineSpan ReclassifyWithCurve(SpanResult& result, int curveMinPts);

			
			/**
			 *  @brief    档内未分类点基于线性特征修复导线
			 *
			 *  @param    const std::vector<bool> & linearity	点云线性特征
			 *  @param    PointCloudViewPtr input				点云数据
			 *  @param    int curveMinPts						导线聚类最少点数
			 *  @param    std::vector<int> & indices			档内点云索引
			 *  @param    std::vector<int> & curveIndices		修复导线点云索引
			 *
			 *  @return   void
			 */
			void ReclassifyWithLinearityAndCurve(const std::vector<bool>& linearity,
												 PointCloudViewPtr input,
												 int curveMinPts,
												 std::vector<int>& indices,
												 std::vector<int>& curveIndices);



			void ReclassifyPoleFromUnclassified(std::vector<Trunk>& trunks,
												std::vector<std::vector<int>>& trunkClusters,
												const std::vector<int>& indices,
												std::vector<std::vector<int>>& poleIndicesList);

			/**
			 *  @brief    获得档内未分类点云
			 *
			 *  @param    SpanResult & result
			 *
			 *  @return   std::vector<int>
			 */
			std::vector<int> GetUnclassified(SpanResult& result);



		private:
			WireClassifyOptions _options; // 导线点云提取参数
			PointCloudViewPtr _input;	  // 输入点云数据
			std::vector<int> _indices;	  // 处理点云的索引列表

			Positions _polePositions; // 电杆坐标
			Polygons _polePolygons;	  // 电杆边界
			Vectors _poleSides;		  // 电杆横担方向

			PowerlineSpan _beforeSpan;			   // 前方档
			PowerlineSpan _afterSpan;			   // 后方档
			std::vector<PowerlineSpan> _wireSpans; // 导线线档内信息
		};
	}
}

#endif // WIRE_CLASSIFIER_H_
