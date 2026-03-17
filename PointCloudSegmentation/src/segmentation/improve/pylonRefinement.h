/////////////////////////////////////////////////////////////////////
// 文件名称：pylonRefinement.h
// 功能描述：铁塔范围内部点云分类细化
// 创建标识：吕伟	2022/9/26
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef PYLON_REFINEMENT_H_
#define PYLON_REFINEMENT_H_

#include <src/segmentation/powerlineCurve.h>

namespace d3s {
	namespace pcs {

		class Grid2D;
		class Grid3D;

		struct PylonRefinementOptions
		{
			bool curve_fit_fast_clustering_enable; // 拟合曲线时是否启用快速聚类
			int cluster_min_points;				   // 聚类簇最小点数

			double cluster_radius; // 电力线聚类距离阈值
			double connect_radius; // 导线连接点距离阈值
			double part_step;
			double part_width;
			double curve_expand;

			double max_curve_dist; // 距离曲线的最近阈值
		};

		class PylonRefinement
		{
		public:
			typedef std::vector<osg::Vec3d> Positions;
			typedef std::vector<osg::Vec3d> Vectors;
			typedef std::vector<std::vector<osg::Vec3d>> Polygons;


			PylonRefinement(const PylonRefinementOptions& options,
							PointCloudViewPtr input,
							const std::vector<int>& indices);

			~PylonRefinement();

			/**
			 *  @brief    获得导线连接点位置
			 *
			 *  @return  PylonRefinement::Connections
			 */
			PylonConnections GetConnections() const;

			/**
			 *  @brief    设置铁塔位置
			 *
			 *  @prarm	 const Positions & pylonPositions
			 *
			 *  @return   void
			 */
			void SetPylonPositions(const Positions& pylonPositions);

			/**
			 *  @brief    设置铁塔边界
			 *
			 *  @prarm	 const Polygons & pylonPolygons
			 *
			 *  @return   void
			 */
			void SetPylonPolygons(const Polygons& pylonPolygons);

			/**
			 *  @brief    设置杆塔横担方向
			 *
			 *  @param    const Vectors & pylonSides
			 *
			 *  @return   void
			 */
			void SetPylonSides(const Vectors& pylonSides);

			/**
			 *  @brief    执行细化分类算法
			 *
			 *  @return   void
			 */
			void Refine();

		private:
			/**
			 *  @brief    纠正铁塔在档内的电力线部分
			 *
			 *  @param    const osg::Vec3d& center					铁塔中心
			 *  @param    const PowerlineSpan & span				线路的一档
			 *  @param    const std::vector<int> & indices			铁塔点云
			 *
			 *  @return   void
			 */
			std::vector<int> CorrectPylonInSpan(const osg::Vec3d& center,
												const PowerlineSpan& span,
												const std::vector<int>& indices);

			/**
			 *  @brief    获得杆塔前后局部范围内的电力线
			 *
			 *  @param    const Grid2D & grid					点云格网数据
			 *  @param    const osg::Vec3d & p0					范围起点
			 *  @param    const osg::Vec3d & p1					范围终点
			 *  @param    const std::vector<osg::Vec3d> & polygon0	起点杆塔边界
			 *  @param    const std::vector<osg::Vec3d> & polygon1	终点杆塔边界
			 *  @param    double step							长度
			 *  @param    double width							宽度
			 *
			 *  @return   std::vector<int>
			 */
			std::vector<int> GetPowerlinesInSpan(const Grid2D& grid,
												 int i,
												 int j,
												 double step,
												 double width);

			std::vector<int> GetPowerlinesOutOfSpan(const Grid2D& grid,
													int i,
													int j,
													double step,
													double width);

			/**
			 *  @brief    计算近似档数据
			 *
			 *  @param    const std::vector<int> & indices		点云索引
			 *
			 *  @return   d3s::pcs::PowerlineSpan
			 */
			PowerlineSpan ComputeApproxSpan(const std::vector<int>& indices,
											const osg::Vec3d& pos,
											const osg::Vec3d& dir,
											double maxexpand);


			/**
			 *  @brief    计算杆塔挂点位置
			 *
			 *  @param    const PowerlineSpan & forward			前视档
			 *  @param    const PowerlineSpan & backward		后视档
			 *  @param    const std::vector<int> & pylon		铁塔点云
			 *  @param    PylonConnection & connection			铁塔挂点
			 *
			 *  @return   void
			 */
			void ComputePylonConnections(const PowerlineSpan& forward,
										 const PowerlineSpan& backward,
										 const Grid3D& voxels,
										 PylonConnection& connection);

			/**
			 *  @brief    合并距离过于接近的连接点
			 *
			 *  @param    double tolerance									距离阈值
			 *  @param    PylonConnection & connection						合并后的连接点集合
			 *
			 *  @return   void
			 */
			void MergeConnectionNearly(double tolerance, PylonConnection& connection);

		private:
			PylonRefinementOptions _options; // 杆塔点云细化参数
			PointCloudViewPtr _input;		 // 输入点云数据
			std::vector<int> _indices;		 // 处理点云的索引列表

			Positions _pylonPositions;
			Polygons _pylonPolygons;
			Vectors _pylonSides;

			PylonConnections _pylonConnections;
		};
	}
}

#endif // PYLON_REFINEMENT_H_