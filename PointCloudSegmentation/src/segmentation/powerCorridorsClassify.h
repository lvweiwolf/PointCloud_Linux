//////////////////////////////////////////////////////////////////////
// 文件名称：powerCorridorsClassify.h
// 功能描述：电力线分类
// 创建标识：吕伟	2022/4/11
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once

#include "../utils/threading.h"
#include "../core/pointTypes.h"
#include <osg/Vec3d>
namespace d3s {
	namespace pcs {

		class GridCell;
		class Grid2D;

		// 电力线分类参数设置
		struct PowerCorridorsClassifyOptions
		{
			// 遗留算法参数
			// --------------
			int max_span_error = 1;			// 档中最大错误尝试次数
			double min_span_angle_theta;	// 档方向与导线方向最小夹角阈值
			double min_body_growing_height; // 塔体生长最小高度
			double max_body_growing_height; // 塔体生长最大高度
			double body_growing_step;		// 塔体生长步长
			double body_growing_resolition; // 塔体生长掩码分辨率
			double max_curve_dist;			// 距离曲线的最近阈值

			bool distribution = false;

			// 改进算法相关参数
			// --------------
			double location_cellsize = 1.5;
			double location_zstep = 1.0;
			double location_th1 = 15.0;
			double location_th2 = 4.0;
			double location_tv = 0.8;
			double location_tc = 0.2;
			double location_th = 0.15;
			double location_outlier_coeff = 1.5;
			double location_linearity = 0.9;
			double location_min_proj = 300.0;
			double location_max_span = 1000.0;
			double location_corridor_width = 60.0;
			int location_neighbors = 2;

			double pylon_head_length = 10.0;
			double pylon_d1_scale = 0.5;
			double pylon_d2_scale = 1.0;
			double pylon_terminal_scale = 1.0;
			double pylon_corner_scale = 1.0;

			double powerline_slice_step = 10.0;
			double powerline_slice_thickness = 10.0;
			double powerline_cluster_th = 4.0;
			double powerline_cluster_radius = 0.4;
			int powerline_cluster_min_points = 20;
			int powerline_curve_min_points = 300;
			double powerline_curve_expand_length = 20.0;
			bool powerline_curve_fit_fast = true;


			double refine_cluster_radius = 0.4;
			double refine_connect_radius = 1.0;
			int refine_cluster_min_points = 20;
			bool refine_curve_fit_fast = false;

			bool attachment_enable = false;
			double attachment_voxel_size = 1.0;
			double attachment_search_radius = 1.0;
			double attachment_x_length = 2.0;
			double attachment_y_length = 2.0;
			double attachment_z_length = 2.0;
			double attachment_x_fillrate = 0.7;
			double attachment_y_fillrate = 0.7;
			double attachment_xy_fillrate = 0.3;
			double attachment_xz_fillrate = 0.3;
			double attachment_leadwire_x_fr = 0.5;
			double attachment_leadwire_x_len = 2.0;
			double attachment_suspension_thr = 0.9;
			double attachment_strain_thr = 0.06;
			std::string attachment_suspension_method = "verticality";
			std::string attachment_strain_method = "sphericity";

			void Print() const;
		};

		// 电力分类线程
		class PowerCorridorsClassify : public Thread
		{

		public:
			PowerCorridorsClassify(const PowerCorridorsClassifyOptions& options,
								   PointCloudViewPtr input);

			virtual ~PowerCorridorsClassify();

			/**
			 *  @brief    设置点云索引
			 *
			 *  @param    const std::vector<int> & indices			点云索引
			 *
			 *  @return   void
			 */
			void SetIndices(const std::vector<int>& indices);

			/**
			 *  @brief    设置用户指定杆塔坐标
			 *
			 *  @param    const std::vector<osg::Vec3d> & positions
			 *
			 *  @return   void
			 */
			void SetPylonPositions(const std::vector<osg::Vec3d>& positions);

			/**
			 *  @brief    获得划分网格单元
			 *
			 *  @return   std::vector<d3s::pcs::GridCell>
			 */
			Grid2D GetGrid() const;

		private:
			virtual void Run() override;


			PowerCorridorsClassifyOptions _options; // 输电线路走廊分类参数
			PointCloudViewPtr _input;				// 输入点云数据
			std::vector<int> _indices;				// 待处理点云索引

			std::unique_ptr<Grid2D> _grid; // 点云网格结果
			std::vector<osg::Vec3d> _positions;
		};

	}
}
