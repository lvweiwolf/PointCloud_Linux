//////////////////////////////////////////////////////////////////////
// 文件名称：vegetationExtraction.h
// 功能描述：点云植被提取
// 创建标识：吕伟	2022/5/9
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once

#include "../utils/threading.h"
#include "../core/pointTypes.h"
#include <memory>
#include <share_ptr.h>
namespace d3s {
	namespace pcs {

		class GridCell;
		class Grid2D;
		class IValueBuffer;

		struct EnviromentClassifyOptions
		{
			// 降噪参数
			// -----------
			bool denoise;			   // 是否降噪
			double denoise_voxel_size; // 降噪体素大小
			int denoise_min_pts;	   // 降噪体素内最小点数

			// 植被
			// -----------
			double cell_size;				   // 网格单元大小
			double min_vegetation_height;	   // 植被最低高度
			double min_high_vegetation_height; // 高植被最低高度

			// 建筑物
			// -----------
			double min_building_area;		   // 建筑物最小面积
			double max_building_area;		   // 建筑物最大面积
			double min_building_height;		   // 建筑物最小高度
			double euclidean_cluster_distance; // 欧式聚类阈值
			double planar_ransac_distance;	   // 平面分割的距离阈值
			double planar_inlier_pts_ratio;	   // 平面拟合点占总点数比例阈值
			int euclidean_cluster_min_pts;	   // 单个聚类最少点数

			// 道路
			// -----------
			double road_cell_size;			 // 格网单元大小
			double road_height_threshold;	 // 高程差阈值
			double road_slope_threshold;	 // 坡度高差阈值
			double road_min_area;			 // 最小面积阈值
			double road_max_area;			 // 最大面积
			double road_shape_threshold;	 // 形状系数阈值
			double road_linearity_threshold; // 线性系数阈值


			// 打印参数配置
			void Print();
		};

		// 植被分类
		class EnviromentClassify : public Thread
		{
		public:
			EnviromentClassify(const EnviromentClassifyOptions& options, PointCloudViewPtr input);

			virtual ~EnviromentClassify();

			/**
			 *  @brief    设置点云索引
			 *
			 *  @param    const std::vector<int> & indices			点云索引
			 *
			 *  @return   void
			 */
			void SetIndices(const std::vector<int>& indices);

			/**
			 *  @brief    设置道路矢量数据
			 *
			 *  @param    d3s::share_ptr<IValueBuffer> roadVectorize
			 *
			 *  @return   void
			 */
			void SetRoadVectorize(d3s::share_ptr<IValueBuffer> roadVectorize);

			/**
			 *  @brief    获得划分网格单元
			 *
			 *  @return   std::vector<d3s::pcs::GridCell>
			 */
			Grid2D GetGrid() const;

		private:
			virtual void Run() override;

			/**
			 *  @brief    植被候选单元分类
			 *
			 *  @return   void
			 */
			void VegetationCandidateClassify();


			EnviromentClassifyOptions _options; // 环境分类参数
			PointCloudViewPtr _input;			// 输入待分类点云数据
			std::vector<int> _indices;			// 指定的待处理点云索引

			std::unique_ptr<Grid2D> _grid; // 点云数据格网划分结果
			
			d3s::share_ptr<IValueBuffer> _roadVectorize; // 道路矢量数据
		};
	}
}