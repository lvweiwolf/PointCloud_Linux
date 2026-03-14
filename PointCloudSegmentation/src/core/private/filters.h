/////////////////////////////////////////////////////////////////////
// 文件名称：filters.h
// 功能描述：点云滤波相关接口
// 创建标识：吕伟	2022/6/14
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once
#include "../pointTypes.h"
namespace d3s {
	namespace pcs {

		/**
		 *  @brief    使用渐进形态学滤波提取地面点
		 *
		 *  @prarm	 const PointCloudView<PointPCLH> & pcv			点云数据
		 *  @prarm	 std::vector<int>& ground						提取地面点索引
		 *  @prarm	 int max_window_size							最大窗口尺寸
		 *  @prarm	 float slope									坡度阈值
		 *  @prarm	 float max_dist									最大距离阈值
		 *  @prarm	 float init_dist								初始距离
		 *  @prarm	 float cellsize									划分单元大小
		 *  @prarm	 float base										窗口尺寸基数
		 *  @prarm	 bool exponential								窗口尺寸是否呈指数增长
		 *
		 *  @return  bool
		 */
		bool groundSegmentation_ApproximatePMF(PointCloudView<PointPCLH>& pcv,
											   std::vector<int>& ground,
											   int max_window_size,
											   float slope,
											   float max_dist,
											   float init_dist,
											   float cellsize,
											   float base,
											   bool exponential = true);

		bool groundSegmentation_ApproximatePMF(PointCloudView<PointPCLH>& pcv,
											   const std::vector<int>& indices,
											   std::vector<int>& ground,
											   int max_window_size,
											   float slope,
											   float max_dist,
											   float init_dist,
											   float cellsize,
											   float base,
											   bool exponential = true);

		/**
		 *  @brief    点云随机采样
		 *
		 *  @param    const PointCloudView<PointPCLH> & input
		 *  @param    size_t numberOfPoints
		 *  @param    PointCloudView<PointPCLH> & output
		 *
		 *  @return   void
		 */
		void cloudRandomSampling(const PointCloudView<PointPCLH>& input,
								 size_t numberOfPoints,
								 PointCloudView<PointPCLH>& output);

		/**
		 *  @brief    点云索引随机采样
		 *
		 *  @param    const std::vector<int> & indices		点云索引
		 *  @param    size_t numSample						采样数量
		 *
		 *  @return   std::vector<int>
		 */
		std::vector<int> randomSampling(const std::vector<int>& indices, size_t numSample);


		std::vector<int> farthestPointSampling(const PointCloudView<PointPCLH>& input,
											   const std::vector<int>& indices,
											   size_t numSample);
	}
}