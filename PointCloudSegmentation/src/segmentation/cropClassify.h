//////////////////////////////////////////////////////////////////////
// 文件名称：cropClassify.h
// 功能描述：青苗赔偿农作物分类
// 创建标识：吕伟	2023/1/28
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once


#include "../utils/threading.h"
#include "../core/pointTypes.h"
#include <vector>
#include <functional>
namespace d3s {
	namespace pcs {

		class GridCell;
		class Grid2D;

		struct CropClassifyOptions
		{
			double min_height;		// 过滤点云最小高度
			double max_height;		// 过滤点云最大高度
			double min_area;		// 农作物区域最小面积
			double min_forest_area; // 林地区域最小面积
			double pred_threshold;	// 预测概率阈值

			std::string cache_dir;	// 缓存目录
			std::string ncnn_param; // UNet的 NCNN 网络结构文件
			std::string ncnn_model; // UNet的 NCNN 网络权重文件

			std::vector<std::string> dom_files;

			// 打印参数配置
			void Print();
		};

		class CropClassify : public Thread
		{
		public:
			CropClassify(const CropClassifyOptions& options,
						 PointCloudViewPtr input,
						 const std::vector<int>& indices,
						 std::function<void(double)> progress_func);

			virtual ~CropClassify();

		private:
			virtual void Run() override;

			/**
			 *  @brief    林地区域聚类
			 *
			 *  @param    double cellsize							点云划分格网单元大小
			 *  @param    double min_forest_area					林区最小面积(否则为散树)
			 *  @param    int min_high_vegs							格网单元聚类最少高植被点数
			 *
			 *  @return   void
			 */
			void ForestRegionClustering(double cellsize, double min_forest_area, int min_high_vegs);

			/**
			 *  @brief    农作与深度学习推理预测
			 *
			 *  @return   bool
			 */
			bool CropInference();


			CropClassifyOptions _options; // 作物分类参数
			PointCloudViewPtr _input;	  // 输入待分类点云数据
			std::vector<int> _indices;	  // 指定的待处理点云索引

			std::vector<int> _crops;		  // 作物点云索引
			std::vector<int> _dense_forests;  // 密集林区点云索引
			std::vector<int> _sparse_forests; // 稀疏林区点云索引

			std::function<void(double)> _progress_func;
		};
	}
}
