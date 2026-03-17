//////////////////////////////////////////////////////////////////////
// 文件名称：buildingExtraction.h
// 功能描述：建筑物、水域检测提取
// 创建标识：吕伟	2022/5/16
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef BUILDING_EXTRACTION_H_
#define BUILDING_EXTRACTION_H_

#include <src/core/pointTypes.h>

namespace cv {
	class Mat;
}

namespace d3s {
	namespace pcs {

		class GridCell;
		class Grid2D;

		struct BuildingExtractionOptions
		{
			double min_building_area; // 建筑物最小面积

			double max_building_area; // 建筑物最大面积

			double min_building_height; // 建筑物最小高度

			double euclidean_cluster_distance; // 欧式聚类阈值

			double planar_ransac_distance; // 平面分割的距离阈值

			double planar_inlier_pts_ratio; // 平面拟合点占总点数比例阈值

			int euclidean_cluster_min_pts; // 单个聚类最少点数

			int morph_kernel_size = 3; // 形态学处理卷积核大小
		};

		// BuildingExtraction
		//////////////////////////////////////////////////////////////////////////

		class BuildingExtraction
		{
		public:
			BuildingExtraction(const BuildingExtractionOptions& options, PointCloudViewPtr input);

			~BuildingExtraction();


			/**
			 *  @brief    建筑物提取
			 *
			 *  @return   void
			 */
			void Extract();

		private:
			BuildingExtractionOptions _options; // 建筑物提取分类参数
			PointCloudViewPtr _input;			// 输出待分类的点云数据
		};
	}
}

#endif // BUILDING_EXTRACTION_H_