//////////////////////////////////////////////////////////////////////
// 文件名称：dangerSegmentation.h
// 功能描述：危险区域识别
// 创建标识：吕伟	2023/9/13
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once

#include "../utils/threading.h"

#include "../plot/plotHandle.h"
#include <memory>
#include <ncnn/platform.h>

namespace d3s {
	namespace pcs {

		struct SteepSlopeOptions
		{
			double resolution = 0.5;
			double degree_threshold = 35.0;
			double area_threshold = 150.0;

			// 打印参数
			void Print();
		};

		class SteepSlopeSegmentation : public Thread
		{
		public:
			SteepSlopeSegmentation(const SteepSlopeOptions& options,
								   PointCloudViewPtr input,
								   const std::vector<int>& indices);

			virtual ~SteepSlopeSegmentation();

		private:
			virtual void Run() override;


			SteepSlopeOptions _options; // 树种识别参数
			PointCloudViewPtr _input;	// 输入点云数据
			std::vector<int> _indices;	// 待处理点云索引
		};


		struct ExcavationOptions
		{
			double resolution = 0.5;
			double degree_threshold = 35.0;
			double hag_threshold = 2.0;
			double smooth_threshold = 0.1;
			double area_threshold = 200.0;

			// 打印参数
			void Print();
		};

		class ExcavationSegmentation : public Thread
		{
		public:
			ExcavationSegmentation(const ExcavationOptions& options,
								   PointCloudViewPtr input,
								   const std::vector<int>& indices);

			virtual ~ExcavationSegmentation();

		private:
			virtual void Run() override;


			ExcavationOptions _options; // 树种识别参数
			PointCloudViewPtr _input;	// 输入点云数据
			std::vector<int> _indices;	// 待处理点云索引
		};

		class GridCell;
		class Grid2D;
		struct DOMDataset;

		struct DomSegmentOptions
		{
			std::string dom_file_path;
			std::string road_segment_param;
			std::string road_segment_bin;
			std::string building_segment_param;
			std::string building_segment_bin;
			std::string water_segment_param;
			std::string water_segment_bin;
			
			// 打印参数
			void Print();
		};

		class DomSegmentation : public Thread
		{
		public:
			DomSegmentation(const DomSegmentOptions& options,
							PointCloudViewPtr input,
							const std::vector<int>& indices);

			virtual ~DomSegmentation();

		private:
			virtual void Run() override;


			void RoadSegmentation(Grid2D& grid, const std::shared_ptr<DOMDataset>& ds);

			void BuildingSegmentation(Grid2D& grid, const std::shared_ptr<DOMDataset>& ds);

			void WaterSegmentation(Grid2D& grid, const std::shared_ptr<DOMDataset>& ds);

			void RunBinarySegmentation(Grid2D& grid,
									   const std::shared_ptr<DOMDataset>& ds,
									   const std::string& ncnn_param_path,
									   const std::string& ncnn_model_path,
									   double minArea,
									   double scoreThr,
									   std::vector<int>& outindices);

			DomSegmentOptions _options; // 正射影像识别参数
			PointCloudViewPtr _input;	// 输入点云数据
			std::vector<int> _indices;	// 待处理点云索引
		};

	}
}