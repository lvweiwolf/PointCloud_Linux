#include <src/segmentation/dangerSegmentation.h>
#include <src/segmentation/gridCell.h>
#include <src/core/api.h>
#include <src/core/private/rasterProcess.h>
#include <src/core/private/gdalProcess.h>
#include <src/core/private/cloudProcess.h>
#include <src/core/private/statistics.h>
#include <src/core/deeplearning/inference.h>
#include <src/utils/logging.h>
#include <src/utils/misc.h>
#include <src/io/rasterWriter.h>

#include <numeric>
#include <memory>
#include <string>


// #define WRITE_RASTER_DEBUG
// #define WRITE_MASK_IMAGE

extern std::string DebugDirectory;

namespace d3s {
	namespace pcs {
		// SteepSlopeOptions
		//////////////////////////////////////////////////////////////////////////
		void SteepSlopeOptions::Print()
		{
			PCS_INFO("steep_slope_resolution: %lf", resolution);
			PCS_INFO("steep_slope_degree_threshold: %lf", degree_threshold);
			PCS_INFO("steep_slope_area_threshold: %lf", area_threshold);
		}

		// SteepSlopeSegmentation
		//////////////////////////////////////////////////////////////////////////
		SteepSlopeSegmentation::SteepSlopeSegmentation(const SteepSlopeOptions& options,
													   PointCloudViewPtr input,
													   const std::vector<int>& indices)
			: _options(options), _input(input), _indices(indices)
		{
		}

		SteepSlopeSegmentation::~SteepSlopeSegmentation() {}

		void SteepSlopeSegmentation::Run()
		{
			// 兼容空索引
			if (_indices.empty())
			{
				_indices.resize(_input->size());
				std::iota(_indices.begin(), _indices.end(), 0);
			}

			// 未能获取到可以处理的点云数据索引
			if (_indices.empty())
			{
				PCS_WARN("[SteepSlopeSegmentation] 待处理点云索引为空!");
				return;
			}

			double resolution = _options.resolution;
			double slopeThr = _options.degree_threshold;
			double areaThr = _options.area_threshold;

			osg::BoundingBox bbox;
			computeMinMax3D(*_input, _indices, bbox);

			std::vector<int> ground;
			getClassification(*_input, eGround, ground);

			// 生成掩码
			Rasterd mask;
			createBinaryMask(*_input, _indices, bbox, resolution, mask);

			// 生成地形
			Rasterd dem;
			createCloudRaster(*_input, ground, resolution, dem);

			// 计算离地高度
			if (!_input->normalized)
				normalizeByGround(dem, *_input);

			int width = mask.width();
			int height = mask.height();
			double nodata = std::numeric_limits<double>::quiet_NaN();

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (mask.at(c, r) == 0.0)
						dem.at(c, r) = nodata;
				}
			}

			// 计算坡度角
			Rasterd slope = computeSlope(dem);
			Rasterd grad = computeGrad(slope);

			width = slope.width();
			height = slope.height();

			Rasterd slopeMask(slope.extents(), 0.0);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (slope.at(c, r) > slopeThr && grad.at(c, r) <= 0.1)
						slopeMask.at(c, r) = 1.0;
				}
			}

			morphology(slopeMask, cv::MORPH_OPEN, cv::MORPH_RECT, 3, 3);
			morphology(slopeMask, cv::MORPH_CLOSE, cv::MORPH_RECT, 6, 6);

			slopeMask = filterWithArea(slopeMask, areaThr);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (slopeMask.at(c, r) == 0.0)
						slope.at(c, r) = nodata;
				}
			}

#ifdef WRITE_RASTER_DEBUG
			// 将光栅数据写入本地
			RasterWriter rasterWriter(R"(SlopeDegree.tiff)");
			rasterWriter.write(&slope, "");
#endif

			for (size_t i = 0; i < _input->size(); ++i)
			{
				const auto& p = _input->points[i];

				if (p.hag > 3.0)
					continue;

				double halfEdge = slopeMask.edgeLength() / 2.0;
				double edgeBit = slopeMask.edgeLength() * .000001;

				int xi = slopeMask.xCell(p.x + halfEdge - edgeBit);
				int yi = slopeMask.yCell(p.y + halfEdge - edgeBit);

				xi = clamp(xi, 0, slopeMask.width() - 1);
				yi = clamp(yi, 0, slopeMask.height() - 1);

				if (slopeMask.at(xi, yi) > 0.0)
					ColorManager::debugTags[i] = 0;
			}
		}


		// ExcavationOptions
		//////////////////////////////////////////////////////////////////////////
		void ExcavationOptions::Print()
		{
			PCS_INFO("excavation_resolution: %lf", resolution);
			PCS_INFO("excavation_degree_threshold: %lf", degree_threshold);
			PCS_INFO("excavation_hag_threshold: %lf", hag_threshold);
			PCS_INFO("excavation_area_threshold: %lf", area_threshold);
		}

		// ExcavationSegmentation
		//////////////////////////////////////////////////////////////////////////

		ExcavationSegmentation::ExcavationSegmentation(const ExcavationOptions& options,
													   PointCloudViewPtr input,
													   const std::vector<int>& indices)
			: _options(options), _input(input), _indices(indices)
		{
		}

		ExcavationSegmentation::~ExcavationSegmentation() {}

		void ExcavationSegmentation::Run()
		{
			// 兼容空索引
			if (_indices.empty())
			{
				_indices.resize(_input->size());
				std::iota(_indices.begin(), _indices.end(), 0);
			}

			// 未能获取到可以处理的点云数据索引
			if (_indices.empty())
			{
				PCS_WARN("[ExcavationSegmentation] 待处理点云索引为空!");
				return;
			}

			double resolution = _options.resolution;
			double slopeThr = _options.degree_threshold;
			double hagThr = _options.hag_threshold;
			double smoothThr = _options.smooth_threshold;
			double areaThr = _options.area_threshold;

			osg::BoundingBox bbox;
			computeMinMax3D(*_input, _indices, bbox);

			std::vector<int> ground;
			getClassification(*_input, eGround, ground);

			// 生成地形
			Rasterd dem;
			createCloudRaster(*_input, ground, resolution, dem);

#ifdef WRITE_RASTER_DEBUG
			RasterWriter demWriter(R"(DEM.tiff)");
			demWriter.write(&dem, "");
#endif

			// 计算离地高度
			if (!_input->normalized)
				normalizeByGround(dem, *_input);

			// 生成离地高度
			Rasterd heights;
			createHagRaster(*_input, _indices, bbox, resolution, heights);
			gaussianBlur(heights, 3, 3, 1);
			Rasterd smoothness = computeLaplacian(heights);

			int width = heights.width();
			int height = heights.height();
			double nodata = std::numeric_limits<double>::quiet_NaN();

			Rasterd slope = computeSlope(dem);

			Rasterd expose = dem;

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					double h = heights.at(c, r);

					if (std::isnan(h) || smoothness.at(c, r) > smoothThr || h > hagThr ||
						slope.at(c, r) < slopeThr)
					{
						expose.at(c, r) = nodata;
					}
				}
			}

			Rasterd mask(expose.extents(), 0.0);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (!std::isnan(expose.at(c, r)))
						mask.at(c, r) = 1.0;
				}
			}

#ifdef WRITE_RASTER_DEBUG
			RasterWriter writer1(R"(MASK1.tiff)");
			writer1.write(&mask, "");
#endif

			morphology(mask, cv::MORPH_OPEN, cv::MORPH_RECT, 3, 3);
			morphology(mask, cv::MORPH_CLOSE, cv::MORPH_RECT, 6, 6);

#ifdef WRITE_RASTER_DEBUG
			RasterWriter writer2(R"(MASK2.tiff)");
			writer2.write(&mask, "");
#endif

			mask = filterWithArea(mask, areaThr);

#ifdef WRITE_RASTER_DEBUG
			RasterWriter writer3(R"(MASK3.tiff)");
			writer3.write(&mask, "");
#endif

			for (size_t i = 0; i < _input->size(); ++i)
			{
				const auto& p = _input->points[i];

				if (p.hag > 3.0)
					continue;

				double halfEdge = mask.edgeLength() / 2.0;
				double edgeBit = mask.edgeLength() * .000001;

				int xi = mask.xCell(p.x + halfEdge - edgeBit);
				int yi = mask.yCell(p.y + halfEdge - edgeBit);

				xi = clamp(xi, 0, mask.width() - 1);
				yi = clamp(yi, 0, mask.height() - 1);

				if (mask.at(xi, yi) > 0.0)
					ColorManager::debugTags[i] = 1;
			}
		}

		// DomSegmentOptions
		//////////////////////////////////////////////////////////////////////////
		void DomSegmentOptions::Print()
		{
			PCS_INFO("dom_file_path: %s", dom_file_path.c_str());
			PCS_INFO("dom_road_segment_param: %s", road_segment_param.c_str());
			PCS_INFO("dom_road_segment_model: %s", road_segment_bin.c_str());
			PCS_INFO("dom_building_segment_param: %s", building_segment_param.c_str());
			PCS_INFO("dom_building_segment_model: %s", building_segment_bin.c_str());
			PCS_INFO("dom_water_segment_param: %s", water_segment_param.c_str());
			PCS_INFO("dom_water_segment_model: %s", water_segment_bin.c_str());
		}


		// DomSegmentation
		//////////////////////////////////////////////////////////////////////////
		DomSegmentation::DomSegmentation(const DomSegmentOptions& options,
										 PointCloudViewPtr input,
										 const std::vector<int>& indices)
			: _options(options), _input(input), _indices(indices)
		{
		}

		DomSegmentation::~DomSegmentation() {}

		void DomSegmentation::Run()
		{
			// 兼容空索引
			if (_indices.empty())
			{
				_indices.resize(_input->size());
				std::iota(_indices.begin(), _indices.end(), 0);
			}

			// 未能获取到可以处理的点云数据索引
			if (_indices.empty())
			{
				PCS_WARN("[DomSegmentation] 待处理点云索引为空!");
				return;
			}

			if (!ExistsFile(_options.dom_file_path))
			{
				PCS_WARN("[DomSegmentation] 正射影像文件不存在.");
				return;
			}

			double resolution = 0.5;
			double pixelsize = 512;
			double cellsize = (pixelsize - 1) * resolution;

			Grid2D grid;
			ComputeGridCells(cellsize, _input, _indices, grid);

			auto ds = createDOMDataset(_options.dom_file_path);

			// 道路识别提取
			RoadSegmentation(grid, ds);

			// 厂房、居民区识别提取
			BuildingSegmentation(grid, ds);

			// 湖泊、河流识别提取
			WaterSegmentation(grid, ds);

			closeDOMDataset(ds); // 关闭正射影像数据
		}

		void DomSegmentation::RoadSegmentation(Grid2D& grid, const std::shared_ptr<DOMDataset>& ds)
		{
			std::vector<int> indices;

			// 执行二分类分割
			RunBinarySegmentation(grid,
								  ds,
								  _options.road_segment_param,
								  _options.road_segment_bin,
								  100.0,
								  0.1,
								  indices);

			for (size_t i = 0; i < indices.size(); ++i)
				ColorManager::debugTags[indices[i]] = 2;
		}

		void DomSegmentation::BuildingSegmentation(Grid2D& grid,
												   const std::shared_ptr<DOMDataset>& ds)
		{
			std::vector<int> indices;

			// 执行二分类分割
			RunBinarySegmentation(grid,
								  ds,
								  _options.building_segment_param,
								  _options.building_segment_bin,
								  20.0,
								  0.3,
								  indices);

			for (size_t i = 0; i < indices.size(); ++i)
				ColorManager::debugTags[indices[i]] = 3;
		}

		void DomSegmentation::WaterSegmentation(Grid2D& grid, const std::shared_ptr<DOMDataset>& ds)
		{
			std::vector<int> indices;

			// 执行二分类分割
			RunBinarySegmentation(grid,
								  ds,
								  _options.water_segment_param,
								  _options.water_segment_bin,
								  100.0,
								  0.05,
								  indices);


			for (size_t i = 0; i < indices.size(); ++i)
				ColorManager::debugTags[indices[i]] = 4;
		}

		void DomSegmentation::RunBinarySegmentation(Grid2D& grid,
													const std::shared_ptr<DOMDataset>& ds,
													const std::string& ncnn_param_path,
													const std::string& ncnn_model_path,
													double minArea,
													double scoreThr,
													std::vector<int>& outindices)
		{
			if (!ds)
				return;

			if (!ExistsFile(ncnn_param_path) || !ExistsFile(ncnn_model_path))
			{
				PCS_ERROR("[DomSegmentation] 推理失败，模型文件不存在.");
				return;
			}

			const auto& offset = _input->offset_xyz;

#ifdef INFERENCE_USE_NCNN
			ncnn::Net ncnn_model;
#if NCNN_VULKAN
			int device_index = 0;
			if (select_gpu_device(device_index))
			{
				ncnn_model.opt.use_vulkan_compute = true;
				ncnn_model.set_vulkan_device(device_index);
			}

			if (device_index < 0)
			{
				PCS_ERROR("[DomSegmentation] 推理失败，未能检测到设备.");
				return;
			}
#endif
			ncnn_model.load_param(ncnn_param_path.c_str());
			ncnn_model.load_model(ncnn_model_path.c_str());
#endif


			for (int i = 0; i < (int)grid.size(); ++i)
			{
				const auto& cell = grid.at(i);

				if (cell.GetSize() <= 0)
					continue;

				BoundingBox2D bbox2d(cell.xMin() + offset.x(),
									 cell.yMin() + offset.y(),
									 cell.xMax() + offset.x(),
									 cell.yMax() + offset.y());

				cv::Mat img, label, score;
				readDOMImage(ds, bbox2d, img);

				if (img.empty())
					continue;

				double xRes = (bbox2d.xMax() - bbox2d.xMin()) / (double)img.cols;
				double yRes = (bbox2d.yMin() - bbox2d.yMax()) / (double)img.rows;

#ifdef WRITE_MASK_IMAGE
				std::string directory = DebugDirectory + StringPrintf(R"(%d\)", i);
				std::string img_path = directory + StringPrintf("img%d.png", i);
				CreateDirIfNotExists(directory);
				cv::imwrite(img_path, img); // 写入RGB图像
#endif
				cv::Size img_size(img.cols, img.rows);
				// #ifdef _OPENMP
				// #pragma omp critical
				// #endif
				{
					// WARNING: 预测图像，不可并发
					// inference time: 100~200ms
#ifdef INFERENCE_USE_NCNN
					pp_stdnet2_inference_ncnn(ncnn_model, 512, img, label, score);
#endif
				}

				// 预测数据后处理
				cv::Mat mask(label.rows, label.cols, CV_8UC1, cv::Scalar(0));

				for (int r = 0; r < label.rows; ++r)
				{
					for (int c = 0; c < label.cols; ++c)
					{
						if (label.at<uint8_t>(r, c) > 0 && score.at<float>(r, c) > scoreThr)
							mask.at<uint8_t>(r, c) = 255;
					}
				}

				// 调整大小到原始尺寸
				cv::resize(mask, mask, img_size, cv::INTER_NEAREST);
				cv::resize(label, label, img_size, cv::INTER_NEAREST);

				// 移除较小面积区域
				std::vector<std::vector<cv::Point>> contours;
				std::vector<cv::Vec4i> hierarchy;
				cv::findContours(mask, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

				for (size_t i = 0; i < contours.size(); ++i)
				{
					double area = cv::contourArea(contours[i]) * (xRes * yRes);

					if (area < minArea)
						cv::fillPoly(mask, contours[i], cv::Scalar(0));
				}

				// 更新label多分类掩码
				for (int r = 0; r < mask.rows; ++r)
					for (int c = 0; c < mask.cols; ++c)
						if (mask.at<uint8_t>(r, c) == 0)
							label.at<uint8_t>(r, c) = 0;

#ifdef WRITE_MASK_IMAGE

				auto colorMap = createPseudoColors(256);
				cv::Mat dst(label.rows, label.cols, CV_8UC3, cv::Scalar(255, 255, 255));

				for (int r = 0; r < label.rows; ++r)
				{
					for (int c = 0; c < label.cols; ++c)
					{
						int index = (int)label.at<uint8_t>(r, c);
						if (index > 0)
						{
							dst.at<cv::Vec3b>(r, c) = cv::Vec3b(colorMap[index][0] * 255,
																colorMap[index][1] * 255,
																colorMap[index][2] * 255);
						}
					}
				}

				std::string mask_path = directory + StringPrintf("mask%d.png", i);
				cv::imwrite(mask_path, dst);
#endif

				// 计算格网单元内点云位置是否在mask范围中
				auto cellIndices = cell.GetIndices();

				for (size_t j = 0; j < cellIndices.size(); ++j)
				{
					const auto& p = _input->points[cellIndices[j]];
					float xStart = bbox2d.xMin() - offset.x();
					float yStart = bbox2d.yMax() - offset.y();

					if (p.hag > 3.0)
						continue;

					int c = std::floor((p.x - xStart) / xRes);
					int r = std::floor((p.y - yStart) / yRes);

					c = clamp(c, 0, mask.cols - 1);
					r = clamp(r, 0, mask.rows - 1);

					// 农作物类别处理...
					uint8_t index = label.at<uint8_t>(r, c);
					if (index > 0)
						outindices.push_back(cellIndices[j]);
				}
			}
		}

	}
}