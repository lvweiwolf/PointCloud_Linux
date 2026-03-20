#include <src/segmentation/cropClassify.h>
#include <src/segmentation/gridCell.h>
#include <src/algorithm/math.h>
#include <src/algorithm/boundingbox2d.h>
#include <src/plot/plotHandle.h>
#include <src/core/private/statistics.h>
#include <src/core/private/gdalProcess.h>
#include <src/core/private/rasterProcess.h>
#include <src/core/deeplearning/inference.h>
#include <src/utils/misc.h>

#include <ncnn/net.h>
#include <ncnn/gpu.h>

#include <random>

// #define FOREST_REGION_DEBUG
// #define WRITE_MASK_IMAGE

extern std::string DebugDirectory;

namespace d3s {
	namespace pcs {

		void CropClassifyOptions::Print()
		{
			PCS_INFO("min_height: %lf", min_height);
			PCS_INFO("max_height: %lf", max_height);
			PCS_INFO("min_area: %lf", min_area);
			PCS_INFO("min_forest_area: %lf", min_forest_area);
			PCS_INFO("pred_threshold: %lf", pred_threshold);
			PCS_INFO("cache_dir: %s", cache_dir.c_str());
			PCS_INFO("ncnn_param: %s", ncnn_param.c_str());
			PCS_INFO("ncnn_model: %s", ncnn_model.c_str());

			for (size_t i = 0; i < dom_files.size(); ++i)
				PCS_INFO("filename %d: %s", i, dom_files[i].c_str());
		}

		// CropClassify
		//////////////////////////////////////////////////////////////////////////
		CropClassify::CropClassify(const CropClassifyOptions& options,
								   PointCloudViewPtr input,
								   const std::vector<int>& indices,
								   std::function<void(double)> progress_func)
			: _options(options), _input(input), _indices(indices), _progress_func(progress_func)
		{
		}

		CropClassify::~CropClassify() {}


		void CropClassify::Run()
		{
			// 兼容空索引
			if (_indices.empty())
			{
				for (size_t i = 0; i < _input->size(); ++i)
				{
					const auto& p = _input->points[i];
					uint32_t treeId = p.data[3];

					if (treeId > 0)
						continue;

					// 获取植被点
					if (p.label == eLowVegetation || p.label == eHighVegetation)
						_indices.push_back(i);
				}
			}

			// 未能获取到可以处理的点云数据索引
			if (_indices.empty())
			{
				PCS_WARN("[CropClassify] 待处理点云索引为空!");
				return;
			}

			// 林地区域聚类、识别
			double cellsize = 2.0; // 2m
			double min_forest_area = _options.min_forest_area;
			int min_high_vegetation = 20;

			// ForestRegionClustering(cellsize, min_forest_area, min_high_vegetation);

			// 农作物识别分类
			if (!CropInference())
				return;



			// 获得农作物点云索引
			// getClassification(*_input, eCrop_Corn, _crops);
		}

		void CropClassify::ForestRegionClustering(double cellsize,
												  double min_forest_area,
												  int min_high_vegs)
		{
			typedef std::array<int, 2> Index;
			typedef std::vector<std::vector<Index>> Clusters;


			std::vector<int> vegIndices;

			// 高植被点参与区域识别聚类
			for (size_t i = 0; i < _indices.size(); ++i)
			{
				const auto& p = _input->points[_indices[i]];

				if (p.label == eHighVegetation)
					vegIndices.push_back(_indices[i]);
			}

			Grid2D grid;
			ComputeGridCells(cellsize, _input, vegIndices, grid);
			int nRows = grid.getNumRows();
			int nCols = grid.getNumColumns();

			// 统计单元内高植被点数量
			std::vector<int> highVegetationCounts(nRows * nCols, 0);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int r = 0; r < nRows; ++r)
			{
				for (int c = 0; c < nCols; ++c)
				{
					const auto& cell = grid.at(r, c);

					if (cell.GetSize() <= 0)
						continue;

					const auto& cellIndices = cell.GetIndices();

					for (size_t i = 0; i < cellIndices.size(); ++i)
					{
						const auto& p = _input->points[cellIndices[i]];

						if (p.label == eHighVegetation)
							highVegetationCounts[r * nCols + c]++;
					}
				}
			}

			// 生长聚类
			Clusters clusters;
			std::vector<bool> processed(nRows * nCols, false);

			for (int r = 0; r < nRows; ++r)
			{
				for (int c = 0; c < nCols; ++c)
				{
					int i = r * nCols + c;

					const auto& currentCell = grid.at(r, c);
					if (currentCell.GetSize() <= 0)
						continue;

					// 没有高植被点
					if (highVegetationCounts[i] <= min_high_vegs)
						continue;

					// 已处理过的，不再处理
					if (processed[i])
						continue;

					std::vector<Index> seed_queue;

					int sq_idx = 0;
					seed_queue.push_back({ r, c });
					processed[i] = true;

					while (sq_idx < (int)seed_queue.size())
					{
						Index index = seed_queue[sq_idx];
						const int rr = index[0];
						const int cc = index[1];

						const auto& cell = grid.at(rr, cc);

						// 跳过空的格网
						if (cell.GetSize() <= 0)
						{
							sq_idx++;
							continue;
						}

						//  8 邻域生长，遍历相邻接节点
						for (int yi = rr - 1; yi <= rr + 1; ++yi)
						{
							for (int xi = cc - 1; xi <= cc + 1; ++xi)
							{
								int ri = clamp(yi, 0, nRows - 1);
								int ci = clamp(xi, 0, nCols - 1);
								int j = ri * nCols + ci;

								const auto& cellNbr = grid.at(ri, ci);
								if (cellNbr.GetSize() <= 0)
									continue;

								if (highVegetationCounts[j] <= min_high_vegs)
									continue;

								if (processed[j])
									continue;

								seed_queue.push_back({ ri, ci });
								processed[j] = true;
							}
						}

						sq_idx++;
					} // end of while

					if (!seed_queue.empty())
						clusters.push_back(seed_queue);
				}
			}

			PCS_INFO("[CropClassify] 识别到 %d 个林地区域.", clusters.size());


			std::uniform_real_distribution<double> dist(0, 1); // 随机分布
			std::vector<double> regionAreas(clusters.size());
			std::vector<std::vector<int>> regionIndices(clusters.size());
			std::vector<osg::Vec4> colorMap(clusters.size());
			std::vector<osg::Vec4> colorMap2 = { osg::Vec4(1, 0, 0, 1), osg::Vec4(0, 0, 1, 1) };

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)clusters.size(); ++i)
			{
				std::random_device rd;
				std::mt19937 gen(rd());
				osg::Vec4 dummy;

				// 为有效树木分配颜色
				auto& color = colorMap[i];

				if (color == dummy)
				{
					float r, g, b;
					b = dist(gen);
					g = dist(gen);
					r = dist(gen);
					color = osg::Vec4(r, g, b, 1.0f);
				}

				auto& region = regionIndices[i];
				const auto& cluster = clusters.at(i);

				for (size_t j = 0; j < cluster.size(); ++j)
				{
					const auto& index = cluster.at(j);
					const auto& cell = grid.at(index[0], index[1]);

					if (cell.GetSize() <= 0)
						continue;

					const auto& cellIndices = cell.GetIndices();
					region.insert(region.end(), cellIndices.begin(), cellIndices.end());
				}
			}

			// 开始计算面积
			{
				osg::BoundingBox bbox;
				computeMinMax3D(*_input, _indices, bbox);

				double resolution = 1.0;
				double half = resolution / 2.0;
				double eps = resolution * .000001;

				RasterExtents extents;
				extents.resolution = resolution;
				extents.x = bbox.xMin() - half;
				extents.y = bbox.yMin() - half;
				extents.width = ((bbox.xMax() - extents.x) / extents.resolution) + 1;
				extents.height = ((bbox.yMax() - extents.y) / extents.resolution) + 1;

				Rasterd mask(extents, "colormask", 0.0);

				for (size_t i = 0; i < regionIndices.size(); ++i)
				{
					const auto& region = regionIndices.at(i);

					for (size_t j = 0; j < region.size(); ++j)
					{
						const auto& p = _input->points[region[j]];
						int xi = mask.xCell(p.x + half - eps);
						int yi = mask.yCell(p.y + half - eps);

						if (xi < 0 || xi >= mask.width() || yi < 0 || yi >= mask.height())
							continue;

						mask.at(xi, yi) = (double)(i + 1);
					}
				}

				// 统计区域面积
				for (int y = 0; y < mask.height(); ++y)
				{
					for (int x = 0; x < mask.width(); ++x)
					{
						int treeId = (int)mask.at(x, y);

						if (treeId == 0)
							continue;

						regionAreas[treeId - 1] += (resolution * resolution);
					}
				}

				for (size_t i = 0; i < regionIndices.size(); ++i)
				{
					const auto& region = regionIndices.at(i);
					const auto& area = regionAreas.at(i);

					if (area > min_forest_area)
					{
						_dense_forests.insert(_dense_forests.end(), region.begin(), region.end());
					}
					else
					{
						_sparse_forests.insert(_sparse_forests.end(), region.begin(), region.end());
					}
				}

#ifdef WRITE_MASK_IMAGE
				writeImage(mask, DebugDirectory + "forest_color_map.jpg", colorMap);

				Rasterd mask2(extents, "colormask2", 0.0);

				for (size_t i = 0; i < regionIndices.size(); ++i)
				{
					const auto& region = regionIndices.at(i);
					const auto& area = regionAreas.at(i);
					int id = area > min_forest_area ? 0 : 1;

					for (size_t j = 0; j < region.size(); ++j)
					{
						const auto& p = _input->points[region[j]];
						int xi = mask2.xCell(p.x + half - eps);
						int yi = mask2.yCell(p.y + half - eps);

						if (xi < 0 || xi >= mask2.width() || yi < 0 || yi >= mask2.height())
							continue;

						mask2.at(xi, yi) = (double)(id + 1);
					}
				}

				writeImage(mask2, DebugDirectory + "forest_color_map2.jpg", colorMap2);
#endif
			}


#ifdef FOREST_REGION_DEBUG
			ColorManager::treeSegments.resize(_input->size(), -1);
			ColorManager::treeColorMap.clear();

			for (size_t i = 0; i < colorMap2.size(); ++i)
				ColorManager::treeColorMap[i] = colorMap2.at(i);

			for (size_t i = 0; i < regionIndices.size(); ++i)
			{
				const auto& region = regionIndices[i];
				const auto& area = regionAreas.at(i);
				int id = area > min_forest_area ? 0 : 1;

				for (size_t j = 0; j < region.size(); ++j)
					ColorManager::treeSegments[region[j]] = id;
			}
#endif
		}

		bool CropClassify::CropInference()
		{
			std::string cache_dir = _options.cache_dir;
			std::string ncnn_param_path = _options.ncnn_param;
			std::string ncnn_model_path = _options.ncnn_model;
			double min_area = _options.min_area;
			double pred_threshold = _options.pred_threshold;
			double filter_min_height = _options.min_height;
			double filter_max_height = _options.max_height;
			std::vector<std::string> dom_filenames = _options.dom_files;
			std::vector<std::string> dom_files;

			// 统计有效影像文件列表
			for (const auto& filename : dom_filenames)
			{
				if (!filename.empty() && ExistsFile(filename))
					dom_files.push_back(filename);
			}


			std::vector<int> indices;

			for (size_t i = 0; i < _indices.size(); ++i)
			{
				const auto& p = _input->points[_indices[i]];

				// 植被点按高度过滤
				// 2024.7.11 wjf,去掉高度过滤，需要识别果树为经济作物
				indices.push_back(_indices[i]);
			}


			// 根据地理投影分辨率计算划分尺寸(以米为单位)
			double cellsize = 30.0;
			double padding = 0.0;
			const auto& offset = _input->offset_xyz;

			// 计算并划分网格单元
			Grid2D grid;
			ComputeGridCells(cellsize, _input, indices, grid);

			if (_progress_func)
				_progress_func(5.0);

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
				PCS_ERROR("[CropClassify] 推理失败，未能检测到设备.");
				return false;
			}
#endif
			ncnn_model.load_param(ncnn_param_path.c_str());
			ncnn_model.load_model(ncnn_model_path.c_str());

			//  创建栅格数据集

			if (_progress_func)
				_progress_func(10.0);

			int idx = 0;

#ifdef _OPENMP
#pragma omp parallel for num_threads(6)
#endif
			for (int i = 0; i < (int)grid.size(); ++i)
			{
				const auto& cell = grid.at(i);

				if (cell.GetSize() <= 0)
					continue;

				// WARNING: 边界扩充 padding, 缓解分割结果包围盒边界明显问题
				BoundingBox2D bbox2d(cell.xMin() + offset.x() - padding,
									 cell.yMin() + offset.y() - padding,
									 cell.xMax() + offset.x() + padding,
									 cell.yMax() + offset.y() + padding);

				cv::Mat img, score, pred;
				std::shared_ptr<DOMDataset> ds = nullptr;

				try
				{
					ds = createDOMDataset(dom_files);
					readDOMImage(ds, bbox2d, img, cache_dir);
					closeDOMDataset(ds);
				}
				catch (...)
				{
					PCS_ERROR("[CropClassify] 存在gdal句柄冲突");
				}

				if (img.empty())
					continue;

				double xRes = (bbox2d.xMax() - bbox2d.xMin()) / (double)img.cols;
				double yRes = (bbox2d.yMin() - bbox2d.yMax()) / (double)img.rows;

#ifdef WRITE_MASK_IMAGE
				std::string directory = DebugDirectory + StringPrintf(R"(%d\)", i);
				CreateDirIfNotExists(directory);

				// 写入RGB图像
				cv::imwrite(directory + "img.png", img);
#endif
				cv::Size img_size(img.cols, img.rows);

				// #pragma omp critical
				{
					// WARNING: 预测图像，不可并发, inference time: 100~200ms
					pp_stdnet2_inference_ncnn(ncnn_model, 512, img, score, pred);
				}


				// 预测数据后处理
				cv::Mat mask(score.rows, score.cols, CV_8UC1, cv::Scalar(0));

				{
					for (int r = 0; r < score.rows; ++r)
					{
						for (int c = 0; c < score.cols; ++c)
						{
							if (score.at<uint8_t>(r, c) > 0 &&
								pred.at<float>(r, c) > pred_threshold)
							{
								mask.at<uint8_t>(r, c) = 255;
							}
						}
					}

					// 调整大小到原始尺寸
					cv::resize(mask, mask, img_size, cv::INTER_NEAREST);
					cv::resize(score, score, img_size, cv::INTER_NEAREST);

					// 移除较小面积区域
					std::vector<std::vector<cv::Point>> contours;
					std::vector<cv::Vec4i> hierarchy;
					cv::findContours(mask,
									 contours,
									 hierarchy,
									 cv::RETR_LIST,
									 cv::CHAIN_APPROX_NONE);


					for (size_t i = 0; i < contours.size(); ++i)
					{
						double area = cv::contourArea(contours[i]) * fabs(xRes * yRes);
						if (area < min_area)
							cv::fillPoly(mask, contours[i], cv::Scalar(0));
					}

					// 更新score多分类掩码
					for (int r = 0; r < mask.rows; ++r)
					{
						for (int c = 0; c < mask.cols; ++c)
						{
							if (mask.at<uint8_t>(r, c) != 255)
							{
								mask.at<uint8_t>(r, c) = 0;
								score.at<uint8_t>(r, c) = 0;
							}
						}
					}
				}

#ifdef WRITE_MASK_IMAGE
				auto colorMap = createPseudoColors(256);
				cv::Mat dst(score.rows, score.cols, CV_8UC3, cv::Scalar(0));

				for (int r = 0; r < score.rows; ++r)
				{
					for (int c = 0; c < score.cols; ++c)
					{
						int index = (int)score.at<uint8_t>(r, c);
						if (index > 0)
						{
							dst.at<cv::Vec3b>(r, c) = cv::Vec3b(colorMap[index][0] * 255,
																colorMap[index][1] * 255,
																colorMap[index][2] * 255);
						}
					}
				}

				cv::imwrite(directory + "mask.png", dst);
#endif

				// 计算格网单元内点云位置是否在mask范围中
				auto cellIndices = cell.GetIndices();
				std::vector<bool> hits(mask.cols * mask.rows, false);

				for (size_t j = 0; j < cellIndices.size(); ++j)
				{
					auto& p = _input->points[cellIndices[j]];
					float xStart = bbox2d.xMin() - offset.x();
					float yStart = bbox2d.yMax() - offset.y();

					int c = std::floor((p.x - xStart) / xRes);
					int r = std::floor((p.y - yStart) / yRes);

					c = clamp(c, 0, mask.cols - 1);
					r = clamp(r, 0, mask.rows - 1);

					// 农作物类别处理...
					uint8_t label = score.at<uint8_t>(r, c);

					if (label > 0)
						p.label = eCrop_Start + label;
				}

				// 统计类别像素数量
				// #pragma omp critical
				// 				{
				// 					idx++;
				// 					if (_progress_func)
				// 						_progress_func(10.0 + (double)idx * 80.0 /
				// (double)grid.size());
				//
				// 				}
			}


			ncnn_model.clear(); // 退出前，清除

#ifdef NCNN_VULKAN
			VkInstance inst = ncnn::get_gpu_instance();
			if (inst)
				ncnn::destroy_gpu_instance();

#endif
			return true;
		}

	}
}
