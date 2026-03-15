#include <src/segmentation/treeSpeciesIdentification.h>
#include <src/segmentation/gridCell.h>

#include <src/core/private/statistics.h>
#include <src/core/private/gdalProcess.h>
#include <src/core/private/rasterProcess.h>
#include <src/core/deeplearning/inference.h>
#include <src/plot/plotHandle.h>
#include <src/utils/misc.h>

#include <vulkan/vulkan.h>
#include <ncnn/net.h>

#include <numeric>
#include <algorithm>

// #define WRITE_MASK_IMAGE

extern std::string DebugDirectory;

namespace d3s {
	namespace pcs {

		// TreeSpeciesIndendtifyOptions
		//////////////////////////////////////////////////////////////////////////
		void TreeSpeciesIndendtifyOptions::Print()
		{
			PCS_INFO("cache_dir: %s", cache_dir.c_str());
			PCS_INFO("ncnn_param: %s", ncnn_param.c_str());
			PCS_INFO("ncnn_model: %s", ncnn_model.c_str());
			PCS_INFO("num_classes: %d", num_classes);

			for (size_t i = 0; i < dom_files.size(); ++i)
				PCS_INFO("filename %d: %s", i, dom_files[i].c_str());
		}

		// TreeSpeciesIdentify
		//////////////////////////////////////////////////////////////////////////


		TreeSpeciesIdentify::TreeSpeciesIdentify(const TreeSpeciesIndendtifyOptions& options,
												 PointCloudViewPtr input,
												 const std::vector<int>& indices,
												 std::function<void(double)> progress_func)
			: _options(options), _input(input), _indices(indices), _progress_func(progress_func)
		{
		}

		TreeSpeciesIdentify::~TreeSpeciesIdentify() {}

		void TreeSpeciesIdentify::Run()
		{
			// 兼容空索引
			if (_indices.empty())
			{
				for (size_t i = 0; i < _input->size(); ++i)
				{
					const auto& p = _input->points[i];
					// 获取树木点云
					if (p.data[3] > 0)
						_indices.push_back(i);
				}
			}

			// 未能获取到可以处理的点云数据索引
			if (_indices.empty())
			{
				PCS_WARN("[TreeSpeciesIdentify] 待处理点云索引为空!");
				return;
			}

			// 初始化树种分类点云颜色映射
			/*
			ColorManager::treeSpecies.resize(_input->size(), -1);
			ColorManager::treeSpeciesColorMap.clear();

			ColorManager::treeSpeciesColorMap[0] = osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f);	  // 红
			ColorManager::treeSpeciesColorMap[1] = osg::Vec4(1.0f, 0.647f, 0.0f, 1.0f);	  // 橙
			ColorManager::treeSpeciesColorMap[2] = osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f);	  // 黄
			ColorManager::treeSpeciesColorMap[3] = osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f);	  // 绿
			ColorManager::treeSpeciesColorMap[4] = osg::Vec4(0.0f, 1.0f, 1.0f, 1.0f);	  // 青
			ColorManager::treeSpeciesColorMap[5] = osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f);	  // 蓝
			ColorManager::treeSpeciesColorMap[6] = osg::Vec4(0.502f, 0.0f, 0.502f, 1.0f); // 紫
			*/

			/*
			cv::RNG rng(time(0));
			for (int i = 0; i < _options.num_classes; ++i)
			{
				float r, g, b;
				b = rng.uniform(0.0f, 1.0f);
				g = rng.uniform(0.0f, 1.0f);
				r = rng.uniform(0.0f, 1.0f);

				ColorManager::treeSpeciesColorMap[i] = osg::Vec4(r, g, b, 1.0f);
			}*/

			// 树种类别映射
			// std::vector<Classification> treeClasses = {
			//	eTree_Cedar,   // 杉树 101
			//	eTree_Pine,	   // 松树 102
			//	eTree_Cypress, // 柏树 103
			//	eTree_Bamboo,  // 竹子 104
			//	eTree_Camphor  // 樟树 105
			//};

			// 按树编号统计点云索引
			std::map<int, std::vector<int>> indicesPerTree;
			std::vector<std::vector<int>> arrIndices;

			for (size_t i = 0; i < _indices.size(); ++i)
			{
				const auto& p = _input->points[_indices[i]];
				uint32_t treeId = p.data[3];
				indicesPerTree[treeId].push_back(_indices[i]);
			}

			if (_progress_func)
				_progress_func(5.0);

			// 统计每颗树的点云索引
			arrIndices.reserve(indicesPerTree.size());
			for (const auto& kv : indicesPerTree)
				arrIndices.push_back(kv.second);

			std::string cache_dir = _options.cache_dir;
			std::string ncnn_param_path = _options.ncnn_param;
			std::string ncnn_model_path = _options.ncnn_model;
			std::vector<std::string> dom_filenames = _options.dom_files;
			std::vector<std::string> dom_files;

			// 统计有效影像文件列表
			for (const auto& filename : dom_filenames)
			{
				if (!filename.empty() && ExistsFile(filename))
					dom_files.push_back(filename);
			}

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
				PCS_ERROR("[TreeSpeciesIdentify] 推理失败，未能检测到设备.");
				return;
			}
#endif
			ncnn_model.load_param(ncnn_param_path.c_str());
			ncnn_model.load_model(ncnn_model_path.c_str());

			auto ds = createDOMDataset(dom_files);
			auto offset = _input->offset_xyz;

			if (_progress_func)
				_progress_func(10.0);

			int idx = 0;

			// 根据单颗树点云截取DOM图像
#ifdef _OPENMP
#pragma omp parallel for num_threads(10)
#endif
			for (int i = 0; i < (int)arrIndices.size(); ++i)
			{
				const auto& indices = arrIndices[i];

				osg::BoundingBox bbox;
				computeMinMax3D(*_input, indices, bbox);

				BoundingBox2D bbox2d(bbox.xMin() + offset.x(),
									 bbox.yMin() + offset.y(),
									 bbox.xMax() + offset.x(),
									 bbox.yMax() + offset.y());

				cv::Mat img, score;
#pragma omp critical
				{
					readDOMImage(ds, bbox2d, img, cache_dir);
				}

				if (img.empty())
				{
					PCS_WARN("无法从DOM中获取数据.");
					continue;
				}

				double xRes = (bbox2d.xMax() - bbox2d.xMin()) / (double)img.cols;
				double yRes = (bbox2d.yMin() - bbox2d.yMax()) / (double)img.rows;

				cv::Mat dst(img.rows, img.cols, CV_8UC3, cv::Scalar(255, 255, 255));
				double resolution = 0.2;
				int csize = std::abs(std::floor(resolution / xRes));
				int rsize = std::abs(std::floor(resolution / yRes));

				for (size_t j = 0; j < indices.size(); ++j)
				{
					const auto& p = _input->points[indices[j]];
					float xStart = bbox2d.xMin() - offset.x();
					float yStart = bbox2d.yMax() - offset.y();

					int c = std::floor((p.x - xStart) / xRes);
					int r = std::floor((p.y - yStart) / yRes);

					c = clamp(c, 0, dst.cols - 1);
					r = clamp(r, 0, dst.rows - 1);

					int cmin = clamp(c - csize, 0, dst.cols - 1);
					int rmin = clamp(r - rsize, 0, dst.rows - 1);
					int width = 2 * csize;
					int height = 2 * rsize;

					if ((cmin + width) > dst.cols)
						width = dst.cols - cmin;

					if ((rmin + height) > dst.rows)
						height = dst.rows - rmin;

					if (height == 0 || width == 0)
						continue;
					cv::Rect roi(cmin, rmin, width, height);
					img(roi).copyTo(dst(roi));
				}

				int numEmptyPixels = 0;

				for (int r = 0; r < dst.rows; ++r)
				{
					for (int c = 0; c < dst.cols; ++c)
					{
						cv::Vec3b pixel = dst.at<cv::Vec3b>(r, c);

						if (pixel(0) == 255 && pixel(1) == 255 && pixel(2) == 255)
							++numEmptyPixels;

						if (pixel(0) == 0 && pixel(1) == 0 && pixel(2) == 0)
							++numEmptyPixels;
					}
				}

				// 剔除存在大量留白的图像
				if (numEmptyPixels > (0.9 * dst.rows * dst.cols))
				{

#ifdef WRITE_MASK_IMAGE
					std::string path = StringPrintf(R"(%s%d.png)", DebugDirectory.c_str(), i);
					// 写入RGB图像
					cv::imwrite(path, dst);
#endif
				}
				else
				{
					std::vector<float> scores;
					std::vector<int> idx;
#pragma omp critical
					{
						pplcnet_x1_0_inference_ncnn(ncnn_model, 224, dst, scores);
					}

					idx.resize(scores.size());
					std::iota(idx.begin(), idx.end(), 0);

					int maxPosition =
						std::max_element(scores.begin(), scores.end()) - scores.begin();
					float maxScore = scores[maxPosition];

					PCS_DEBUG("num_classes: %d, class index: %d, score: %.4f",
							  scores.size(),
							  maxPosition,
							  maxScore);

					for (size_t j = 0; j < indices.size(); ++j)
					{
						auto& p = _input->points[indices[j]];
						// ColorManager::treeSpecies[indices[j]] = maxPosition;

						// 不在树种类别范围内，标记为其他树种
						if (maxScore < 0.5)
							p.label = eTree_Other;
						else
							p.label = eTree_Start + maxPosition + 1;
					}
				}

#pragma omp critical
				{
					idx++;
					if (_progress_func)
						_progress_func(10.0 + (double)idx * 80.0 / (double)arrIndices.size());
				}
			}

			closeDOMDataset(ds);

			ncnn_model.clear(); // 退出前，清除
#ifdef NCNN_VULKAN
			VkInstance inst = ncnn::get_gpu_instance();
			if (inst)
				ncnn::destroy_gpu_instance();
#endif
		}

	}
}