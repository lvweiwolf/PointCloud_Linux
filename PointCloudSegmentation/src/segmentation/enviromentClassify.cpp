#include <src/segmentation/enviromentClassify.h>
#include <src/segmentation/gridCell.h>
#include <src/segmentation/buildingExtraction.h>
#include <src/segmentation/improve/roadClassifier.h>

#include <include/ClassificationDef.h>

#include <src/utils/logging.h>


namespace d3s {
	namespace pcs {

		// EnviromentClassifyOptions
		//////////////////////////////////////////////////////////////////////////
		void EnviromentClassifyOptions::Print()
		{
			PCS_INFO("denoise: %s", denoise ? "true" : "false");
			PCS_INFO("denoise_voxel_size: %lf", denoise_voxel_size);
			PCS_INFO("denoise_min_pts: %d", denoise_min_pts);

			PCS_INFO("cell_size: %lf", cell_size);
			PCS_INFO("min_vegetation_height: %lf", min_vegetation_height);
			PCS_INFO("min_high_vegetation_height: %lf", min_high_vegetation_height);
			PCS_INFO("min_building_area: %lf", min_building_area);
			PCS_INFO("max_building_area: %lf", max_building_area);
			PCS_INFO("min_building_height: %lf", min_building_height);
			PCS_INFO("euclidean_cluster_distance: %lf", euclidean_cluster_distance);
			PCS_INFO("planar_ransac_distance: %lf", planar_ransac_distance);
			PCS_INFO("planar_inlier_pts_ratio: %lf", planar_inlier_pts_ratio);
			PCS_INFO("euclidean_cluster_min_pts: %d", euclidean_cluster_min_pts);

			PCS_INFO("road_cell_size: %lf", road_cell_size);
			PCS_INFO("road_height_threshold: %lf", road_height_threshold);
			PCS_INFO("road_slope_threshold: %lf", road_slope_threshold);
			PCS_INFO("road_min_area: %lf", road_min_area);
			PCS_INFO("road_max_area: %lf", road_max_area);
			PCS_INFO("road_shape_threshold: %lf", road_shape_threshold);
			PCS_INFO("road_linearity_threshold: %lf", road_linearity_threshold);
		}

		// EnviromentClassify
		//////////////////////////////////////////////////////////////////////////
		EnviromentClassify::EnviromentClassify(const EnviromentClassifyOptions& options,
											   PointCloudViewPtr input)
			: _options(options), _input(input), _grid(new Grid2D())
		{
		}

		EnviromentClassify::~EnviromentClassify() {}


		void EnviromentClassify::SetIndices(const std::vector<int>& indices) { _indices = indices; }

		void EnviromentClassify::SetRoadVectorize(d3s::share_ptr<IValueBuffer> roadVectorize)
		{
			_roadVectorize = roadVectorize;
		}

		d3s::pcs::Grid2D EnviromentClassify::GetGrid() const { return *_grid; }

		void EnviromentClassify::Run()
		{
			// 兼容空索引
			if (_indices.empty())
			{
				_indices.reserve(_input->size());

				for (size_t i = 0; i < _input->size(); ++i)
				{
					const auto& p = _input->points[i];

					if (p.label == eUnclassified)
						_indices.push_back(i);
				}

				_indices.shrink_to_fit();
			}

			// 植被、建筑分类前进行降噪
			if (_options.denoise)
			{
				double voxelsize = _options.denoise_voxel_size;
				unsigned int minpts = _options.denoise_min_pts;

				std::vector<int> inliers;
				std::vector<std::vector<int>> clusters;

				EuclideanVoxelCluster(_input,
									  _indices,
									  voxelsize,
									  minpts,
									  std::numeric_limits<int>::max(),
									  clusters);

				for (size_t i = 0; i < clusters.size(); ++i)
				{
					const auto& cluster = clusters.at(i);
					inliers.insert(inliers.end(), cluster.begin(), cluster.end());
				}

				if (!inliers.empty())
					_indices = inliers;
			}

			// 计算并划分网格单元
			ComputeGridCells(_options.cell_size, _input, _indices, *_grid);

			// 植被初步粗提取
			VegetationCandidateClassify();

			// 提取建筑区域
			{
				BuildingExtractionOptions options;

				// 默认设置
				options.euclidean_cluster_distance = _options.euclidean_cluster_distance;
				options.euclidean_cluster_min_pts = _options.euclidean_cluster_min_pts;
				options.planar_ransac_distance = _options.planar_ransac_distance;
				options.planar_inlier_pts_ratio = _options.planar_inlier_pts_ratio;
				options.min_building_area = _options.min_building_area;
				options.max_building_area = _options.max_building_area;
				options.min_building_height = _options.min_building_height;
				options.morph_kernel_size = 3;

				BuildingExtraction extractor(options, _input);
				extractor.Extract();
			}

			// 以下为道路分类的实现
			{
				std::vector<int> ground;
				ground.reserve(_input->size());

				for (size_t i = 0; i < _input->size(); ++i)
				{
					const auto& p = _input->points[i];

					if (p.label == eGround || p.hag < 3.0)
						ground.push_back(i);
				}

				ground.shrink_to_fit();


				RoadClassifyOptions options;
				options.cell_size = _options.road_cell_size;
				options.height_threshold = _options.road_height_threshold;
				options.slope_threshold = _options.road_slope_threshold;
				options.min_area = _options.road_min_area;
				options.max_area = _options.road_max_area;
				options.shape_threshold = _options.road_shape_threshold;
				options.linearity_threshold = _options.road_linearity_threshold;

				RoadClassifier roadClassifier(options, _input, ground);
				roadClassifier.SetRoadVectorize(_roadVectorize);
				roadClassifier.Segment();
			}
		}

		void EnviromentClassify::VegetationCandidateClassify()
		{
#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int k = 0; k < (int)_grid->size(); ++k)
			{
				GridCell& cell = _grid->at(k);
				const std::vector<int>& indices = cell.GetIndices();

				// 跳过不包含点云的单元
				if (indices.empty())
					continue;

				double local_hag_min = DBL_MAX;
				double local_hag_max = -DBL_MAX;

				for (size_t i = 0; i < indices.size(); ++i)
				{
					const auto& p = _input->points[indices[i]];

					local_hag_min = std::min(local_hag_min, (double)p.hag);
					local_hag_max = std::max(local_hag_max, (double)p.hag);
				}

				if (local_hag_max <= _options.min_vegetation_height)
					cell.label = eGround;

				if (local_hag_max > _options.min_vegetation_height &&
					local_hag_max <= _options.min_high_vegetation_height)
				{
					cell.label = eLowVegetation;
				}

				if (local_hag_max > _options.min_high_vegetation_height)
				{
					cell.label = eHighVegetation;
				}

				for (size_t i = 0; i < indices.size(); ++i)
				{
					auto& p = _input->points[indices[i]];

					if (p.hag <= _options.min_vegetation_height)
						p.label = eGround;

					if (p.hag > _options.min_vegetation_height &&
						p.hag <= _options.min_high_vegetation_height)
					{
						p.label = eLowVegetation;
					}

					if (p.hag > _options.min_high_vegetation_height)
					{
						p.label = eHighVegetation;
					}
				}
			}
		}

	}
}