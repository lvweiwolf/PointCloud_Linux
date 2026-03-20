#include <src/service/Segmentation.h>
#include <src/service/PointCloud.h>
#include <src/algorithm/voxel_grid_filter.h>
#include <src/core/api.h>
#include <src/core/pointTypes.h>
#include <src/core/private/rasterProcess.h>
#include <src/core/private/cloudProcess.h>
#include <src/core/private/filters.h>
#include <src/core/private/statistics.h>
#include <src/segmentation/powerCorridorsClassify.h>
#include <src/segmentation/enviromentClassify.h>
#include <src/segmentation/cropClassify.h>
#include <src/segmentation/treeSegmentation.h>
#include <src/segmentation/treeSpeciesIdentification.h>
#include <src/segmentation/pylonLocation.h>
#include <src/segmentation/powerlineCurve.h>
#include <src/segmentation/dangerSegmentation.h>
#include <src/reconstruct/pylonReconstruct.h>

#include <osg/BoundingBox>
#include <numeric>


extern std::string DebugDirectory;

namespace d3s {
	namespace pcs {

		bool CloudNormalize(PointCloudView<PointPCLH>& cloud)
		{
			if (!cloud.normalized)
			{
				std::vector<int> ground;
				getClassification(cloud, eGround, ground);

				if (ground.empty())
					return false;

#if 1
				// 创建稀疏的格网，加速地形创建
				osg::BoundingBox bbox = cloud.bbox;
				PointCloudView<PointPCLH>::PointCloud points;

				for (size_t i = 0; i < ground.size(); ++i)
				{
					const auto& p = cloud.points[ground[i]];
					points.push_back(p);
				}

				VoxelGridSample(points,
								Eigen::Vector3f(bbox.xMin(), bbox.yMin(), bbox.zMin()),
								Eigen::Vector3f(bbox.xMax(), bbox.yMax(), bbox.zMax()),
								0.5);


				Rasterd raster;
				createCloudRaster(points, cloud.bbox, 1.0, raster); // 1px = 1m
#else
				Rasterd raster;
				createCloudRaster(pointCloudView, ground, 1.0, raster);
#endif

				if (!normalizeByGround(raster, cloud))
					return false;
			}

			return cloud.normalized;
		}

		// CGroundSegmentation
		//////////////////////////////////////////////////////////////////////////
		CGroundSegmentation::~CGroundSegmentation() {}

		bool CGroundSegmentation::Segment(const IOptions* options, std::vector<PointId>& result)
		{
			CHECK(options);

			if (!options)
				return false;

			if (!_cloud.valid())
				return false;

			d3s::share_ptr<CPointCloud> cloud = dynamic_cast<CPointCloud*>(_cloud.get());
			CHECK(cloud.valid());

			PointCloudViewPtr pcv = cloud->Data();
			CHECK(pcv);

			auto& pointCloudView = *pcv;

			if (pointCloudView.empty())
				return false;


			int max_window_size = options->GetInt("max_window_size");		  // 最大窗口大小
			float cell_size = options->GetFloat("cell_size");				  // 单元大小
			float slope = options->GetFloat("slope");						  // 坡度
			float initial_distance = options->GetFloat("initial_distance");	  // 初始距离阈值
			float max_distance = options->GetFloat("max_distance");			  // 最大距离阈值
			double morph_resolution = options->GetDouble("morph_resolution"); // 悬浮物滤波分辨率
			int morph_size = options->GetFloat("morph_size");				  // 悬浮物滤波窗口尺寸

			PCS_INFO("[Ground Segmentation] Parameters:");
			PCS_INFO("max_window_size : %d", max_window_size);
			PCS_INFO("cell_size : %lf", cell_size);
			PCS_INFO("slope : %lf", slope);
			PCS_INFO("initial_distance : %lf", initial_distance);
			PCS_INFO("max_distance : %lf", max_distance);
			PCS_INFO("morph_resolution : %lf", morph_resolution);
			PCS_INFO("morph_size : %d", morph_size);

			float base = 2.0f;
			bool exponential = true;
			// PROGRESS(0.0, "正在提取地面点...");

			// 初始化类别
			setClassification(eUnclassified, pointCloudView);

			std::vector<int> ground;
			std::vector<int> indices;

			if (_indices.empty())
			{
				_indices.resize(pointCloudView.size());
				std::iota(_indices.begin(), _indices.end(), 0);
			}

			// 去除悬空导线对地面点提取的干扰
			pointCloudMorph(pointCloudView, _indices, morph_resolution, morph_size, indices);
			// PROGRESS(20.0, "正在提取地面点...");


			if (indices.empty())
				return false;


			// 执行 PMF 算法
			bool bRet = groundSegmentation_ApproximatePMF(pointCloudView,
														  indices,
														  ground,
														  max_window_size,
														  slope,
														  max_distance,
														  initial_distance,
														  cell_size,
														  base,
														  exponential);
			// PROGRESS(80.0, "即将完成地面点提取...");

			// 设置地面点类别
			setClassification(eGround, ground, pointCloudView);
			result = ground;
			// PROGRESS(100.0, "地面点提取完成.");

			return bRet;
		}

		// CPowerCorridorSegmentaion
		//////////////////////////////////////////////////////////////////////////
		CPowerCorridorSegmentaion::~CPowerCorridorSegmentaion() {}

		bool CPowerCorridorSegmentaion::Segment(const IOptions* options,
												std::vector<PointId>& result)
		{
			CHECK(options);

			if (!options)
				return false;

			if (!_cloud.valid())
				return false;

			d3s::share_ptr<CPointCloud> cloud = dynamic_cast<CPointCloud*>(_cloud.get());
			CHECK(cloud.valid());

			PointCloudViewPtr pcv = cloud->Data();
			CHECK(pcv);

			auto& pointCloudView = *pcv;

			if (pointCloudView.empty())
				return false;

			Timer timer;
			timer.Start();

			PowerCorridorsClassifyOptions opts;

			// 遗留算法参数
			opts.max_span_error = options->GetInt("max_span_error");
			opts.min_span_angle_theta = options->GetDouble("min_span_angle_theta");
			opts.min_body_growing_height = options->GetDouble("min_body_growing_height");
			opts.max_body_growing_height = options->GetDouble("max_body_growing_height");
			opts.body_growing_step = options->GetDouble("body_growing_step");
			opts.body_growing_resolition = options->GetDouble("body_growing_resolition");
			opts.max_curve_dist = options->GetDouble("max_curve_dist");

			// 配网线路分类参数
			opts.distribution = options->GetBool("distribution");

			// 杆塔定位参数
			opts.location_cellsize = options->GetDouble("location_cellsize");
			opts.location_zstep = options->GetDouble("location_zstep");
			opts.location_th1 = options->GetDouble("location_th1");
			opts.location_th2 = options->GetDouble("location_th2");
			opts.location_tv = options->GetDouble("location_tv");
			opts.location_tc = options->GetDouble("location_tc");
			opts.location_th = options->GetDouble("location_th");
			opts.location_outlier_coeff = options->GetDouble("location_outlier_coeff");
			opts.location_linearity = options->GetDouble("location_linearity");
			opts.location_min_proj = options->GetDouble("location_min_proj");
			opts.location_max_span = options->GetDouble("location_max_span");
			opts.location_corridor_width = options->GetDouble("location_corridor_width");
			opts.location_neighbors = options->GetInt("location_neighbors");

			// 杆塔提取参数
			opts.pylon_head_length = options->GetDouble("pylon_head_length");
			opts.pylon_d1_scale = options->GetDouble("pylon_d1_scale");
			opts.pylon_d2_scale = options->GetDouble("pylon_d2_scale");
			opts.pylon_terminal_scale = options->GetDouble("pylon_terminal_scale");
			opts.pylon_corner_scale = options->GetDouble("pylon_corner_scale");

			// 电力线分类
			opts.powerline_slice_step = options->GetDouble("powerline_slice_step");
			opts.powerline_slice_thickness = options->GetDouble("powerline_slice_thickness");
			opts.powerline_cluster_th = options->GetDouble("powerline_cluster_th");
			opts.powerline_cluster_radius = options->GetDouble("powerline_cluster_radius");
			opts.powerline_cluster_min_points = options->GetInt("powerline_cluster_min_points");
			opts.powerline_curve_min_points = options->GetInt("powerline_curve_min_points");
			opts.powerline_curve_expand_length =
				options->GetDouble("powerline_curve_expand_length");
			opts.powerline_curve_fit_fast = options->GetBool("powerline_curve_fit_fast");

			// 杆塔细化
			opts.refine_cluster_radius = options->GetDouble("refine_cluster_radius");
			opts.refine_connect_radius = options->GetDouble("refine_connect_radius");
			opts.refine_cluster_min_points = options->GetInt("refine_cluster_min_points");
			opts.refine_curve_fit_fast = options->GetBool("refine_curve_fit_fast");

			// 绝缘子、跳线分类
			opts.attachment_enable = options->GetBool("attachment_enable");
			opts.attachment_voxel_size = options->GetDouble("attachment_voxel_size");
			opts.attachment_search_radius = options->GetDouble("attachment_search_radius");
			opts.attachment_x_length = options->GetDouble("attachment_x_length");
			opts.attachment_y_length = options->GetDouble("attachment_y_length");
			opts.attachment_z_length = options->GetDouble("attachment_z_length");
			opts.attachment_x_fillrate = options->GetDouble("attachment_x_fillrate");
			opts.attachment_y_fillrate = options->GetDouble("attachment_y_fillrate");
			opts.attachment_xy_fillrate = options->GetDouble("attachment_xy_fillrate");
			opts.attachment_xz_fillrate = options->GetDouble("attachment_xz_fillrate");
			opts.attachment_leadwire_x_fr = options->GetDouble("attachment_leadwire_x_fr");
			opts.attachment_leadwire_x_len = options->GetDouble("attachment_leadwire_x_len");
			opts.attachment_suspension_method = options->GetStr("attachment_suspension_method");
			opts.attachment_suspension_thr = options->GetDouble("attachment_suspension_thr");
			opts.attachment_strain_method = options->GetStr("attachment_strain_method");
			opts.attachment_strain_thr = options->GetDouble("attachment_strain_thr");

			// 用户指定杆塔坐标
			std::vector<osg::Vec3d> positions;

			try
			{
				int doubleSize = options->GetInt("DoubleArraySize");
				void* doubleArrayData = options->GetData("DoubleArrayData");

				if (doubleSize > 0)
				{
					if (doubleSize % 3 == 0)
					{
						double* doubleArray = static_cast<double*>(doubleArrayData);
						int positionSize = doubleSize / 3;
						positions.resize(positionSize);


						for (int i = 0; i < positionSize; ++i)
						{
							// 读取地理坐标
							for (int j = 0; j < 3; ++j)
								positions[i][j] = doubleArray[i * 3 + j];

							// 地理坐标减去偏移量
							// positions[i] -= pcv->offset_xyz;
						}
					}
					else
					{
						PCS_WARN(
							"[Powerline Corridors Segmentation] 用户指定杆塔坐标数据格式错误.");
					}
				}
			}
			catch (std::exception& e)
			{
				PCS_WARN("[Powerline Corridors Segmentation] 找不到 'DoubleArraySize', 用户未指定杆塔坐标.");
			}


			opts.Print(); // 打印参数
			// PROGRESS(0.0, "正在进行铁塔、电力线分类...");

			if (!CloudNormalize(pointCloudView))
			{
				PCS_WARN("[Powerline Corridors Segmentation] 点云高度归一化出现错误.");
				return false;
			}

			// PROGRESS(30.0, "正在进行铁塔、电力线分类...");
			PowerCorridorsClassify classification(opts, &pointCloudView);
			classification.SetPylonPositions(positions);
			classification.SetIndices(_indices);

			// 执行分割线程
			classification.Start();
			classification.Wait();

			PCS_INFO("[Powerline Corridors Segmentation] 电力线分类用时 %.3f s.",
					 timer.ElapsedSeconds());

			// PROGRESS(100.0, "铁塔、电力线分类完成.");

			return true;
		}

		// CEnviromentSegmentation
		//////////////////////////////////////////////////////////////////////////
		CEnviromentSegmentation::~CEnviromentSegmentation() {}

		bool CEnviromentSegmentation::Segment(const IOptions* options, std::vector<PointId>& result)
		{
			CHECK(options);

			if (!options)
				return false;

			if (!_cloud.valid())
				return false;

			d3s::share_ptr<CPointCloud> cloud = dynamic_cast<CPointCloud*>(_cloud.get());
			CHECK(cloud.valid());

			PointCloudViewPtr pcv = cloud->Data();
			CHECK(pcv);

			auto& pointCloudView = *pcv;

			if (pointCloudView.empty())
				return false;

			Timer timer;
			timer.Start();

			EnviromentClassifyOptions opts;

			// 降噪参数
			opts.denoise = options->GetBool("denoise");
			opts.denoise_voxel_size = options->GetDouble("denoise_voxel_size");
			opts.denoise_min_pts = options->GetInt("denoise_min_pts");

			// 植被分类参数
			opts.cell_size = options->GetDouble("cell_size");
			opts.min_vegetation_height = options->GetDouble("min_vegetation_height");
			opts.min_high_vegetation_height = options->GetDouble("min_high_vegetation_height");

			// 建筑物分类参数
			opts.euclidean_cluster_distance = options->GetDouble("euclidean_cluster_distance");
			opts.euclidean_cluster_min_pts = options->GetInt("euclidean_cluster_min_pts");
			opts.planar_ransac_distance = options->GetDouble("planar_ransac_distance");
			opts.planar_inlier_pts_ratio = options->GetDouble("planar_inlier_pts_ratio");
			opts.min_building_area = options->GetDouble("min_building_area");
			opts.max_building_area = options->GetDouble("max_building_area");
			opts.min_building_height = options->GetDouble("min_building_height");

			// 道路分类参数
			opts.road_cell_size = options->GetDouble("road_cell_size");
			opts.road_height_threshold = options->GetDouble("road_height_threshold");
			opts.road_slope_threshold = options->GetDouble("road_slope_threshold");
			opts.road_min_area = options->GetDouble("road_min_area");
			opts.road_max_area = options->GetDouble("road_max_area");
			opts.road_shape_threshold = options->GetDouble("road_shape_threshold");
			opts.road_linearity_threshold = options->GetDouble("road_linearity_threshold");

			// 道理矢量数据
			IValueBuffer* roadVectorize =
				static_cast<IValueBuffer*>(options->GetData("RoadVectorize"));

			opts.Print(); // 打印参数
			// PROGRESS(0.0, "正在进行植被、建筑物、道路分类...");

			if (!CloudNormalize(pointCloudView))
			{
				PCS_WARN("[Enviroment Element Segmentation] 点云高度归一化出现错误.");
				return false;
			}

			// PROGRESS(30.0, "正在进行植被、建筑物、道路分类...");

			EnviromentClassify classifier(opts, &pointCloudView);
			classifier.SetIndices(_indices);
			classifier.SetRoadVectorize(roadVectorize);

			// 执行分割线程
			classifier.Start();
			classifier.Wait();

			PCS_INFO("[Enviroment Element Segmentation] 植被、建筑物、道路分类用时 %.3f s.",
					 timer.ElapsedSeconds());
			// PROGRESS(100.0, "植被、建筑物、道路分类完成.");

			return true;
		}

		// CCropSegmentation
		//////////////////////////////////////////////////////////////////////////
		CCropSegmentation::~CCropSegmentation() {}

		bool CCropSegmentation::Segment(const IOptions* options, std::vector<PointId>& result)
		{
			CHECK(options);

			if (!options)
				return false;

			if (!_cloud.valid())
				return false;

			d3s::share_ptr<CPointCloud> cloud = dynamic_cast<CPointCloud*>(_cloud.get());
			CHECK(cloud.valid());

			PointCloudViewPtr pcv = cloud->Data();
			CHECK(pcv);

			auto& pointCloudView = *pcv;

			if (pointCloudView.empty())
				return false;

			Timer timer;
			timer.Start();

			CropClassifyOptions opts;

			opts.min_height = options->GetDouble("min_height");
			opts.max_height = options->GetDouble("max_height");
			opts.min_area = options->GetDouble("min_area");
			opts.min_forest_area = options->GetDouble("min_forest_area");
			opts.pred_threshold = options->GetDouble("pred_threshold");
			opts.cache_dir = options->GetStr("cache_dir");
			opts.ncnn_param = options->GetStr("ncnn_param");
			opts.ncnn_model = options->GetStr("ncnn_model");

			auto area_map_ptr = (std::map<int, double>*)options->GetData("label_area_map");

			// 解析文件列表
			std::string file_token = options->GetStr("dom_files");
			opts.dom_files = StringSplit(file_token, ",");

			// PROGRESS(0.0, "正在进行农作物分类...");

			if (!CloudNormalize(pointCloudView))
			{
				PCS_WARN("[Crop Segmentation] 点云高度归一化出现错误.");
				return false;
			}

			opts.Print(); // 打印参数

			auto progressFunc = [&](double dComplete) {
				// PROGRESS(dComplete, "正在进行农作物分类...");
			};

			// 执行分割线程
			CropClassify classifier(opts, &pointCloudView, _indices, progressFunc);
			classifier.Start();
			classifier.Wait();

			PCS_INFO("[Crop Segmentation] 农作物分类用时 %.3f s.", timer.ElapsedSeconds());
			// PROGRESS(100.0, "农作物分类完成.");

			return true;
		}

		// CTreeIndividual
		//////////////////////////////////////////////////////////////////////////
		CTreeIndividual::CTreeIndividual(std::string method) : CCloudSegmentation(), _method(method)
		{
		}

		CTreeIndividual::~CTreeIndividual() {}

		bool CTreeIndividual::Segment(const IOptions* options, std::vector<PointId>& result)
		{
			CHECK(options);

			if (!options)
				return false;

			if (!_cloud.valid())
				return false;

			// PROGRESS(0.0, "正在进行单木分割...");
			d3s::share_ptr<CPointCloud> cloud = dynamic_cast<CPointCloud*>(_cloud.get());
			CHECK(cloud.valid());

			PointCloudViewPtr pcv = cloud->Data();
			CHECK(pcv);

			auto& pointCloudView = *pcv;

			if (pointCloudView.empty())
				return false;

			if (!CloudNormalize(pointCloudView))
			{
				PCS_WARN("[Individual Tree Segment] 点云高度归一化出现错误.");
				return false;
			}

			// PROGRESS(20.0, "正在进行单木分割...");
			TreeSegmentationOptions opts;

			// Dalponte2016 算法参数
			opts.green_color_style = options->GetBool("green_color_style");
			opts.min_pts = options->GetInt("min_pts");
			opts.min_height = options->GetDouble("min_height");
			opts.max_height = options->GetDouble("max_height");
			opts.min_radius = options->GetDouble("min_radius");
			opts.max_radius = options->GetDouble("max_radius");
			opts.max_crown = options->GetDouble("max_crown");
			opts.th_tree = options->GetDouble("th_tree");
			opts.th_seed = options->GetDouble("th_seed");
			opts.th_crown = options->GetDouble("th_crown");
			opts.resolution = options->GetDouble("resolution");

			// Li2012 算法参数
			opts.dt1 = options->GetDouble("dt1");
			opts.dt2 = options->GetDouble("dt2");
			opts.Zu = options->GetDouble("Zu");
			opts.R = options->GetDouble("R");
			opts.radius = options->GetDouble("radius");

			opts.Print(); // 打印参数

			if (_indices.empty())
			{
				getClassification(pointCloudView, eHighVegetation, _indices);
			}

			// PROGRESS(30.0, "正在进行单木分割...");

			// 执行分割线程
			if (_method == "Dalponte2016")
			{
				Dalponte2016 segment(opts, &pointCloudView, _indices);
				segment.Start();
				segment.Wait();
			}
			else if (_method == "Li2012")
			{
				Li2012 segment(opts, &pointCloudView, _indices);
				segment.Start();
				segment.Wait();
			}
			else
			{
				PCS_WARN("[Individual Tree Segment] 无效的单木分割算法.");
			}


			// PROGRESS(100.0, "单木分割完成.");
			return true;
		}


		// CTreeSpeciesIdentify
		//////////////////////////////////////////////////////////////////////////
		CTreeSpeciesIdentify::CTreeSpeciesIdentify() : CCloudSegmentation() {}

		CTreeSpeciesIdentify::~CTreeSpeciesIdentify() {}

		bool CTreeSpeciesIdentify::Segment(const IOptions* options, std::vector<PointId>& result)
		{
			CHECK(options);

			if (!options)
				return false;

			if (!_cloud.valid())
				return false;

			d3s::share_ptr<CPointCloud> cloud = dynamic_cast<CPointCloud*>(_cloud.get());
			CHECK(cloud.valid());

			PointCloudViewPtr pcv = cloud->Data();
			CHECK(pcv);

			auto& pointCloudView = *pcv;
			if (pointCloudView.empty())
				return false;

			Timer timer;
			timer.Start();
			// PROGRESS(0.0, "正在进行树种识别...");

			TreeSpeciesIndendtifyOptions opts;
			opts.cache_dir = options->GetStr("cache_dir");
			opts.ncnn_param = options->GetStr("ncnn_param");
			opts.ncnn_model = options->GetStr("ncnn_model");
			opts.num_classes = options->GetInt("num_classes");

			// 解析文件列表
			std::string file_token = options->GetStr("dom_files");
			opts.dom_files = StringSplit(file_token, ",");

			opts.Print(); // 打印参数

			auto progressFunc = [&](double dComplete) {
				// PROGRESS(dComplete, "正在进行树种识别...");
			};

			TreeSpeciesIdentify identify(opts, &pointCloudView, _indices, progressFunc);
			// 执行分割线程
			identify.Start();
			identify.Wait();

			PCS_INFO("[Tree Species Identification] 树种识别用时 %.3f s.", timer.ElapsedSeconds());
			// PROGRESS(100.0, "树种识别完成.");
			return true;
		}


		// CDangerSegmentation
		//////////////////////////////////////////////////////////////////////////
		CDangerSegmentation::CDangerSegmentation() : CCloudSegmentation() {}

		CDangerSegmentation::~CDangerSegmentation() {}

		bool CDangerSegmentation::Segment(const IOptions* options, std::vector<PointId>& result)
		{
			CHECK(options);

			if (!options)
				return false;

			if (!_cloud.valid())
				return false;

			d3s::share_ptr<CPointCloud> cloud = dynamic_cast<CPointCloud*>(_cloud.get());
			CHECK(cloud.valid());

			PointCloudViewPtr pcv = cloud->Data();
			CHECK(pcv);

			auto& pointCloudView = *pcv;
			if (pointCloudView.empty())
				return false;

			Timer timer;
			timer.Start();
			// PROGRESS(0.0, "正在进行危险区域识别...");

			// 初始化渲染颜色映射
			ColorManager::debugTags.resize(pointCloudView.size(), -1);
			ColorManager::debugColorMap.clear();
			ColorManager::debugColorMap[0] = osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f);		// 红
			ColorManager::debugColorMap[1] = osg::Vec4(1.0f, 0.647f, 0.0f, 1.0f);	// 橙
			ColorManager::debugColorMap[2] = osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f);		// 黄
			ColorManager::debugColorMap[3] = osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f);		// 绿
			ColorManager::debugColorMap[4] = osg::Vec4(0.0f, 1.0f, 1.0f, 1.0f);		// 青
			ColorManager::debugColorMap[5] = osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f);		// 蓝
			ColorManager::debugColorMap[6] = osg::Vec4(0.502f, 0.0f, 0.502f, 1.0f); // 紫

			// PROGRESS(50.0, "正在进行危险区域识别...");

			// 1. 陡坡识别
			SteepSlopeOptions steep_slope_opts;
			steep_slope_opts.resolution = options->GetDouble("steep_slope_resolution");
			steep_slope_opts.degree_threshold = options->GetDouble("steep_slope_degree_threshold");
			steep_slope_opts.area_threshold = options->GetDouble("steep_slope_area_threshold");
			steep_slope_opts.Print();

			SteepSlopeSegmentation steep_slope_segment(steep_slope_opts, &pointCloudView, _indices);
			steep_slope_segment.Start();
			steep_slope_segment.Wait();

			// 2. 开挖识别
			ExcavationOptions excavatio_opts;
			excavatio_opts.resolution = options->GetDouble("excavation_resolution");
			excavatio_opts.degree_threshold = options->GetDouble("excavation_degree_threshold");
			excavatio_opts.hag_threshold = options->GetDouble("excavation_hag_threshold");
			excavatio_opts.smooth_threshold = options->GetDouble("excavation_smooth_threhold");
			excavatio_opts.area_threshold = options->GetDouble("excavation_area_threshold");
			excavatio_opts.Print();

			ExcavationSegmentation excavation_segment(excavatio_opts, &pointCloudView, _indices);
			excavation_segment.Start();
			excavation_segment.Wait();

			DomSegmentOptions dom_opts;
			dom_opts.dom_file_path = options->GetStr("dom_file_path");
			dom_opts.road_segment_param = options->GetStr("dom_road_segment_param");
			dom_opts.road_segment_bin = options->GetStr("dom_road_segment_model");
			dom_opts.building_segment_param = options->GetStr("dom_building_segment_param");
			dom_opts.building_segment_bin = options->GetStr("dom_building_segment_model");
			dom_opts.water_segment_param = options->GetStr("dom_water_segment_param");
			dom_opts.water_segment_bin = options->GetStr("dom_water_segment_model");

			DomSegmentation dom_segment(dom_opts, &pointCloudView, _indices);
			dom_segment.Start();
			dom_segment.Wait();


			PCS_INFO("[Danger region segmentation] 危险区域识别用时 %.3f s.",
					 timer.ElapsedSeconds());
			// PROGRESS(100.0, "危险区域识别完成.");
			return true;
		}

		// CPylonDetection
		//////////////////////////////////////////////////////////////////////////
		CPylonDetection::~CPylonDetection() {}

		bool CPylonDetection::DetectPositions(const IOptions* options,
											  std::vector<osg::Vec3d>& positions)
		{
			CHECK(options);

			if (!options)
				return false;

			if (!_cloud.valid())
				return false;

			d3s::share_ptr<CPointCloud> cloud = dynamic_cast<CPointCloud*>(_cloud.get());
			CHECK(cloud.valid());

			PointCloudViewPtr pcv = cloud->Data();
			CHECK(pcv);

			auto& pointCloudView = *pcv;

			if (pointCloudView.empty())
				return false;

			Timer timer;
			timer.Start();

			PylonLocationOptions opts;

			// 杆塔定位参数
			opts.location_cellsize = options->GetDouble("location_cellsize");
			opts.location_zstep = options->GetDouble("location_zstep");
			opts.location_th1 = options->GetDouble("location_th1");
			opts.location_th2 = options->GetDouble("location_th2");
			opts.location_tv = options->GetDouble("location_tv");
			opts.location_tc = options->GetDouble("location_tc");
			opts.location_th = options->GetDouble("location_th");
			opts.location_outlier_coeff = options->GetDouble("location_outlier_coeff");
			opts.location_linearity = options->GetDouble("location_linearity");
			opts.location_min_proj = options->GetDouble("location_min_proj");
			opts.location_max_span = options->GetDouble("location_max_span");
			opts.location_corridor_width = options->GetDouble("location_corridor_width");
			opts.location_neighbors = options->GetInt("location_neighbors");

			opts.max_span_error = options->GetInt("max_span_error");
			opts.min_span_angle_theta = options->GetDouble("min_span_angle_theta");

			PCS_INFO("location_cellsize: %lf", opts.location_cellsize);
			PCS_INFO("location_zstep: %lf", opts.location_zstep);
			PCS_INFO("location_th1: %lf", opts.location_th1);
			PCS_INFO("location_th2: %lf", opts.location_th2);
			PCS_INFO("location_tv: %lf", opts.location_tv);
			PCS_INFO("location_tc: %lf", opts.location_tc);
			PCS_INFO("location_th: %lf", opts.location_th);
			PCS_INFO("location_outlier_coeff: %lf", opts.location_outlier_coeff);
			PCS_INFO("location_linearity: %lf", opts.location_linearity);
			PCS_INFO("location_min_proj: %lf", opts.location_min_proj);
			PCS_INFO("location_max_span: %lf", opts.location_max_span);
			PCS_INFO("location_corridor_width: %lf", opts.location_corridor_width);
			PCS_INFO("location_neighbors: %d", opts.location_neighbors);

			// PROGRESS(0.0, "正在进行杆塔自动定位...");

			if (!CloudNormalize(pointCloudView))
			{
				PCS_WARN("[Pylon Detection] 点云高度归一化出现错误.");
				return false;
			}

			// 兼容空索引
			if (_indices.empty())
			{
				_indices.reserve(pointCloudView.size());

				for (size_t i = 0; i < pointCloudView.size(); ++i)
				{
					const auto& p = pointCloudView.points[i];

					if (p.label != eGround && p.hag > 0)
						_indices.push_back(i);
				}

				_indices.shrink_to_fit();
			}

			// PROGRESS(30.0, "正在进行杆塔自动定位...");

			PylonLocation locator(opts, &pointCloudView, _indices);
			locator.Location(positions);

			PCS_INFO("[Pylon Detection] 杆塔自动定位用时 %.3f s.", timer.ElapsedSeconds());

			// PROGRESS(100.0, "杆塔自动定位完成.");

			return true;
		}

		// CCloudClusteringImpl
		//////////////////////////////////////////////////////////////////////////
		CCloudClusteringImpl::~CCloudClusteringImpl() {}

		void CCloudClusteringImpl::EuclideanCluster(double tolerance,
													int minPts,
													int maxPts,
													Clusters& clusters)
		{
			if (!_cloud.valid())
				return;

			d3s::share_ptr<CPointCloud> cloud = dynamic_cast<CPointCloud*>(_cloud.get());
			CHECK(cloud.valid());

			PointCloudViewPtr pcv = cloud->Data();
			CHECK(pcv);

			auto& pointCloudView = *pcv;

			if (pointCloudView.empty())
				return;

			Timer timer;
			timer.Start();

			// PROGRESS(0.0, "正在进行点云聚类...");

			// 兼容空索引
			if (_indices.empty())
			{
				_indices.resize(pointCloudView.size());
				std::iota(_indices.begin(), _indices.end(), 0);
			}

			// PROGRESS(30.0, "正在进行点云聚类...");

			euclideanClusterSafe(pointCloudView, _indices, tolerance, minPts, maxPts, clusters);

			PCS_INFO("[Cloud Clustering] 点云聚类用时 %.3f s.", timer.ElapsedSeconds());

			// PROGRESS(100.0, "点云聚类完成.");
		}

		void CCloudClusteringImpl::EuclideanClusterFast(double tolerance,
														int minPts,
														int maxPts,
														Clusters& clusters)
		{
			if (!_cloud.valid())
				return;

			d3s::share_ptr<CPointCloud> cloud = dynamic_cast<CPointCloud*>(_cloud.get());
			CHECK(cloud.valid());

			PointCloudViewPtr pcv = cloud->Data();
			CHECK(pcv);

			auto& pointCloudView = *pcv;

			if (pointCloudView.empty())
				return;

			Timer timer;
			timer.Start();

			// PROGRESS(0.0, "正在进行点云聚类...");

			// 兼容空索引
			if (_indices.empty())
			{
				_indices.resize(pointCloudView.size());
				std::iota(_indices.begin(), _indices.end(), 0);
			}

			// PROGRESS(30.0, "正在进行点云聚类...");

			EuclideanVoxelCluster(&pointCloudView, _indices, tolerance, minPts, maxPts, clusters);

			PCS_INFO("[Cloud Clustering] 点云聚类用时 %.3f s.", timer.ElapsedSeconds());

			// PROGRESS(100.0, "点云聚类完成.");
		}


		// CTiffRasterize
		//////////////////////////////////////////////////////////////////////////
		CTiffRasterize::~CTiffRasterize() {}

		void CTiffRasterize::WriteTiff(const std::string& filepath,
									   const std::string& srs,
									   double resolution)
		{
			if (!_cloud.valid())
				return;

			d3s::share_ptr<CPointCloud> cloud = dynamic_cast<CPointCloud*>(_cloud.get());
			CHECK(cloud.valid());

			PointCloudViewPtr pcv = cloud->Data();
			CHECK(pcv);

			auto& pointCloudView = *pcv;

			if (pointCloudView.empty())
				return;

			Timer timer;
			timer.Start();

			// PROGRESS(0.0, "正在进行点云栅格化...");

			// 兼容空索引
			if (_indices.empty())
			{
				_indices.resize(pointCloudView.size());
				std::iota(_indices.begin(), _indices.end(), 0);
			}

			// PROGRESS(30.0, "正在进行点云栅格化...");

			// 计算包围框
			osg::BoundingBox bbox;
			computeMinMax3D(pointCloudView, _indices, bbox);

			Rasterd raster, mask;
			PointCloudView<PointPCLH>::PointCloud points;

			for (size_t i = 0; i < _indices.size(); ++i)
			{
				const auto& p = pointCloudView.points[_indices[i]];
				points.push_back(p);
			}

			// 生成掩码
			createBinaryMask(pointCloudView, _indices, bbox, resolution, mask);

			// 1px = 1m
			createCloudRaster(points, bbox, resolution, raster);

			int width = mask.width();
			int height = mask.height();
			double nodata = std::numeric_limits<double>::quiet_NaN();

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (mask.at(c, r) == 0.0)
						raster.at(c, r) = nodata;
				}
			}

			// 写入到本地
			writeTiff(raster, filepath, srs);

			PCS_INFO("[Cloud Tiff Raster] 点云栅格化用时 %.3f s.", timer.ElapsedSeconds());

			// PROGRESS(100.0, "点云栅格化完成.");
		}

		// CPowerlineCurveFitting
		//////////////////////////////////////////////////////////////////////////
		CPowerlineCurveFitting::~CPowerlineCurveFitting() {}

		bool CPowerlineCurveFitting::CurveFitting(double expandLength, PARACURVE& params)
		{
			if (!_cloud.valid())
				return false;

			d3s::share_ptr<CPointCloud> cloud = dynamic_cast<CPointCloud*>(_cloud.get());
			CHECK(cloud.valid());

			PointCloudViewPtr pcv = cloud->Data();
			CHECK(pcv);

			auto& pointCloudView = *pcv;

			if (pointCloudView.empty())
				return false;

			Timer timer;
			timer.Start();

			// PROGRESS(0.0, "正在进行电力线拟合...");

			// 兼容空索引
			if (_indices.empty())
			{
				_indices.reserve(pointCloudView.size());

				for (size_t i = 0; i < pointCloudView.size(); ++i)
				{
					const auto& p = pointCloudView.points[i];
					_indices.push_back(i);
				}

				_indices.shrink_to_fit();
			}

			// PROGRESS(30.0, "正在进行电力线拟合...");

			auto curve = PowerlineCurveFitting(&pointCloudView,
											   _indices,
											   expandLength,
											   osg::Vec4(1.f, 1.f, 1.f, 1.f)
#ifdef _render_Powerline_Cluster
												   ,
											   true
#endif
			);

			// 专递曲线参数
			params.rotate = curve->rotate;
			params.origin = curve->origin;
			params.xmean = curve->xmean;
			params.ymin = curve->ymin;
			params.ymax = curve->ymax;
			params.a = curve->a;
			params.b = curve->b;
			params.c = curve->c;

			PCS_INFO("[Cloud Clustering] 电力线拟合用时 %.3f s.", timer.ElapsedSeconds());

			// PROGRESS(100.0, "电力线拟合完成.");

			return true;
		}

		// CPylonReconstruct
		//////////////////////////////////////////////////////////////////////////
		CPylonReconstruct::~CPylonReconstruct() {}

		bool CPylonReconstruct::Reconstruct(const IOptions* options)
		{
			if (!_cloud.valid())
				return false;

			d3s::share_ptr<CPointCloud> cloud = dynamic_cast<CPointCloud*>(_cloud.get());
			CHECK(cloud.valid());

			PointCloudViewPtr pcv = cloud->Data();
			CHECK(pcv);

			auto& pointCloudView = *pcv;

			if (pointCloudView.empty())
				return false;

			Timer timer;
			timer.Start();

			PylonReconstructOptions opts;
			opts.decompose_head_len = options->GetDouble("decompose_head_len");
			opts.decompose_z_step = options->GetDouble("decompose_z_step");
			opts.decompose_x_step = options->GetDouble("decompose_x_step");
			opts.decompose_windows_size = options->GetDouble("decompose_windows_size");
			opts.decompose_fillrate = options->GetDouble("decompose_fillrate");
			opts.decompose_s1_ratio = options->GetDouble("decompose_s1_ratio");
			opts.decompose_aspect_error = options->GetDouble("decompose_aspect_error");
			opts.matching_rms_threshold = options->GetDouble("matching_rms_threshold");
			opts.matching_bbox_threshold = options->GetDouble("matching_bbox_threshold");
			opts.matching_library_folder = options->GetStr("matching_library_folder");
			opts.matching_num_registration = options->GetInt("matching_num_registration");

			opts.Print(); // 打印参数
			// PROGRESS(0.0, "正在进行铁塔点云重建...");

			GenericMesh* mesh = static_cast<GenericMesh*>(options->GetData("MeshData"));

			// PROGRESS(30.0, "正在进行铁塔点云重建...");

			PylonReconstruct recon(opts, &pointCloudView, mesh);
			recon.Reconstruct();

			// PROGRESS(100.0, "铁塔点云重建完成.");

			return true;
		}


	}
}