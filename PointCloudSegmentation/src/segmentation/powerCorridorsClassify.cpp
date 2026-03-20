#include <src/segmentation/powerCorridorsClassify.h>
#include <src/segmentation/gridCell.h>
#include <src/segmentation/pylonLocation.h>

#include <src/segmentation/improve/poleClassifier.h>
#include <src/segmentation/improve/pylonClassifier.h>
#include <src/segmentation/improve/wireClassifier.h>
#include <src/segmentation/improve/powerlineClassifier.h>
#include <src/segmentation/improve/pylonRefinement.h>
#include <src/segmentation/improve/attachmentClassifier.h>

#include <src/algorithm/geometry2d_op.h>
#include <src/core/private/statistics.h>
#include <src/core/private/rasterProcess.h>
#include <src/core/private/cloudProcess.h>
#include <src/io/rasterWriter.h>
#include <src/plot/geomCreator.h>
#include <src/plot/plotHandle.h>


#include <osg/Geode>
#include <osg/AnimationPath>
#include <osg/MatrixTransform>


// #define _PROCESS_CELL_USE_HAG

extern std::string DebugDirectory;

namespace d3s {
	namespace pcs {

		// PowerCorridorsClassifyOptions
		//////////////////////////////////////////////////////////////////////////
		void PowerCorridorsClassifyOptions::Print() const
		{
			PCS_INFO("max_span_error: %d", max_span_error);
			PCS_INFO("min_span_angle_theta: %lf", min_span_angle_theta);
			PCS_INFO("min_body_growing_height: %lf", min_body_growing_height);
			PCS_INFO("max_body_growing_height: %lf", max_body_growing_height);
			PCS_INFO("body_growing_step: %lf", body_growing_step);
			PCS_INFO("body_growing_resolition: %lf", body_growing_resolition);
			PCS_INFO("max_curve_dist: %lf", max_curve_dist);

			PCS_INFO("distribution: %s", distribution ? "true" : "false");

			// 改进算法参数
			PCS_INFO("location_cellsize: %lf", location_cellsize);
			PCS_INFO("location_zstep: %lf", location_zstep);
			PCS_INFO("location_th1: %lf", location_th1);
			PCS_INFO("location_th2: %lf", location_th2);
			PCS_INFO("location_tv: %lf", location_tv);
			PCS_INFO("location_tc: %lf", location_tc);
			PCS_INFO("location_th: %lf", location_th);
			PCS_INFO("location_outlier_coeff: %lf", location_outlier_coeff);
			PCS_INFO("location_linearity: %lf", location_linearity);
			PCS_INFO("location_min_proj: %lf", location_min_proj);
			PCS_INFO("location_max_span: %lf", location_max_span);
			PCS_INFO("location_corridor_width: %lf", location_corridor_width);
			PCS_INFO("location_neighbors: %d", location_neighbors);

			PCS_INFO("pylon_head_length: %lf", pylon_head_length);
			PCS_INFO("pylon_d1_scale: %lf", pylon_d1_scale);
			PCS_INFO("pylon_d2_scale: %lf", pylon_d2_scale);
			PCS_INFO("pylon_terminal_scale: %lf", pylon_terminal_scale);
			PCS_INFO("pylon_corner_scale: %lf", pylon_corner_scale);

			PCS_INFO("powerline_slice_step: %lf", powerline_slice_step);
			PCS_INFO("powerline_slice_thickness: %lf", powerline_slice_thickness);
			PCS_INFO("powerline_cluster_th: %lf", powerline_cluster_th);
			PCS_INFO("powerline_cluster_radius: %lf", powerline_cluster_radius);
			PCS_INFO("powerline_cluster_min_points: %d", powerline_cluster_min_points);
			PCS_INFO("powerline_curve_min_points: %d", powerline_curve_min_points);
			PCS_INFO("powerline_curve_expand_length: %lf", powerline_curve_expand_length);
			PCS_INFO("powerline_curve_fit_fast: %s", powerline_curve_fit_fast ? "true" : "false");

			PCS_INFO("refine_cluster_radius: %lf", refine_cluster_radius);
			PCS_INFO("refine_cluster_min_points: %d", refine_cluster_min_points);
			PCS_INFO("refine_curve_fit_fast: %s", refine_curve_fit_fast ? "true" : "false");

			PCS_INFO("attachment_enable: %s", attachment_enable ? "true" : "false");
			PCS_INFO("attachment_voxel_size: %lf", attachment_voxel_size);
			PCS_INFO("attachment_search_radius: %lf", attachment_search_radius);
			PCS_INFO("attachment_x_length: %lf", attachment_x_length);
			PCS_INFO("attachment_y_length: %lf", attachment_y_length);
			PCS_INFO("attachment_z_length: %lf", attachment_z_length);
			PCS_INFO("attachment_x_fillrate: %lf", attachment_x_fillrate);
			PCS_INFO("attachment_y_fillrate: %lf", attachment_y_fillrate);
			PCS_INFO("attachment_xy_fillrate: %lf", attachment_xy_fillrate);
			PCS_INFO("attachment_xz_fillrate: %lf", attachment_xz_fillrate);
			PCS_INFO("attachment_leadwire_x_fr: %lf", attachment_leadwire_x_fr);
			PCS_INFO("attachment_leadwire_x_len: %lf", attachment_leadwire_x_len);
			PCS_INFO("attachment_suspension_method: %s", attachment_suspension_method.c_str());
			PCS_INFO("attachment_suspension_thr: %lf", attachment_suspension_thr);
			PCS_INFO("attachment_strain_method: %s", attachment_strain_method.c_str());
			PCS_INFO("attachment_strain_thr: %lf", attachment_strain_thr);
		}

		// PowerCorridorsClassify
		//////////////////////////////////////////////////////////////////////////
		PowerCorridorsClassify::PowerCorridorsClassify(const PowerCorridorsClassifyOptions& options,
													   PointCloudViewPtr input)
			: _options(options), _input(input), _grid(new Grid2D())
		{
		}

		PowerCorridorsClassify::~PowerCorridorsClassify() {}

		void PowerCorridorsClassify::SetIndices(const std::vector<int>& indices)
		{
			_indices = indices;
		}

		void PowerCorridorsClassify::SetPylonPositions(const std::vector<osg::Vec3d>& positions)
		{
			_positions = positions;
		}

		Grid2D PowerCorridorsClassify::GetGrid() const { return *_grid; }

		void PowerCorridorsClassify::Run()
		{
			// 兼容空索引
			if (_indices.empty())
			{
				_indices.reserve(_input->size());

				for (size_t i = 0; i < _input->size(); ++i)
				{
					const auto& p = _input->points[i];

					if (p.label != eGround && p.hag > 0)
						_indices.push_back(i);
				}

				_indices.shrink_to_fit();
			}

			if (_indices.empty())
				return;
			
			{
				// 杆塔定位
				std::vector<osg::Vec3d> positions;
				std::vector<osg::Vec3d> sides;
				std::vector<std::vector<osg::Vec3d>> polygons;
				PylonConnections connections;

				if (_positions.empty())
				{
					// 未指定杆塔坐标，自定定位杆塔
					PylonLocationOptions options0;

					options0.location_cellsize = _options.location_cellsize;
					options0.location_zstep = _options.location_zstep;
					options0.location_th1 = _options.location_th1;
					options0.location_th2 = _options.location_th2;
					options0.location_tv = _options.location_tv;
					options0.location_tc = _options.location_tc;
					options0.location_th = _options.location_th;
					options0.location_outlier_coeff = _options.location_outlier_coeff;
					options0.location_linearity = _options.location_linearity;
					options0.location_min_proj = _options.location_min_proj;
					options0.location_max_span = _options.location_max_span;
					options0.location_corridor_width = _options.location_corridor_width;
					options0.location_neighbors = _options.location_neighbors;

					options0.max_span_error = _options.max_span_error;
					options0.min_span_angle_theta = _options.min_span_angle_theta;

					PylonLocation locator(options0, _input, _indices);
					locator.Location(positions);
				}
				else
				{
					positions = _positions;

					for (size_t i = 0; i < positions.size(); ++i)
					{
						const auto& pos = positions.at(i);
						PCS_INFO("杆塔 %d 坐标: (%lf, %lf, %lf)", i, pos.x(), pos.y(), pos.z());
					}
				}

				if (positions.size() > 1)
				{
					if (_options.distribution)
					{
						// 配网电杆提取
						PoleClassifyOptions options1;
						options1.height_threshold = _options.location_th1;
						options1.head_length = _options.pylon_head_length;
						options1.d1_scale = _options.pylon_d1_scale;
						options1.d2_scale = _options.pylon_d2_scale;

						PoleClassifier poleClassifier(options1, _input, _indices);
						poleClassifier.SetPolePositions(positions);
						poleClassifier.Segment();

						positions = poleClassifier.GetPolePositions();
						polygons = poleClassifier.GetPolePolygons();
						sides = poleClassifier.GetPoleSides();
					}
					else
					{
						// 铁塔提取，边界计算
						PylonClassifyOptions options1;
						options1.cell_size = _options.location_cellsize;
						options1.num_neighbors = _options.location_neighbors;
						options1.height_threshold = _options.location_th1;

						options1.min_body_growing_height = _options.min_body_growing_height;
						options1.max_body_growing_height = _options.max_body_growing_height;
						options1.body_growing_step = _options.body_growing_step;
						options1.body_growing_resolition = _options.body_growing_resolition;

						options1.head_length = _options.pylon_head_length;
						options1.d1_scale = _options.pylon_d1_scale;
						options1.d2_scale = _options.pylon_d2_scale;
						options1.terminal_scale = _options.pylon_terminal_scale;
						options1.corner_scale = _options.pylon_corner_scale;

						PylonClassifier pylonClassifier(options1, _input, _indices);
						pylonClassifier.SetPylonPositions(positions);
						pylonClassifier.Segment();

						positions = pylonClassifier.GetPylonPositions();
						polygons = pylonClassifier.GetPylonPolygons();
						sides = pylonClassifier.GetPylonSides();
					}
				}

				if (positions.size() > 1)
				{
					if (_options.distribution)
					{
						// 配电线路导线提取
						WireClassifyOptions options2;
						options2.slice_step = _options.powerline_slice_step;
						options2.slice_thickness = _options.powerline_slice_thickness;
						options2.cluster_th = _options.powerline_cluster_th;
						options2.cluster_radius = _options.powerline_cluster_radius;
						options2.cluster_min_points = _options.powerline_cluster_min_points;
						options2.curve_min_points = _options.powerline_curve_min_points;
						options2.curve_expand_length = _options.powerline_curve_expand_length;
						options2.curve_fit_fast_clustering_enable =
							_options.powerline_curve_fit_fast;

						WireClassifier wireClassifier(options2, _input, _indices);
						wireClassifier.SetPolePositions(positions);
						wireClassifier.SetPolePolygons(polygons);
						wireClassifier.SetPoleSides(sides);
						wireClassifier.Segment();
					}
					else
					{
						// 电力线提取
						PowerlineClassifyOptions options2;
						options2.slice_step = _options.powerline_slice_step;
						options2.slice_thickness = _options.powerline_slice_thickness;
						options2.cluster_th = _options.powerline_cluster_th;
						options2.cluster_radius = _options.powerline_cluster_radius;
						options2.cluster_min_points = _options.powerline_cluster_min_points;
						options2.curve_min_points = _options.powerline_curve_min_points;
						options2.curve_expand_length = _options.powerline_curve_expand_length;
						options2.curve_fit_fast_clustering_enable =
							_options.powerline_curve_fit_fast;


						PowerlineClassifier powerlineClassifier(options2, _input, _indices);
						powerlineClassifier.SetPylonPositions(positions);
						powerlineClassifier.SetPylonPolygons(polygons);
						powerlineClassifier.SetPylonSides(sides);
						powerlineClassifier.Segment();

						// 杆塔细化
						PylonRefinementOptions options3;
						options3.curve_fit_fast_clustering_enable = _options.refine_curve_fit_fast;
						options3.cluster_min_points = _options.refine_cluster_min_points;
						options3.cluster_radius = _options.refine_cluster_radius;
						options3.connect_radius = _options.refine_connect_radius;
						options3.part_step = _options.powerline_slice_step;
						options3.part_width = _options.powerline_slice_thickness;
						options3.curve_expand = _options.powerline_curve_expand_length;
						options3.max_curve_dist = _options.max_curve_dist;

						PylonRefinement pylonRefinement(options3, _input, _indices);
						pylonRefinement.SetPylonPositions(positions);
						pylonRefinement.SetPylonPolygons(polygons);
						pylonRefinement.SetPylonSides(sides);
						pylonRefinement.Refine();

						connections = pylonRefinement.GetConnections();
					}
				}

				if (connections.size() > 1)
				{
					// 绝缘子分类
					AttachmentClassifyOptions options4;
					options4.head_cellsize = _options.location_cellsize;
					options4.head_length = _options.pylon_head_length;

					options4.enable = _options.attachment_enable;
					options4.voxel_size = _options.attachment_voxel_size;
					options4.search_radius = _options.attachment_search_radius;
					options4.x_length = _options.attachment_x_length;
					options4.y_length = _options.attachment_y_length;
					options4.z_length = _options.attachment_z_length;
					options4.x_fillrate = _options.attachment_x_fillrate;
					options4.y_fillrate = _options.attachment_y_fillrate;
					options4.xy_fillrate = _options.attachment_xy_fillrate;
					options4.xz_fillrate = _options.attachment_xz_fillrate;
					options4.leadwire_x_fr = _options.attachment_leadwire_x_fr;
					options4.leadwire_x_len = _options.attachment_leadwire_x_len;
					options4.suspension_method = _options.attachment_suspension_method;
					options4.suspension_thr = _options.attachment_suspension_thr;
					options4.strain_method = _options.attachment_strain_method;
					options4.strain_thr = _options.attachment_strain_thr;

					AttachmentClassifier insulatorClassifier(options4, _input, _indices);
					insulatorClassifier.SetPylonPositions(positions);
					insulatorClassifier.SetPylonPolygons(polygons);
					insulatorClassifier.SetPylonSides(sides);
					insulatorClassifier.SetPylonConnections(connections);

					insulatorClassifier.Segment();
				}
			}
		}
	}
}