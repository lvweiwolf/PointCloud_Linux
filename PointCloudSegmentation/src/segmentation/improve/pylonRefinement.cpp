//stdafx.h
#include "pylonRefinement.h"

#include <mutex>

#include "../../algorithm/geometry2d_op.h"
#include "../../algorithm/math.h"
#include "../../core/api.h"
#include "../../core/private/cloudProcess.h"
#include "../../core/private/rasterProcess.h"
#include "../../core/private/statistics.h"

#include "../../segmentation/gridCell.h"

#include "../../plot/plotHandle.h"

#include "pylonClassifier.h"
//#define RENDER_PART_OF_SPAN_BOUND
//#define RENDER_FITTING_CURVE
//#define RENDER_CONNECT_POSITIONS

namespace d3s {
	namespace pcs {

		PylonRefinement::PylonRefinement(const PylonRefinementOptions& options,
										 PointCloudViewPtr input,
										 const std::vector<int>& indices)
			: _options(options), _input(input), _indices(indices)
		{
		}

		PylonRefinement::~PylonRefinement() {}

		PylonConnections PylonRefinement::GetConnections() const { return _pylonConnections; }

		void PylonRefinement::SetPylonPositions(const Positions& pylonPositions)
		{
			_pylonPositions = pylonPositions;
		}

		void PylonRefinement::SetPylonPolygons(const Polygons& pylonPolygons)
		{
			_pylonPolygons = pylonPolygons;
		}

		void PylonRefinement::SetPylonSides(const Vectors& pylonSides) { _pylonSides = pylonSides; }

		void PylonRefinement::Refine()
		{
			if (_pylonPositions.size() < 2)
				return;

			double T_s = 1.2; // 杆塔边界范围缩放，使导线的纠正更稳定
			double step = _options.part_step;
			double width = _options.part_width;
			double expand = _options.curve_expand;
			double connect_threshold = _options.connect_radius;

			const Positions& pylonPositions = _pylonPositions;
			const Polygons& pylonPolygons = _pylonPolygons;
			const Vectors& pylonSides = _pylonSides;

			PylonConnections& pylonConnections = _pylonConnections;

			Grid2D grid;
			ComputeLocalGridCells(5.0, _input, _indices, grid);
			pylonConnections.resize(pylonPositions.size());

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)pylonPositions.size(); ++i)
			{
				std::vector<int> prevIndices, nextIndices, refineIndices;

				// 获得带细化的杆塔点云
				const auto& pos = pylonPositions.at(i);
				const auto& polygon = pylonPolygons.at(i);
				const auto& side = pylonSides.at(i);

				osg::BoundingBox bbox = getPolygonBound(polygon);

				for (size_t j = 0; j < grid.size(); ++j)
				{
					const auto& cell = grid.at(j);

					if (cell.GetSize() <= 0)
						continue;

					double x = bbox.center().x();
					double y = bbox.center().y();
					double cx = cell.center().x();
					double cy = cell.center().y();
					double radius = bbox.radius() * T_s;
					double distance = sqrt((cx - x) * (cx - x) + (cy - y) * (cy - y));

					if (distance > 3.0 * radius)
						continue;

					const auto& cellIndices = cell.GetIndices();

					for (size_t k = 0; k < cellIndices.size(); ++k)
					{
						const auto& p = _input->points[cellIndices[k]];
						distance = sqrt((p.x - x) * (p.x - x) + (p.y - y) * (p.y - y));

						if (distance > radius) // 超过半径范围
							continue;

						refineIndices.push_back(cellIndices[k]);
					}
				}

				// 获得杆塔前后局部的电力线点云
				if (i == 0)
				{
					prevIndices = GetPowerlinesOutOfSpan(grid, i, i + 1, 100.0, 1000.0);
					nextIndices = GetPowerlinesInSpan(grid, i, i + 1, step, width);
				}
				else if (i == pylonPositions.size() - 1)
				{
					prevIndices = GetPowerlinesInSpan(grid, i, i - 1, step, width);
					nextIndices = GetPowerlinesOutOfSpan(grid, i, i - 1, 100.0, 1000.0);
				}
				else
				{
					prevIndices = GetPowerlinesInSpan(grid, i, i - 1, step, width);
					nextIndices = GetPowerlinesInSpan(grid, i, i + 1, step, width);
				}

				// 局部电力线聚类，并进行曲线拟合. 拟合后补全杆塔边界范围内的电力线点.
				double height = std::min((polygon[0] - polygon[1]).length(),
										 (polygon[0] - polygon[3]).length());
				double expand = height;

				PowerlineSpan backwardSpan = ComputeApproxSpan(prevIndices, pos, side, expand);
				PowerlineSpan forwardSpan = ComputeApproxSpan(nextIndices, pos, side, expand);

				// 补全电力线
				{
					std::vector<int> indices = CorrectPylonInSpan(pos, forwardSpan, refineIndices);
					setClassification(ePowerline, indices, *_input);
				}

				{
					std::vector<int> indices = CorrectPylonInSpan(pos, backwardSpan, refineIndices);
					setClassification(ePowerline, indices, *_input);
				}

				// 计算杆塔对应挂点位置
				PylonConnection connection;

				if (!backwardSpan.empty() && !forwardSpan.empty())
				{
					std::vector<int> pylonIndices, pylonAndPowerlines; // 铁塔点云

					for (size_t j = 0; j < refineIndices.size(); ++j)
					{
						const auto& p = _input->points[refineIndices[j]];

						if (!pointInPolygon(polygon, p.x, p.y))
							continue;

						pylonAndPowerlines.push_back(refineIndices[j]);

						if (p.label == ePylon)
							pylonIndices.push_back(refineIndices[j]);
					}

					if (!pylonAndPowerlines.empty())
					{
						// 杆坐标生成 3D 格网
						double voxelsize = 2.0 * connect_threshold;
						Grid3D voxels;
						ComputeLocalGridCells(voxelsize,
											  _input,
											  pylonAndPowerlines,
											  eUnclassified,
											  voxels);

						// 计算挂点位置
						ComputePylonConnections(forwardSpan, backwardSpan, voxels, connection);

						// 合并过于接近的挂点
						MergeConnectionNearly(voxelsize, connection);
					}

					PCS_DEBUG("[PylonRefinement] 杆塔 %d 检测到挂点 %d 个.", i, connection.size());

#ifdef RENDER_CONNECT_POSITIONS
					for (size_t j = 0; j < connection.size(); ++j)
					{
						std::string name = StringPrintf("connections_%d_%d", i, j);
						RenderPosition(name,
									   connection[j],
									   connect_threshold * 0.5,
									   _input,
									   osg::Vec4(0, 1, 0, 1));
					}
#endif
				}

				pylonConnections[i] = connection;
			}
		}

		std::vector<int> PylonRefinement::CorrectPylonInSpan(const osg::Vec3d& center,
															 const PowerlineSpan& span,
															 const std::vector<int>& indices)
		{
			std::vector<int> closedIndices;

			if (span.empty())
				return closedIndices;

			double T_d = _options.max_curve_dist;

			// 处理每股导线
			for (size_t i = 0; i < span.size(); ++i)
			{
				PowerlineCurvePtr curve = span[i];
				CHECK(curve.get());

				if (!curve.get())
					continue;

				for (size_t j = 0; j < indices.size(); ++j)
				{
					const auto& p = _input->points[indices[j]];
					osg::Vec3d v(p.x, p.y, p.z);

					if (curve->DistanceTo(v) > T_d)
						continue;

					closedIndices.push_back(indices[j]);
				}
			}

			return closedIndices;
		}

		std::vector<int> PylonRefinement::GetPowerlinesInSpan(const Grid2D& grid,
															  int i,
															  int j,
															  double step,
															  double width)
		{
			std::vector<int> result;
			const auto& pylonPositions = _pylonPositions;
			const auto& pylonPolygons = _pylonPolygons;
			const auto& pylonSides = _pylonSides;

			const auto& p0 = pylonPositions.at(i);
			const auto& p1 = pylonPositions.at(j);
			const auto& polygon0 = pylonPolygons.at(i);
			const auto& polygon1 = pylonPolygons.at(j);

			osg::Vec3d dir = p1 - p0; // 档方向
			dir.z() = 0.0;
			dir.normalize();
			osg::Vec3d ortho = dir ^ osg::Z_AXIS;

			double height0 = std::min((polygon0[0] - polygon0[1]).length(),
									  (polygon0[0] - polygon0[3]).length());
			double height1 = std::min((polygon1[0] - polygon1[1]).length(),
									  (polygon1[0] - polygon1[3]).length());

			double width0 = std::max((polygon0[0] - polygon0[1]).length(),
									 (polygon0[0] - polygon0[3]).length());
			double width1 = std::max((polygon1[0] - polygon1[1]).length(),
									 (polygon1[0] - polygon1[3]).length());

			osg::Vec3d t0 = p0 + dir * height0 * 0.5;
			osg::Vec3d t1 = p1 - dir * height1 * 0.5;
			double length = (t1 - t0).length();

			width = std::max(width, std::max(width0, width1));
			width += 2.0; // 扩展5m，提高稳定性
			step = std::max(width, step);
			step = std::min(step, length); // 局部范围长度不超过档距

			t1 = t0 + dir * std::min(step * 3.0, length);

			osg::Vec3d side0 = ortho;
			double side_len0 = width;

			// 裁剪的多边形在转角杆塔的边界范围上保持“贴合”
			{
				double cos0 = fabs(pylonSides[i] * ortho);

				if (cos0 == 0.0)
					side0 = ortho;
				else
				{
					side0 = pylonSides[i];
					side_len0 = width / cos0;
				}

				// 保持同侧
				if (dir * side0 < 0.0)
					side0 = -side0;

				if (side0 * ortho < 0.0)
					ortho = -ortho;
			}

			// 计算档局部范围边界
			std::vector<osg::Vec3d> polygon(4);
			polygon[0] = t0 + side0 * 0.5 * side_len0;
			polygon[1] = t1 + ortho * 0.5 * width;
			polygon[2] = t1 - ortho * 0.5 * width;
			polygon[3] = t0 - side0 * 0.5 * side_len0;

#ifdef RENDER_PART_OF_SPAN_BOUND // DEBUG: 绘制当边界
			RenderPowerlineMask(polygon,
								_input,
								StringPrintf("part_of_span_%d_%d", i, j),
								osg::Vec4(240.f / 255.f, 229.f / 255.f, 191.f / 255.f, 1));
#endif

			// 检索多边形边界内的点
			osg::BoundingBox bbox = getPolygonBound(polygon);

			for (size_t m = 0; m < grid.size(); ++m)
			{
				const auto& cell = grid.at(m);

				if (cell.GetSize() <= 0)
					continue;

				if (!(osg::maximum(cell.xMin(), bbox.xMin()) <=
						  osg::minimum(cell.xMax(), bbox.xMax()) &&
					  osg::maximum(cell.yMin(), bbox.yMin()) <=
						  osg::minimum(cell.yMax(), bbox.yMax())))
				{
					continue;
				}

				const auto& cellIndices = cell.GetIndices();

				for (size_t n = 0; n < cellIndices.size(); ++n)
				{
					const auto& p = _input->points[cellIndices[n]];

					if (p.label == ePowerline && pointInPolygon(polygon, p.x, p.y))
						result.push_back(cellIndices[n]);
				}
			}

			return result;
		}

		std::vector<int> PylonRefinement::GetPowerlinesOutOfSpan(const Grid2D& grid,
																 int i,
																 int j,
																 double step,
																 double width)
		{
			std::vector<int> result;
			const auto& pylonPositions = _pylonPositions;
			const auto& pylonPolygons = _pylonPolygons;
			const auto& pylonSides = _pylonSides;

			const auto& p = pylonPositions.at(i);
			const auto& q = pylonPositions.at(j);
			const auto& poly = pylonPolygons.at(i);

			osg::Vec3d dir = p - q;
			dir.z() = 0.0;
			dir.normalize();
			osg::Vec3d ortho = dir ^ osg::Z_AXIS;

			double height = std::min((poly[0] - poly[1]).length(), (poly[0] - poly[3]).length());

			osg::Vec3d t0 = p + dir * height * 0.5;
			osg::Vec3d t1 = t0 + dir * step;

			osg::Vec3d side0 = ortho;
			double side_len0 = width;

			// 裁剪的多边形在转角杆塔的边界范围上保持“贴合”
			{
				double cos0 = fabs(pylonSides[i] * ortho);

				if (cos0 == 0.0)
					side0 = ortho;
				else
				{
					side0 = pylonSides[i];
					side_len0 = width / cos0;
				}

				// 保持同侧
				if (dir * side0 < 0.0)
					side0 = -side0;

				if (side0 * ortho < 0.0)
					ortho = -ortho;
			}

			// 计算档局部范围边界
			std::vector<osg::Vec3d> polygon(4);
			/*polygon[0] = t0 + ortho * 0.5 * width;
			polygon[1] = t1 + ortho * 0.5 * width;
			polygon[2] = t1 - ortho * 0.5 * width;
			polygon[3] = t0 - ortho * 0.5 * width;*/
			polygon[0] = t0 + side0 * 0.5 * side_len0;
			polygon[1] = t1 + side0 * 0.5 * side_len0;
			polygon[2] = t1 - side0 * 0.5 * side_len0;
			polygon[3] = t0 - side0 * 0.5 * side_len0;

#ifdef RENDER_PART_OF_SPAN_BOUND
			RenderPowerlineMask(polygon,
								_input,
								StringPrintf("part_out_of_span_%d_%d", i, j),
								osg::Vec4(240.f / 255.f, 229.f / 255.f, 191.f / 255.f, 1));
#endif
			// 检索多边形边界内的点
			osg::BoundingBox bbox = getPolygonBound(polygon);

			for (size_t m = 0; m < grid.size(); ++m)
			{
				const auto& cell = grid.at(m);

				if (cell.GetSize() <= 0)
					continue;

				if (!(osg::maximum(cell.xMin(), bbox.xMin()) <=
						  osg::minimum(cell.xMax(), bbox.xMax()) &&
					  osg::maximum(cell.yMin(), bbox.yMin()) <=
						  osg::minimum(cell.yMax(), bbox.yMax())))
				{
					continue;
				}

				const auto& cellIndices = cell.GetIndices();

				for (size_t n = 0; n < cellIndices.size(); ++n)
				{
					const auto& p = _input->points[cellIndices[n]];

					if (p.label == ePowerline && pointInPolygon(polygon, p.x, p.y))
						result.push_back(cellIndices[n]);
				}
			}

			return result;
		}

		PowerlineSpan PylonRefinement::ComputeApproxSpan(const std::vector<int>& indices,
														 const osg::Vec3d& pos,
														 const osg::Vec3d& dir,
														 double maxexpand)
		{
			PowerlineSpan span;

			if (indices.empty())
				return span;

			double tolerance = _options.cluster_radius;
			int curveMinPts = _options.cluster_min_points;
			bool fast_clustering = _options.curve_fit_fast_clustering_enable;

			std::vector<std::vector<int>> clusters;

			if (fast_clustering)
			{
				EuclideanVoxelCluster(_input,
									  indices,
									  tolerance,
									  curveMinPts,
									  std::numeric_limits<int>::max(),
									  clusters);
			}
			else
			{
				euclideanClusterSafe(*_input,
									 indices,
									 tolerance,
									 curveMinPts,
									 std::numeric_limits<int>::max(),
									 clusters);
			}

			int label = 1;

			for (size_t i = 0; i < clusters.size(); ++i)
			{
				const auto& cluster = clusters.at(i);

				osg::Vec4ub c =
					ColorManager::colorTable[eGround + label % (eNumOfClassification - eGround)];

				// 拟合单股导线的曲线
				auto curve = PowerlineCurveFitting(_input,
												   cluster,
												   pos,
												   dir,
												   maxexpand,
												   osg::Vec4((float)c.r() / 255.f,
															 (float)c.g() / 255.f,
															 (float)c.b() / 255.f,
															 1.0f)
#ifdef RENDER_FITTING_CURVE
													   ,
												   true
#endif
				);

				// 添加单股导线的曲线到当前档
				span.push_back(curve);

				label++;
			}

			return span;
		}

		void PylonRefinement::ComputePylonConnections(const PowerlineSpan& forward,
													  const PowerlineSpan& backward,
													  const Grid3D& voxels,
													  PylonConnection& connection)
		{
			double connect_threshold = _options.connect_radius;
			const PointCloudViewPtr cloud = _input;

			for (int i = 0; i < (int)forward.size(); ++i)
			{
				auto forwardCurve = forward[i];

				for (int j = 0; j < (int)backward.size(); ++j)
				{
					auto backwardCurve = backward[j];

					//  检测单元内的点云，获得距离前后挡曲线最近点
					std::vector<int> connectIndices;

					auto visitor = [&](const std::shared_ptr<GridCell>& cell) {
						if (!cell || cell->GetSize() <= 0)
							return;

						const auto& center = cell->center();
						const auto& cellIndices = cell->GetIndices();

						// 单元中心与曲线太远，则跳过单元内的点云与曲线的计算
						double dist_to_center0 = forwardCurve->DistanceTo(center);

						if (dist_to_center0 > 3.0 * connect_threshold)
							return;

						// 单元中心与曲线太远，则跳过单元内的点云与曲线的计算
						double dist_to_center1 = backwardCurve->DistanceTo(center);

						if (dist_to_center1 > 3.0 * connect_threshold)
							return;

						for (size_t k = 0; k < cellIndices.size(); ++k)
						{
							const auto& p = cloud->points[cellIndices[k]]; // 铁塔点
							osg::Vec3d v(p.x, p.y, p.z);

							double dist0 = forwardCurve->DistanceTo(v);
							if (dist0 > connect_threshold)
								continue;

							double dist1 = backwardCurve->DistanceTo(v);
							if (dist1 > connect_threshold)
								continue;

							connectIndices.push_back(cellIndices[k]);
						}
					};

					voxels.Visit(visitor);

					if (!connectIndices.empty())
					{
						// 连接点平均位置
						osg::Vec3d cp;
						double inverse = 1.0 / (double)connectIndices.size();

						for (size_t k = 0; k < connectIndices.size(); ++k)
						{
							const auto& p = cloud->points[connectIndices[k]];
							cp += osg::Vec3d(p.x, p.y, p.z) * inverse;
						}

						connection.push_back(cp);
					}
				}
			}

		}

		void PylonRefinement::MergeConnectionNearly(double tolerance, PylonConnection& connection)
		{ 
			// 合并距过于接近的连接点坐标
			std::vector<std::vector<size_t>> clusters;
			std::set<size_t> processed;

			for (size_t i = 0; i < connection.size(); ++i)
			{
				// 已处理过的，不再处理
				if (processed.find(i) != processed.end())
					continue;

				std::vector<size_t> seed_queue;

				int sq_idx = 0;
				seed_queue.push_back(i);
				processed.insert(i);

				while (sq_idx < (int)seed_queue.size())
				{
					size_t k = seed_queue[sq_idx];
					const auto& p = connection.at(k);

					// 遍历相邻接节点
					for (size_t j = 0; j < connection.size(); ++j)
					{
						if (processed.find(j) != processed.end())
							continue;

						const auto& q = connection[j];
						double distance = (q - p).length();

						if (distance < tolerance)
						{
							seed_queue.push_back(j);
							processed.insert(j);
						}
					}

					sq_idx++;
				} // end of while

				if (!seed_queue.empty())
					clusters.push_back(seed_queue);
			}

			PylonConnection newConnection;

			for (size_t i = 0; i < clusters.size(); ++i)
			{
				const auto& cluster = clusters.at(i);

				if (cluster.empty())
					continue;

				osg::Vec3d connectPoint;

				for (size_t j = 0; j < cluster.size(); ++j)
					connectPoint += connection[cluster[j]];

				connectPoint = connectPoint / (double)cluster.size();

				newConnection.push_back(connectPoint);
			}

			std::swap(connection, newConnection);
		}

	} // namespace pcs
} // namespace d3s