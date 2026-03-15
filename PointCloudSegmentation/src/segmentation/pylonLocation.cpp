//stdafx.h
#include "pylonLocation.h"

#include <mutex>
#include <numeric>

#include "../algorithm/math.h"
#include "../algorithm/geometry2d_op.h"

#include "../plot/geomCreator.h"
#include "../plot/plotHandle.h"

#include "../core/pointTypes.hpp"
#include "../core/private/rasterProcess.h"
#include "../core/private/cloudProcess.h"
#include "../core/private/statistics.h"


// #define RENDER_LOCATION_GRID
// #define RENDER_POSITIONS
// #define RENDER_PYLON_PATH
//#define WRITE_FEATURE_IMAGE


#ifdef WRITE_FEATURE_IMAGE
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#endif

extern std::string DebugDirectory;

namespace d3s {
	namespace pcs {
		namespace {

			double SumDeltaH(PointCloudViewPtr input, const std::vector<std::vector<int>>& clusters)
			{
				double sum = 0.0;

				for (size_t i = 0; i < clusters.size(); ++i)
				{
					double zmin = DBL_MAX;
					double zmax = -DBL_MAX;

					const auto& cluster = clusters.at(i);

					for (size_t j = 0; j < cluster.size(); ++j)
					{
						const auto& p = input->points[cluster[j]];

						zmin = std::min(zmin, (double)p.z);
						zmax = std::max(zmax, (double)p.z);
					}

					sum += (zmax - zmin);
				}

				return sum;
			}

			double GetCompatibleResolution(double cellsize)
			{
				if (cellsize >= 1.0)
					return 0.5;
				else
					return 0.2;
			}

			double GetCompatibleLSDScale(double cellsize)
			{
				if (cellsize >= 1.0)
					return 0.7;
				else
					return 0.8;
			}
		}

		// LocationGrid
		//////////////////////////////////////////////////////////////////////////
		LocationGrid::LocationGrid() {}

		LocationGrid::LocationGrid(int width, int height) { resize(width, height); }

		LocationGrid::LocationGrid(const LocationGrid& rhs)
			: Grid2D(rhs), _cellFeatures(rhs._cellFeatures)
		{
		}

		void LocationGrid::resize(int width, int height)
		{
			Grid2D::resize(width, height);
			_cellFeatures.resize(width * height);
		}

		void LocationGrid::ComputeCellFeatures(double height_threshold,
											   double tolerance,
											   int neighbors)
		{
			double distance = tolerance;
			int nSize = size();

			// 计算每个格网的垂直分布系数
#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int idx = 0; idx < nSize; ++idx)
			{
				const auto& cell = at(idx);

				// 跳过空的格网单元
				if (cell.GetSize() <= 0)
					continue;

				PointCloudViewPtr input = (PointCloudViewPtr)cell.GetInput();

				// 单元内点集进行欧式聚类
				std::vector<std::vector<int>> clusters;
				ClusteringOnZAxis(cell, distance, clusters);

				auto& cellFeature = GetFeatureAt(idx);
				double hSum = SumDeltaH(input, clusters);

				cellFeature->Fv = cellFeature->h <= 0.0 ? 0.0 : hSum / cellFeature->h;

				// 统计格网内高于指定阈值的点数
				{
					const auto& indices = cell.GetIndices();

					for (size_t i = 0; i < indices.size(); ++i)
					{
						const auto& p = input->points[indices[i]];

						if (p.hag > height_threshold)
							cellFeature->Fn += 1.0;
					}
				}
			}


			int nNbr = neighbors;
			int nRows = getNumRows();
			int nCols = getNumColumns();

			// 计算格网附近区域的凸包分布系数
#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int r = 0; r < nRows; ++r)
			{
				for (int c = 0; c < nCols; ++c)
				{
					const auto& cell = at(r, c);

					if (cell.GetSize() <= 0)
						continue;

					const auto& cellFeature = GetFeatureAt(r, c);

					double zmax = -DBL_MAX;
					double Fv_max = -DBL_MAX;
					double Fv_sum = 0.0;
					double num = 0.0;

					// 附近的格网上
					for (int yi = r - nNbr; yi <= r + nNbr; ++yi)
					{
						for (int xi = c - nNbr; xi <= c + nNbr; ++xi)
						{
							int ri = clamp(yi, 0, nRows - 1);
							int ci = clamp(xi, 0, nCols - 1);

							// 跳过当前格网单元自身
							if (ci == c && ri == r)
								continue;

							num += 1.0;
							const auto& cellNbr = at(ri, ci);

							if (cellNbr.GetSize() <= 0)
								continue;

							const auto& cellFeatureNbr = GetFeatureAt(ri, ci);

							zmax = std::max(zmax, (double)cellNbr.zMax());
							Fv_max = std::max(Fv_max, cellFeatureNbr->Fv);
							Fv_sum += cellFeatureNbr->Fv;
						}
					}

					// 剔除最大值
					Fv_sum -= Fv_max;
					cellFeature->Fc = 1.0 - (Fv_sum / (num - 1.0));
					cellFeature->Fh = fabs(zmax - cell.zMax()) / cellFeature->h;
				}
			}


#ifdef WRITE_FEATURE_IMAGE
			// DEBUG: 输出垂直分布系数图像、凸包系数图像
			{
				cv::Mat fvMat(nRows, nCols, CV_8UC3, cv::Scalar(0, 0, 0));
				cv::Mat fcMat(nRows, nCols, CV_8UC3, cv::Scalar(0, 0, 0));
				cv::Mat fhMat(nRows, nCols, CV_8UC3, cv::Scalar(0, 0, 0));


				for (int r = 0; r < nRows; ++r)
				{
					for (int c = 0; c < nCols; ++c)
					{
						const auto& cell = at(r, c);

						if (cell.GetSize() <= 0)
							continue;

						const auto& cellFeature = GetFeatureAt(r, c);

						double fv = cellFeature->Fv;
						double fc = cellFeature->Fc;
						double fh = cellFeature->Fh;

						osg::Vec4 color = ColorManager::colorMap[clamp((int)(fv * 255), 0, 255)];
						fvMat.at<cv::Vec3b>(nRows - r - 1, c) =
							cv::Vec3b(255 * color.r(), 255 * color.g(), 255 * color.b());

						color = ColorManager::colorMap[clamp((int)(fc * 255), 0, 255)];
						fcMat.at<cv::Vec3b>(nRows - r - 1, c) =
							cv::Vec3b(255 * color.r(), 255 * color.g(), 255 * color.b());

						color = ColorManager::colorMap[clamp((int)(fh * 255), 0, 255)];
						fhMat.at<cv::Vec3b>(nRows - r - 1, c) =
							cv::Vec3b(255 * color.r(), 255 * color.g(), 255 * color.b());
					}
				}

				std::string fvpath = DebugDirectory + "fv.jpg";
				std::string fcpath = DebugDirectory + "fc.jpg";
				std::string fhpath = DebugDirectory + "fh.jpg";

				cv::cvtColor(fvMat, fvMat, cv::COLOR_BGR2RGB);
				cv::imwrite(fvpath.c_str(), fvMat);

				cv::cvtColor(fcMat, fcMat, cv::COLOR_BGR2RGB);
				cv::imwrite(fcpath.c_str(), fcMat);

				cv::cvtColor(fhMat, fhMat, cv::COLOR_BGR2RGB);
				cv::imwrite(fhpath.c_str(), fhMat);
			}

#endif
		}

		void LocationGrid::ClusteringOnZAxis(const GridCell& cell,
											 double distance,
											 std::vector<std::vector<int>>& clusters)
		{
			if (cell.GetSize() <= 0)
				return;

			const auto& indices = cell.GetIndices();
			PointCloudViewPtr input = (PointCloudViewPtr)cell.GetInput();

			double dH = cell.zMax() - cell.zMin();
			double step = std::min(distance, dH);

			if (step <= 0.0)
				return;

			double stepInv = 1.0 / step;
			int num = std::floor(dH * stepInv) + 1;

			std::vector<std::vector<int>> bins;
			bins.resize(num);

			// 垂直方向分仓
			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& p = input->points[indices[i]];
				double z = p.z;

				int index = std::floor((z - cell.zMin()) * stepInv);
				index = clamp(index, 0, num - 1);

				CHECK(index < bins.size());
				bins[index].push_back(indices[i]);
			}

			// 检测连续区域
			std::vector<int> cluster;

			for (size_t i = 0; i < bins.size(); ++i)
			{
				const auto& bin = bins.at(i);

				if (bin.empty())
				{
					if (!cluster.empty())
						clusters.push_back(cluster);

					cluster.clear();
					continue;
				}

				cluster.insert(cluster.end(), bin.begin(), bin.end());
			}

			if (!cluster.empty())
				clusters.push_back(cluster);
		}

		std::shared_ptr<LocationGrid::CellFeature>& LocationGrid::GetFeatureAt(int row, int col)
		{
			return _cellFeatures[row * _nColumns + col];
		}

		const std::shared_ptr<LocationGrid::CellFeature>& LocationGrid::GetFeatureAt(int row,
																					 int col) const
		{
			return _cellFeatures.at(row * _nColumns + col);
		}

		std::shared_ptr<LocationGrid::CellFeature>& LocationGrid::GetFeatureAt(int index)
		{
			return _cellFeatures[index];
		}

		const std::shared_ptr<LocationGrid::CellFeature>& LocationGrid::GetFeatureAt(
			int index) const
		{
			return _cellFeatures.at(index);
		}

		// PylonLocation
		//////////////////////////////////////////////////////////////////////////

		PylonLocation::PylonLocation(const PylonLocationOptions& options,
									 PointCloudViewPtr input,
									 const std::vector<int>& indices)
			: _options(options), _input(input), _indices(indices)
		{
		}

		PylonLocation::~PylonLocation() {}

		PylonLocation::GridPtr PylonLocation::GetGrid() const { return _grid; }

		void PylonLocation::Location(Positions& pylonPositions)
		{
			int nNbr = _options.location_neighbors;
			double Th_1 = _options.location_th1;
			double cellsize = _options.location_cellsize;
			double vertical_distance = _options.location_zstep;

			GridPtr grid = std::make_shared<LocationGrid>();
			LocalClusters clusters;
			std::vector<LocationGrid::Index> locations;

			// 创建网格
			ComputeLocationGridCells(cellsize, _input, _indices, *grid);

			// 预处理 - 过滤低点位格网
			RemoveLowHeightCells(grid);

			// 计算格网特征系数
			grid->ComputeCellFeatures(Th_1, vertical_distance, nNbr);

			// 格网聚类
			Clustering(grid, clusters);

			// 获得候选格网类簇
			Filtering(grid, clusters);

			if (clusters.empty())
			{
				PCS_WARN("[PylonLocation] 格网聚类数量为空.");
				return;
			}

			// 获得铁塔位置
			ComputePylonLocations(grid, clusters, locations);

			// DEBUG: 绘制杆塔位置
#ifdef RENDER_LOCATION_GRID
			RenderPylonLocations(grid, clusters, locations);
#endif
			auto posistions = GetLocationPositions(grid, locations);

			pylonPositions = ComputePylonPositions(posistions, cellsize);

			// 铁塔位置排序
			PylonPositionSorting(pylonPositions);

			PylonPositionValidate(grid, pylonPositions);

#ifdef RENDER_POSITIONS
			RenderPylonPositions(pylonPositions, _options.location_cellsize, _input);
#endif

			_grid = grid;
			_pylonPostions = pylonPositions;
		}

		void PylonLocation::PylonPositionValidate(GridPtr grid, Positions& pylonPositions)
		{
			double cell_size = _options.location_cellsize;
			double corridor_width = _options.location_corridor_width;
			double T_l = _options.location_max_span;
			double T_sita = _options.min_span_angle_theta;
			int maxError = _options.max_span_error;

			// 检查铁塔之间是否有相同方向的导线
			if (pylonPositions.empty())
			{
				PCS_WARN("[PylonLocation] WARNING: 未检测任何铁塔.");
				return;
			}

			/*
			 * 铁塔档间方向试错：
			 * 1. 根据档的方向检测方向一致的导线数量。如果没有，则表示该档是错误档，错误次数 +1;
			 * 2. 根据检测导线数量占档间总导线数量的占比、导线长度与档距的比例，排除交叉跨越;
			 * 3. 汇总正确的档，确定导线方向
			 */
			std::vector<int> span;
			std::vector<int> removed;

			int cur = 0;
			span.push_back(cur);

			while (cur != pylonPositions.size())
			{
				int errorInSpan = 0;
				int curRedirect = cur;

				for (; errorInSpan < maxError; ++errorInSpan)
				{
					int next = cur + errorInSpan + 1;

					if (next >= pylonPositions.size())
						break;

					const osg::Vec3d& p0 = pylonPositions[cur];
					const osg::Vec3d& p1 = pylonPositions[next];


					// 检索边界内的电力线点云
					std::vector<int> powerlineInSpan;
					GetPowerlineIndices(grid, p0, p1, 5, corridor_width, powerlineInSpan);

					if (powerlineInSpan.size() <= kMinPointsInlinerCell)
						continue;

					osg::BoundingBox bbox;
					computeMinMax3D(*_input, powerlineInSpan, bbox);

					// 生成二值图
					double resolution = GetCompatibleResolution(cell_size);
					Rasterd mask;
					createBinaryMask(*_input, powerlineInSpan, bbox, resolution, mask);

					// 检测图像线段参数
					std::vector<std::array<osg::Vec2d, 2>> lines;
					ImageProcessParameter p;
					p.line_lsd_scale = GetCompatibleLSDScale(cell_size);

					// 检测图像中的线段
					detectLinesLSD(mask, p, lines);

					// 检测直线是否与档方向接近平行
					osg::Vec2d spanDir = osg::Vec2d(p1.x(), p1.y()) - osg::Vec2d(p0.x(), p0.y());

					int numSameDirecion = 0;
					double spanLength = spanDir.length();
					double approx = tanh(32.0 * (spanLength / T_l)); // 1km 最大阈值
					// double angle_threshold = approx * T_θ;
					double angle_threshold = T_sita;

					removeSmallLengths(lines, spanLength * 0.1); // 移除较短的线段

					PCS_INFO("[PylonLocation] 导线夹角在档长度上自适应系数 approx = %lf, "
							 "angle_threshold=%lf.",
							 approx,
							 angle_threshold);

					spanDir.normalize();

					for (size_t i = 0; i < lines.size(); ++i)
					{
						const auto& line = lines[i];
						osg::Vec2d lineDir = line[1] - line[0];
						lineDir.normalize();
						double cosTheta = lineDir * spanDir;

						if (fabs(cosTheta) > angle_threshold) // 根据档长度自适应角度阈值
							++numSameDirecion;

						PCS_DEBUG("[PylonLocation] %d-%d COS(θ) between spanDir and lineDir: %lf.",
								  cur,
								  next,
								  cosTheta);
					}

					PCS_INFO("[PylonLocation] %d-%d Same direction line number: %d.",
							 cur,
							 next,
							 numSameDirecion);


					// 有超过半数的相同方向的直线，判定该档为有效挡
					if (numSameDirecion > 6 || numSameDirecion > 0.5 * lines.size())
					{
						span.push_back(next);
						curRedirect = next;
						break;
					}
					else
					{
						removed.push_back(next);
					}
				}

				if (cur != curRedirect)
					cur = curRedirect;
				else
					cur++;
			}

			std::vector<osg::Vec3d> pylonPositionsTemp;
			pylonPositionsTemp.reserve(pylonPositions.size());

			for (size_t i = 0; i < span.size(); ++i)
				pylonPositionsTemp.push_back(pylonPositions[span[i]]);

			PCS_INFO("[PylonLocation] 原始铁塔坐标数: %d, 校验后铁塔坐标数: %d.",
					 pylonPositions.size(),
					 pylonPositionsTemp.size());

			std::swap(pylonPositions, pylonPositionsTemp);

#ifdef RENDER_PYLON_PATH
			// 显示到界面
			RenderLinePath("pylons", pylonPositions, _input, osg::Vec4(0, 1, 0, 1));
#endif
		}

		void PylonLocation::PylonPositionSorting(Positions& pylonPositions)
		{
			osg::Vec3d major, middle, minor;
			std::vector<osg::Vec3d> pts;

			for (size_t i = 0; i < pylonPositions.size(); ++i)
			{
				osg::Vec3d pnt = pylonPositions[i];
				pnt.z() = 0.0;
				pts.push_back(pnt);
			}

			// PCA特征向量
			getEigenVectors(pts, major, middle, minor);
			osg::Vec3d orientation = major;
			osg::Vec3d center = _input->bbox.center();
			center.z() = 0.0;

			// 按照主方向投影排序路径点
			std::sort(pylonPositions.begin(),
					  pylonPositions.end(),
					  [&](const osg::Vec3d& lhs, const osg::Vec3d& rhs) {
						  osg::Vec3d p0 = lhs;
						  osg::Vec3d p1 = rhs;
						  p0.z() = p1.z() = 0.0;

						  osg::Vec3d d1 = p0 - center;
						  double l1 = d1.length();
						  d1.normalize();
						  double project1 = orientation * d1 * l1;

						  osg::Vec3d d2 = p1 - center;
						  double l2 = d2.length();
						  d2.normalize();
						  double project2 = orientation * d2 * l2;

						  return project1 < project2;
					  });
		}




		void PylonLocation::GetPowerlineIndices(GridPtr grid,
												const osg::Vec3d& p0,
												const osg::Vec3d& p1,
												double offset,
												double width,
												std::vector<int>& indices)
		{
			double Th_1 = _options.location_th1;

			osg::Vec3d dir = p1 - p0; // 方向
			dir.normalize();
			osg::Vec3d ortho = dir ^ osg::Z_AXIS;

			std::vector<osg::Vec3d> polygon(4);
			polygon[0] = p0 + dir * offset + ortho * 0.5 * width;
			polygon[1] = p1 - dir * offset + ortho * 0.5 * width;
			polygon[2] = p1 - dir * offset - ortho * 0.5 * width;
			polygon[3] = p0 + dir * offset - ortho * 0.5 * width;
			osg::BoundingBox bbox = getPolygonBound(polygon);



			for (size_t i = 0; i < grid->size(); ++i)
			{
				const auto& cell = grid->at(i);
				if (cell.GetSize() <= 0)
					continue;

				if (!(osg::maximum(cell.xMin(), bbox.xMin()) <=
						  osg::minimum(cell.xMax(), bbox.xMax()) &&
					  osg::maximum(cell.yMin(), bbox.yMin()) <=
						  osg::minimum(cell.yMax(), bbox.yMax())))
				{
					continue;
				}


				const auto& cellFeature = grid->GetFeatureAt(i);
				const auto& cellIndices = cell.GetIndices();

				for (size_t k = 0; k < cellIndices.size(); ++k)
				{
					const auto& p = _input->points[cellIndices[k]];

					if (p.hag > Th_1 && pointInPolygon(polygon, p.x, p.y))
						indices.push_back(cellIndices[k]);
				}
			}
		}

		PylonLocation::Positions PylonLocation::GetLocationPositions(
			GridPtr grid,
			const std::vector<LocationGrid::Index>& locations)
		{
			double Th_1 = _options.location_th1;
			int nNbr = _options.location_neighbors;
			int nRows = grid->getNumRows();
			int nCols = grid->getNumColumns();

			Positions positions;
			positions.resize(locations.size());


#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)locations.size(); ++i)
			{
				const auto& index = locations.at(i);
				int r = index[0];
				int c = index[1];

				std::vector<int> indices;

				// 获得附近格网的点云
				for (int yi = r - nNbr; yi <= r + nNbr; ++yi)
				{
					for (int xi = c - nNbr; xi <= c + nNbr; ++xi)
					{
						int ri = clamp(yi, 0, nRows - 1);
						int ci = clamp(xi, 0, nCols - 1);

						const auto& cellNbr = grid->at(ri, ci);

						if (cellNbr.GetSize() <= 0)
							continue;

						const auto& cellIndicesNbr = cellNbr.GetIndices();

						for (size_t j = 0; j < cellIndicesNbr.size(); ++j)
						{
							const auto& p = _input->points[cellIndicesNbr[j]];

							if (p.hag > Th_1)
								indices.push_back(cellIndicesNbr[j]);
						}
					}
				}

				// 中心点
				for (size_t j = 0; j < indices.size(); ++j)
				{
					const auto& p = _input->points[indices[j]];

					positions[i] += osg::Vec3d(p.x, p.y, p.z) / (double)indices.size();
				}
			}

			return positions;
		}

		PylonLocation::Positions PylonLocation::ComputePylonPositions(const Positions& positions,
																	  double cellsize)
		{
			int nNbr = _options.location_neighbors;

			// 合并距过于接近的坐标，他们其实是杆塔坐标
			std::vector<std::vector<int>> position_clusters;
			std::vector<bool> processed(positions.size(), false);

			for (int i = 0; i < (int)positions.size(); ++i)
			{
				// 已处理过的，不再处理
				if (processed[i])
					continue;

				std::vector<int> seed_queue;

				int sq_idx = 0;
				seed_queue.push_back(i);
				processed[i] = true;

				while (sq_idx < (int)seed_queue.size())
				{
					int ii = seed_queue[sq_idx];
					const auto& pos0 = positions.at(ii);

					for (int j = 0; j < (int)positions.size(); ++j)
					{
						if (processed[j])
							continue;

						const auto& pos1 = positions.at(j);
						double distance = (pos1 - pos0).length();

						if (distance < cellsize * (2 * nNbr + 1))
						{
							seed_queue.push_back(j);
							processed[j] = true;
						}
					}

					sq_idx++;
				} // end of while

				if (!seed_queue.empty())
					position_clusters.push_back(seed_queue);
			}

			// 坐标点簇的平均值作为杆塔坐标
			Positions finalPositions;

			for (size_t i = 0; i < position_clusters.size(); ++i)
			{
				const auto& position_cluster = position_clusters.at(i);

				if (position_cluster.empty())
					continue;

				osg::Vec3d position;

				for (size_t j = 0; j < position_cluster.size(); ++j)
					position += positions[position_cluster[j]];

				position = position / (double)position_cluster.size();

				finalPositions.push_back(position);
			}

			PCS_INFO("[PylonLocation] 检测到杆塔 %d 基.", finalPositions.size());

			return finalPositions;
		}

#if 0
		void PylonLocation::ComputePylonLocations(GridPtr grid,
												  const LocalClusters& clusters,
												  std::vector<LocationGrid::Index>& locations)
		{
			if (!grid)
				return;

			double T_v = _options.location_tv;
			double T_c = _options.location_tc;
			double T_h = _options.location_th;

			// 计算杆塔位置
			for (size_t c = 0; c < clusters.size(); ++c)
			{
				const auto& cluster = clusters[c];

				for (size_t i = 0; i < cluster.size(); ++i)
				{
					auto index = cluster.at(i);

					const auto& cell = grid->at(index[0], index[1]);

					if (cell.GetSize() <= 0)
						continue;

					const auto& cellFeature = grid->GetFeatureAt(index[0], index[1]);

					if (cellFeature->Fv > T_v && cellFeature->Fc > T_c /*&& cellFeature->Fh < T_h*/)
					{
						locations.push_back(index);
					}
				}
			}

			PCS_INFO("[PylonLocation] 检测到杆塔候选格网 %d 个.", locations.size());
		}
#endif

		void PylonLocation::ComputePylonLocations(GridPtr grid,
												  const LocalClusters& clusters,
												  std::vector<LocationGrid::Index>& locations)
		{
			double T_outlier = _options.location_outlier_coeff;
			double T_v = _options.location_tv;
			double T_c = _options.location_tc;

			std::vector<double> data;

			for (size_t c = 0; c < clusters.size(); ++c)
			{
				const auto& cluster = clusters[c];

				for (size_t i = 0; i < cluster.size(); ++i)
				{
					auto index = cluster.at(i);

					const auto& cell = grid->at(index[0], index[1]);

					if (cell.GetSize() <= 0)
						continue;

					const auto& cellFeature = grid->GetFeatureAt(index[0], index[1]);

					if (cellFeature->Fn > 0.0)
						data.push_back(cellFeature->Fn);
				}
			}

			double mean = Mean(data);	  // 均值
			double stddev = Stddev(data); // 标准差

			for (size_t c = 0; c < clusters.size(); ++c)
			{
				const auto& cluster = clusters[c];

				for (size_t i = 0; i < cluster.size(); ++i)
				{
					auto index = cluster.at(i);

					const auto& cell = grid->at(index[0], index[1]);

					if (cell.GetSize() <= 0)
						continue;

					const auto& cellFeature = grid->GetFeatureAt(index[0], index[1]);

					if ((cellFeature->Fn - mean) > T_outlier * stddev && cellFeature->Fv > T_v &&
						cellFeature->Fc > T_c)
					{
						locations.push_back(index);
					}
				}
			}

			PCS_INFO("[PylonLocation] 检测到杆塔候选格网 %d 个.", locations.size());
		}

		void PylonLocation::Filtering(GridPtr grid, LocalClusters& clusters)
		{
			if (!grid)
				return;

			double T_LC = _options.location_linearity;
			double T_L = _options.location_min_proj;
			double T_mc = 0.75;

			std::vector<bool> clusterValids(clusters.size(), true);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)clusters.size(); ++i)
			{
				const auto& cluster = clusters.at(i);

				// std::vector<int> indices;
				std::vector<osg::Vec3d> pts;

				// 获得类簇点集
				for (size_t j = 0; j < cluster.size(); ++j)
				{
					auto index = cluster.at(j);
					const auto& cell = grid->at(index[0], index[1]);

					if (cell.GetSize() <= 0)
						continue;

					// const auto& cellIndices = cell.GetIndices();
					// indices.insert(indices.end(), cellIndices.begin(), cellIndices.end());
					osg::Vec3d center = cell.center();
					center.z() = 0; 
					pts.push_back(center);
				}

				if (pts.size() > 10)
				{
					//for (size_t j = 0; j < indices.size(); ++j)
					//{
					//	const auto& p = _input->points[indices[j]];
					//	osg::Vec3d v = osg::Vec3d(p.x, p.y, p.z);
					//	v.z() = 0; // 只考虑 XY平面上的特征值、特征向量
					//	pts.push_back(v);
					//}

					// 计算类簇点集在XY平面上的特征值、特征向量
					double lamda1 = 0.0;
					double lamda2 = 0.0;
					double lamda3 = 0.0;
					osg::Vec3d v1, v2, v3;

					getEigenValues(pts, lamda1, lamda2, lamda3); // λ1 > λ2 > λ3
					getEigenVectors(pts, v1, v2, v3);
					v1.normalize();

					double maxProj = -DBL_MAX;
					double minProj = DBL_MAX;

					for (size_t j = 0; j < pts.size(); ++j)
					{
						const osg::Vec3d& v = pts.at(j);

						maxProj = std::max(maxProj, v * v1);
						minProj = std::min(minProj, v * v1);
					}

					double Lc = 1.0 - (lamda2 / lamda1);
					double Pl = maxProj - minProj;

					clusterValids[i] = (Lc > T_LC) && (Pl > T_L);
				}
				else
					clusterValids[i] = false;
			}

			// 移除无效类簇，剩余类簇为候选类簇
			for (int i = (int)clusters.size() - 1; i >= 0; --i)
			{
				if (!clusterValids.at(i))
					clusters.erase(clusters.begin() + i);
			}


			for (int i = (int)clusters.size() - 1; i >= 0; --i)
			{
				const auto& cluster = clusters.at(i);

				// 只剩余一个类簇的情况下，保留它，无论凸包系数是否满足要求
				if (clusters.size() < 2)
					break;

				std::vector<double> values;

				for (size_t j = 0; j < cluster.size(); ++j)
				{
					auto index = cluster.at(j);
					const auto& cell = grid->at(index[0], index[1]);

					if (cell.GetSize() <= 0)
						continue;

					const auto& cellFeature = grid->GetFeatureAt(index[0], index[1]);
					values.push_back(cellFeature->Fc);
				}

				double medianFC = Median(values);
				PCS_INFO("[PylonLocation] 格网类簇 %d 凸包系数中位数: %lf", i, medianFC);

				if (medianFC < T_mc)
					clusters.erase(clusters.begin() + i);
			}

			// 设置格网类簇ID
			for (size_t i = 0; i < clusters.size(); ++i)
			{
				const auto& cluster = clusters[i];

				for (size_t j = 0; j < cluster.size(); ++j)
				{
					auto index = cluster[j];
					auto& cellFeature = grid->GetFeatureAt(index[0], index[1]);

					cellFeature->clusterId = i;
				}
			}
		}

		void PylonLocation::Clustering(GridPtr grid, LocalClusters& clusters)
		{
			if (!grid)
				return;

			double Th_2 = _options.location_th2;

			int nRows = grid->getNumRows();
			int nCols = grid->getNumColumns();

			std::vector<bool> processed(nRows * nCols, false);

			for (int r = 0; r < nRows; ++r)
			{
				for (int c = 0; c < nCols; ++c)
				{
					const auto& currentCell = grid->at(r, c);

					if (currentCell.GetSize() <= 0)
						continue;

					int i = r * nCols + c;

					// 已处理过的，不再处理
					if (processed[i])
						continue;

					std::vector<LocationGrid::Index> seed_queue;

					int sq_idx = 0;
					seed_queue.push_back({ r, c });
					processed[i] = true;

					while (sq_idx < (int)seed_queue.size())
					{
						LocationGrid::Index index = seed_queue[sq_idx];
						const int rr = index[0];
						const int cc = index[1];

						const auto& cell = grid->at(rr, cc);

						// 跳过空的格网
						if (cell.GetSize() <= 0)
						{
							sq_idx++;
							continue;
						}

						const auto& cellFeature = grid->GetFeatureAt(rr, cc);

						//  8 邻域生长，遍历相邻接节点
						for (int yi = rr - 1; yi <= rr + 1; ++yi)
						{
							for (int xi = cc - 1; xi <= cc + 1; ++xi)
							{
								int ri = clamp(yi, 0, nRows - 1);
								int ci = clamp(xi, 0, nCols - 1);

								int j = ri * nCols + ci;
								if (processed[j])
									continue;

								const auto& cellNbr = grid->at(ri, ci);
								if (cellNbr.GetSize() <= 0)
									continue;

								const auto& cellFeatureNbr = grid->GetFeatureAt(ri, ci);
								double hDelta = fabs(cell.zMax() - cellNbr.zMax());
								double fcDelta = fabs(cellFeature->Fc - cellFeatureNbr->Fc);

								if (hDelta < Th_2)
								{
									seed_queue.push_back({ ri, ci });
									processed[j] = true;
								}
							}
						}

						sq_idx++;
					} // end of while

					if (!seed_queue.empty())
						clusters.push_back(seed_queue);
				}
			}
		}

		void PylonLocation::RemoveLowHeightCells(GridPtr grid)
		{
			if (!grid)
				return;

			double Th_1 = _options.location_th1;

			int nRows = grid->getNumRows();
			int nCols = grid->getNumColumns();

			for (int r = 0; r < nRows; ++r)
			{
				for (int c = 0; c < nCols; ++c)
				{
					auto& cell = grid->at(r, c);

					if (cell.GetSize() <= 0)
						continue;

					const auto& cellFeature = grid->GetFeatureAt(r, c);

					if (cellFeature->h < Th_1)
						cell.Clear();
				}
			}
		}



		void PylonLocation::RenderPylonLocations(GridPtr grid,
												 const LocalClusters& clusters,
												 const std::vector<LocationGrid::Index>& locations)
		{
#if 1
			std::vector<osg::Vec4> colorMap = createColors(locations.size());

			for (size_t i = 0; i < locations.size(); ++i)
			{
				const auto& index = locations[i];
				auto& cellFeature = grid->GetFeatureAt(index[0], index[1]);
				cellFeature->label = i;
			}
#else
			std::vector<osg::Vec4> colorMap = createColors(clusters.size());

			for (size_t i = 0; i < clusters.size(); ++i)
			{
				const auto& cluster = clusters[i];

				for (size_t j = 0; j < cluster.size(); ++j)
				{
					auto index = cluster[j];
					auto& cellFeature = grid->GetFeatureAt(index[0], index[1]);

					cellFeature->label = cellFeature->clusterId;
				}
			}
#endif

			// DEBUG: 显示格网聚类簇
			RenderGridCells(*grid, colorMap, _input);
		}

		// ComputeLocationGridCells
		//////////////////////////////////////////////////////////////////////////
		void ComputeLocationGridCells(double cellsize,
									  PointCloudViewPtr input,
									  const std::vector<int>& indices,
									  LocationGrid& grid)
		{
			CHECK_MSG(input, "无效点云数据.");
			CHECK_MSG(!indices.empty(), "空的点云数据.");

			osg::BoundingBox bbox;
			computeMinMax3D(*input, indices, bbox);

			double xmin = bbox.xMin();
			double xmax = bbox.xMax();
			double ymin = bbox.yMin();
			double ymax = bbox.yMax();

			int numRow = std::floor((ymax - ymin) / cellsize) + 1;
			int numColumn = std::floor((xmax - xmin) / cellsize) + 1;

			CHECK_MSG(numColumn > 0, StringPrintf("xmin=%lf, xmax=%lf.", xmin, xmax));
			CHECK_MSG(numRow > 0, StringPrintf("ymin=%lf, ymax=%lf.", ymax, ymin));

			// 网格内存分配
			grid.resize(numColumn, numRow);

			// 初始化网格
#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int r = 0; r < numRow; ++r)
			{
				for (int c = 0; c < numColumn; ++c)
				{
					GridCell& cell = grid.at(r, c);
					cell.SetInput(input);
				}
			}

			int numMutex = cellsize < 1.0 ? 1 : numRow * numColumn;
			std::vector<std::mutex> gridMutex(numMutex);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)indices.size(); ++i)
			{
				const auto& p = input->points[indices[i]];

				int ri = std::floor((p.y - ymin) / cellsize);
				int ci = std::floor((p.x - xmin) / cellsize);
				ci = clamp(ci, 0, numColumn - 1);
				ri = clamp(ri, 0, numRow - 1);

				// 对应单元加锁
				int mutexIndex = cellsize < 1.0 ? 0 : ri * numColumn + ci;
				std::unique_lock<std::mutex> lock(gridMutex[mutexIndex]);
				GridCell& cell = grid.at(ri, ci);
				cell.AddPoint(indices[i]);
			}


			// 更新包围盒
#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int r = 0; r < numRow; ++r)
			{
				for (int c = 0; c < numColumn; ++c)
				{
					GridCell& cell = grid.at(r, c);

					const auto& indices = cell.GetIndices();

					if (indices.empty())
						continue;

					auto& cellFeature = grid.GetFeatureAt(r, c);
					cellFeature = std::make_shared<LocationGrid::CellFeature>();

					double hmax = -DBL_MAX;
					double zmax = -DBL_MAX;
					double zmin = DBL_MAX;

					for (size_t i = 0; i < indices.size(); ++i)
					{
						const auto& p = input->points[indices[i]];

						hmax = std::max(hmax, (double)p.hag);
						zmax = std::max(zmax, (double)p.z);
						zmin = std::min(zmin, (double)p.z);
					}

					cellFeature->h = hmax; // 格网内点的最大高差
					cell._min = osg::Vec3d(xmin + c * cellsize, ymin + r * cellsize, zmin);
					cell._max =
						osg::Vec3d(xmin + (c + 1) * cellsize, ymin + (r + 1) * cellsize, zmax);
				}
			}

			PCS_INFO("[ComputeLocationGridCells] 格网划分完成(%lf).", cellsize);
		}
	}
}