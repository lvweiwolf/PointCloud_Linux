#include <src/segmentation/improve/pylonClassifier.h>
#include <src/segmentation/pylonCommon.h>
#include <src/segmentation/gridCell.h>

#include <src/algorithm/geometry2d_op.h>
#include <src/core/private/rasterProcess.h>
#include <src/core/private/cloudProcess.h>
#include <src/core/private/statistics.h>
#include <src/plot/geomCreator.h>
#include <src/plot/plotHandle.h>

#include <osgDB/WriteFile>

// #define RENDER_POSITIONS

extern std::string DebugDirectory;

namespace d3s {
	namespace pcs {

		PylonClassifier::PylonClassifier(const PylonClassifyOptions& options,
										 PointCloudViewPtr input,
										 const std::vector<int>& indices)
			: _options(options), _input(input), _indices(indices)
		{
		}

		PylonClassifier::~PylonClassifier() {}

		PylonClassifier::Polygons PylonClassifier::GetPylonPolygons() const
		{
			return _pylonPolygons;
		}

		PylonClassifier::Vectors PylonClassifier::GetPylonSides() const { return _pylonSides; }

		void PylonClassifier::SetPylonPositions(const Positions& pylonPositions)
		{
			_pylonPositions = pylonPositions;
		}

		PylonClassifier::Positions PylonClassifier::GetPylonPositions() const
		{
			return _pylonPositions;
		}

		void PylonClassifier::Segment()
		{
			if (_indices.empty())
				return;

			double resolution = _options.body_growing_resolition;
			double noise_cellsize = 0.5 * _options.cell_size;
			double step = _options.body_growing_step;
			double H_min = _options.min_body_growing_height;
			double H_max = _options.max_body_growing_height;
			double T_h = _options.height_threshold;
			double T_r = 1.5;


			Grid2D grid;
			ComputeLocalGridCells(5.0, _input, _indices, grid);

			Polygons pylonPolygons;
			Vectors pylonSides;

			ComputePylonPolygons(grid, _pylonPositions, pylonPolygons, pylonSides);

			// 处理校验后的铁塔位置
			for (size_t ipos = 0; ipos < pylonPolygons.size(); ++ipos)
			{
				const auto& polygon = pylonPolygons.at(ipos);
				osg::BoundingBox pbb = getPolygonBound(polygon);

				std::vector<int> indices; // 杆塔边界范围内的点云

				for (size_t i = 0; i < grid.size(); ++i)
				{
					const auto& cell = grid.at(i);

					if (cell.GetSize() <= 0)
						continue;

					if (!(osg::maximum(cell.xMin(), pbb.xMin()) <=
							  osg::minimum(cell.xMax(), pbb.xMax()) &&
						  osg::maximum(cell.yMin(), pbb.yMin()) <=
							  osg::minimum(cell.yMax(), pbb.yMax())))
					{
						continue;
					}

					const auto& cellIndices = cell.GetIndices();

					for (size_t j = 0; j < cellIndices.size(); ++j)
					{
						auto& p = _input->points[cellIndices[j]];

						if (pointInPolygon(polygon, p.x, p.y))
							indices.push_back(cellIndices[j]);
					}
				}

				// 悬空噪点去除
				RemoveNoiseAbove(noise_cellsize, indices);

				if (indices.empty())
					continue;

				// 计算边界
				osg::BoundingBox bbox;
				computeMinMax3D(*_input, indices, bbox);

				// 局部铁塔点的最大/最小高差

				double hmax = -DBL_MAX;
				double zmax = -DBL_MAX;
				double zmin = DBL_MAX;
				double zupper = -DBL_MAX;
				double zlower = -DBL_MAX;


				// 在相对高度范围内的点云中，确定最低高程值
				for (size_t i = 0; i < indices.size(); ++i)
				{
					const auto& p = _input->points[indices[i]];
					zmin = std::min(zmin, (double)p.z);
					zmax = std::max(zmax, (double)p.z);

					if (p.hag < H_max)
						zupper = std::max(zupper, (double)p.z);

					if (p.hag < H_min)
						zlower = std::max(zlower, (double)p.z);
				}

				if (zupper < 0.0 || zlower < 0.0)
					continue;

				zupper = std::min(zupper, zmax);
				zlower = std::max(zlower, zmin);

				// 安置间隔沿垂直方向分层
				auto layers = CreateVerticalLayers(indices, zlower, zupper, step);

				if (layers.empty())
					continue;

				int morphSize0 = 0.06;
				int morphSize1 = 0.06;
				std::vector<Rasterd> masks;
				masks.resize(layers.size());

				// 按垂直分层生成点云掩码
				for (size_t i = 0; i < layers.size(); ++i)
				{
					const auto& layer = layers[i];
					CHECK(!layer.empty());

					createBodyMask(*_input, layer, bbox, resolution, morphSize0, i, masks[i]);
				}

				// 从顶部到底部，保留相邻掩码共有连通区域
				for (size_t i = masks.size() - 1; i >= 1; --i)
				{
					const auto& mask0 = masks[i];
					auto& mask1 = masks[i - 1];

					intersectConnectComponents(mask0, mask1, i - 1);
				}

				// 掩码膨胀，避免铁塔上的点
				for (size_t i = 0; i < masks.size(); ++i)
					dilate(masks[i], morphSize1, i);

				// 将过滤好的掩码应用到对应层的点云
				for (size_t i = 0; i < layers.size(); ++i)
				{
					const auto& layer = layers[i];
					const auto& maskBin = masks[i];

					double halfEdge = maskBin.edgeLength() / 2.0;
					double edgeBit = maskBin.edgeLength() * .000001;

					for (size_t j = 0; j < layer.size(); ++j)
					{
						auto& p = _input->points[layer[j]];

						int xi = maskBin.xCell(p.x + halfEdge - edgeBit);
						int yi = maskBin.yCell(p.y + halfEdge - edgeBit);

						xi = clamp(xi, 0, maskBin.width() - 1);
						yi = clamp(yi, 0, maskBin.height() - 1);

						if (maskBin.at(xi, yi) == 1.0)
							p.label = ePylon;
						else
							p.label = eUnclassified;
					}
				}

				// 处理塔体下方的点
				CHECK(!masks.empty());

				auto& maskLower = masks[0];
				dilate(maskLower, morphSize1, 0);

				for (size_t i = 0; i < indices.size(); ++i)
				{
					auto& p = _input->points[indices[i]];

					if (p.z < zlower)
					{
						double halfEdge = maskLower.edgeLength() / 2.0;
						double edgeBit = maskLower.edgeLength() * .000001;

						int xi = maskLower.xCell(p.x + halfEdge - edgeBit);
						int yi = maskLower.yCell(p.y + halfEdge - edgeBit);

						xi = clamp(xi, 0, maskLower.width() - 1);
						yi = clamp(yi, 0, maskLower.height() - 1);

						if (maskLower.at(xi, yi) == 1.0 && p.hag >= 1.0)
							p.label = ePylon;
						else
							p.label = eUnclassified;
					}
				}

				// 铁塔边界范围内，未分类点降噪
				double voxelsize = 1.0;
				unsigned int minpts = 50;
				std::vector<int> undefined, corrects;

				for (size_t i = 0; i < indices.size(); ++i)
				{
					auto& p = _input->points[indices[i]];

					if (p.label != eUnclassified)
						continue;

					if (p.hag > std::max(T_h, H_min))
					{
						p.label = ePylon;
					}
					else if (p.z > zlower)
					{
						undefined.push_back(indices[i]);
					}
				}
			}

			_pylonPolygons = pylonPolygons; // 设置杆塔边界
			_pylonSides = pylonSides;		// 设置杆塔横担方向
		}

		void PylonClassifier::ComputePylonPolygons(const Grid2D& grid,
												   Positions& pylonPositions,
												   Polygons& pylonPolygons,
												   Vectors& pylonSides)
		{
			int nNbr = _options.num_neighbors;
			double cell_size = _options.cell_size;
			double towerHeadLength = _options.head_length;
			double defaultRadius = (2 * nNbr + 1) * cell_size;
			double T_h = _options.height_threshold;
			double d1_scale = _options.d1_scale;
			double d2_scale = _options.d2_scale;

			pylonPolygons.resize(pylonPositions.size());
			pylonSides.resize(pylonPositions.size());

			Positions newPylonPositions = pylonPositions;
			std::vector<double> pylonRadius(pylonPositions.size(), defaultRadius);

			// 第一次，使用原始杆塔位置，构建杆塔边界，计算实际杆塔位置和实际边界
			for (size_t i = 0; i < pylonPositions.size(); ++i)
			{
				osg::Vec3d dir, side;
				const auto& cur = pylonPositions[i];

				// 计算杆塔的朝向
				if (i == 0)
					side = GetSideVector(pylonPositions, i, i + 1);
				else if (i == pylonPositions.size() - 1)
					side = GetSideVector(pylonPositions, i - 1, i);
				else
					side = GetSideVector(pylonPositions, i - 1, i, i + 1);

				dir = side ^ osg::Z_AXIS;

				double dLen = defaultRadius;
				std::vector<osg::Vec3d> polygon(4);

				polygon[0] = cur - dir * 0.3 * dLen - side * 1.1 * dLen;
				polygon[1] = cur + dir * 0.3 * dLen - side * 1.1 * dLen;
				polygon[2] = cur + dir * 0.3 * dLen + side * 1.1 * dLen;
				polygon[3] = cur - dir * 0.3 * dLen + side * 1.1 * dLen;

				// 杆塔候选点云
				std::vector<int> headIndices;
				std::vector<int> indicesCandidate = GetPylonCandidate(grid, polygon, T_h);

				// 获得塔头点云
				GetHeadIndices(_input, indicesCandidate, towerHeadLength, headIndices);

				// 重新计算横担方向
				if (i == 0 || i == pylonPositions.size() - 1)
				{
					GetHeadDirection(_input, indicesCandidate, 2.0, side, i);
					dir = side ^ osg::Z_AXIS;

					// 更新杆塔边界
					polygon[0] = cur - dir * 0.3 * dLen - side * 1.1 * dLen;
					polygon[1] = cur + dir * 0.3 * dLen - side * 1.1 * dLen;
					polygon[2] = cur + dir * 0.3 * dLen + side * 1.1 * dLen;
					polygon[3] = cur - dir * 0.3 * dLen + side * 1.1 * dLen;

					indicesCandidate = GetPylonCandidate(grid, polygon, T_h);

					// 再次获得纠正杆塔边界后的塔头点云
					GetHeadIndices(_input, indicesCandidate, towerHeadLength, headIndices);
				}

				pylonSides[i] = side; // 杆塔横担方向

				// 计算塔头半径
				if (!headIndices.empty())
				{
					pylonRadius[i] = ComputePositionRadius(headIndices, newPylonPositions[i], i);
				}

				// 终端塔、转角塔不调整杆塔坐标
				newPylonPositions[i] = pylonPositions[i];
			}

			// 更新杆塔坐标
			pylonPositions = newPylonPositions;

#ifdef RENDER_POSITIONS
			RenderPylonPositions(pylonPositions, _options.cell_size, _input);
#endif
			double scale = 1.0;

			// 第二次，根据实际杆塔位置和半径，构建杆塔边界
			for (size_t i = 0; i < pylonPositions.size(); ++i)
			{
				const auto& cur = pylonPositions[i];
				const auto& side = pylonSides[i];
				osg::Vec3d dir = side ^ osg::Z_AXIS;

				if (i == 0 || i == pylonPositions.size() - 1)
					scale = _options.terminal_scale;

				double dLen = (pylonRadius[i] * 0.5 * scale) * 2.0;
				std::vector<osg::Vec3d> polygon(4);

				// 正方形
				polygon[0] = cur - dir * d1_scale * dLen - side * d2_scale * dLen;
				polygon[1] = cur + dir * d1_scale * dLen - side * d2_scale * dLen;
				polygon[2] = cur + dir * d1_scale * dLen + side * d2_scale * dLen;
				polygon[3] = cur - dir * d1_scale * dLen + side * d2_scale * dLen;

#ifdef _render_Pylon_Mask
				RenderPylonMask(polygon, _input, StringPrintf("pylonmask%d", i));
#endif
				pylonPolygons[i] = polygon;
			}
		}

		std::vector<int> PylonClassifier::GetPylonCandidate(const Grid2D& grid,
															const std::vector<osg::Vec3d>& polygon,
															double heightThr)
		{
			double T_h = heightThr;

			std::vector<int> indicesTemp;
			osg::BoundingBox bbox = getPolygonBound(polygon);

			for (size_t i = 0; i < grid.size(); ++i)
			{
				const auto& cell = grid.at(i);

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

				for (size_t j = 0; j < cellIndices.size(); ++j)
				{
					const auto& p = _input->points[cellIndices[j]];

					if (pointInPolygon(polygon, p.x, p.y) && p.hag > T_h)
						indicesTemp.push_back(cellIndices[j]);
				}
			}

			// 移除杆塔上方可能存在的成群的噪点
			RemoveNoiseAbove(2.0, indicesTemp);

			return indicesTemp;
		}

		double PylonClassifier::ComputePositionRadius(const std::vector<int>& indices,
													  osg::Vec3d& pos,
													  int post)
		{
			CHECK(!indices.empty());

			double cellsize = _options.cell_size;
			double towerHeadTolerance = cellsize >= 1.0 ? 2.0 : 0.5;
			int towerHeadMinPts = 1000;

			std::vector<int> headIndices = indices;

			// 初始杆塔边界范围内可能会存在少许植被点，过滤它们
			{
				std::vector<std::vector<int>> clusters;

				EuclideanVoxelCluster(_input,
									  headIndices,
									  towerHeadTolerance,
									  towerHeadMinPts,
									  std::numeric_limits<int>::max(),
									  clusters);

				std::string fmt = "[PylonClassifier] 塔头点云类簇数 %d";


				if (!clusters.empty())
				{
					int maxPts = 0;
					std::vector<int> mainCluster;

					fmt += "(";

					for (size_t i = 0; i < clusters.size(); ++i)
					{
						const auto& cluster = clusters.at(i);

						if (cluster.size() > maxPts)
						{
							mainCluster = cluster;
							maxPts = cluster.size();
						}

						fmt += std::to_string(cluster.size());
						if (i != clusters.size() - 1)
							fmt += ", ";
					}

					fmt += ")";

					if (mainCluster.size() > towerHeadMinPts)
						headIndices = mainCluster;
				}

				PCS_DEBUG(fmt.c_str(), clusters.size());
			}

			if (headIndices.empty())
				headIndices = indices;

			// 计算类簇点集在XY平面上的特征值、特征向量
			osg::Vec3d center;
			osg::BoundingBox towerHeadBound;
			std::vector<osg::Vec3d> pts;

			for (size_t i = 0; i < headIndices.size(); ++i)
			{
				const auto& p = _input->points[headIndices[i]];
				osg::Vec3d v(p.x, p.y, p.z);

				towerHeadBound.expandBy(v);
				// center += v / (double)(finalIndices.size());
			}

			// 中心点采用边界框的中心，防止点云分布不均匀导致的偏移
			center = towerHeadBound.center();

			for (size_t i = 0; i < headIndices.size(); ++i)
			{
				const auto& p = _input->points[headIndices[i]];
				osg::Vec3d v = osg::Vec3d(p.x, p.y, p.z) - center;
				v.z() = 0; // 只考虑 XY平面上的特征值、特征向量
				pts.push_back(v);
			}

			osg::Vec3d v1, v2, v3;
			getEigenVectors(pts, v1, v2, v3);
			v1.normalize();

			double maxProj = -DBL_MAX;
			double minProj = DBL_MAX;

			for (size_t i = 0; i < pts.size(); ++i)
			{
				const osg::Vec3d& v = pts.at(i);
				maxProj = std::max(maxProj, v * v1);
				minProj = std::min(minProj, v * v1);
			}

			double projLen = maxProj - minProj + 2.0;
			double pylonRadius = projLen * 0.5;

			PCS_DEBUG("[PylonClassifier] 铁塔点云主方向最大投影长度 %lf，铁塔点云半径 %lf",
					  projLen,
					  pylonRadius);

			/*std::vector<osg::Vec3d> path = { center, center + v1 * pylonRadius };
			RenderLinePath(StringPrintf("pylon_pca_v_%d", post),
							path,
							_input,
							osg::Vec4(1.f, 1.f, 0.f, 1.f));*/

			pos = center;
			return pylonRadius;
		}

		void PylonClassifier::RemoveNoiseAbove(double step, std::vector<int>& indices)
		{
#if 0
			CHECK(step > 0.0);
			double zmin = DBL_MAX;
			double zmax = -DBL_MAX;

			// 最大/最小 Z值
			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& p = _input->points[indices[i]];
				zmin = std::min(zmin, (double)p.z);
				zmax = std::max(zmax, (double)p.z);
			}

			int numBucket = std::floor((zmax - zmin) / step) + 1;
			if (numBucket <= 0)
			{
				PCS_WARN("[RemoveSuspensionNoise] 分配桶失败.");
				return;
			}

			// 粪桶
			std::vector<std::vector<int>> buckets;
			buckets.resize(numBucket);

			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& p = _input->points[indices[i]];
				int index = std::floor(((double)p.z - zmin) / step);
				index = clamp(index, 0, numBucket - 1);
				buckets[index].push_back(indices[i]);
			}

			// 从最低桶网上，遇到空层后停止检索
			std::vector<int> inliners;

			for (size_t i = 0; i < buckets.size(); ++i)
			{
				const auto& bucket = buckets.at(i);
				
				if (bucket.empty())
					break;
				
				inliners.insert(inliners.end(), bucket.begin(), bucket.end());
			}

			indices = inliners;
#else
			int towerHeadMinPts = 1000;
			double tolerance = step;
			std::vector<std::vector<int>> clusters;

			EuclideanVoxelCluster(_input,
								  indices,
								  tolerance,
								  towerHeadMinPts,
								  std::numeric_limits<int>::max(),
								  clusters);

			if (!clusters.empty())
			{
				int maxPts = 0;
				std::vector<int> mainCluster;


				for (size_t i = 0; i < clusters.size(); ++i)
				{
					const auto& cluster = clusters.at(i);

					if (cluster.size() > maxPts)
					{
						mainCluster = cluster;
						maxPts = cluster.size();
					}
				}

				if (mainCluster.size() > towerHeadMinPts)
					indices = mainCluster;
			}
#endif
		}


		VerticalLayers PylonClassifier::CreateVerticalLayers(const std::vector<int>& indices,
															 double lower,
															 double upper,
															 double step)
		{
			int numBins = std::floor((upper - lower) / step) + 1;

			// 垂直方向分层
			VerticalLayers layers;
			layers.resize(numBins);

			for (size_t i = 0; i < indices.size(); ++i)
			{
				auto& p = _input->points[indices[i]];

				if (p.z < lower)
					continue;

				if (p.z >= upper)
				{
					p.label = ePylon;
					continue;
				}

				int index = std::floor(((double)p.z - lower) / step);
				index = clamp(index, 0, numBins - 1);

				layers[index].push_back(indices[i]);
			}

			// 剔除空层
			for (auto iter = layers.begin(); iter != layers.end();)
			{
				if (iter->empty())
					iter = layers.erase(iter);
				else
					++iter;
			}

			return layers;
		}
	}
}
