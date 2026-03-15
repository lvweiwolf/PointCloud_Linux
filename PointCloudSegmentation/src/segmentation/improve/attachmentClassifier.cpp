//stdafx.h
#include "attachmentClassifier.h"

#include <mutex>
#include <osgDB/WriteFile>

#include "../../algorithm/math.h"
#include "../../algorithm/geometry2d_op.h"
#include "../../algorithm/voxel_grid_filter.h"

#include "../../core/private/cloudProcess.h"
#include "../../core/private/rasterProcess.h"
#include "../../core/private/statistics.h"

#include "../../plot/geomCreator.h"
#include "../../plot/plotHandle.h"


// #define RENDER_PYLON_OBB
// #define WRITE_LAS

extern std::string DebugDirectory;

namespace d3s {
	namespace pcs {

		namespace {
			bool HasPylon(PylonGrid3D::CellFeaturePtr featureCell)
			{
				return (featureCell && featureCell->has_pylon);
			}

			bool HasPowerline(PylonGrid3D::CellFeaturePtr featureCell)
			{
				return (featureCell && featureCell->has_powerline);
			}

			// 垂直系数算子
			double Verticality(const std::vector<osg::Vec3d>& pts)
			{
				osg::Vec3d v[3];
				getEigenVectors(pts, v[0], v[1], v[2]);
				double value = std::abs(osg::Z_AXIS * v[0]);
				return value;
			};

			// 球形系数算子
			double Sphericity(const std::vector<osg::Vec3d>& pts)
			{
				double lambda[3] = { 0.0, 0.0, 0.0 };
				getEigenValues(pts, lambda[0], lambda[1], lambda[2]);
				double value = (lambda[0] != 0.0) ? (lambda[2] / lambda[0]) : 0.0;
				return value;
			};

			// 计算特征
			double ComputeFeature(const std::string& method, const std::vector<osg::Vec3d>& pts) 
			{
				if (method == "verticality")
					return Verticality(pts);
				else if (method == "sphericity")
					return Sphericity(pts);
				else
					return 0.0;
			}

			double GetFillRate(const std::vector<osg::Vec3d>& points,
							   int fieldIdx,
							   double step,
							   double& fmin,
							   double& fmax)
			{
				double rate = 0.0;

				if (points.empty())
					return rate;

				fmin = DBL_MAX;
				fmax = -DBL_MAX;

				for (size_t i = 0; i < (int)points.size(); ++i)
				{
					float fvalue = points[i][fieldIdx];

					if (fvalue > fmax)
						fmax = fvalue;

					if (fvalue < fmin)
						fmin = fvalue;
				}

				int numBins = std::floor((fmax - fmin) / step) + 1;

				if (numBins <= 0)
				{
					PCS_WARN("[computeFillrate] 计算仓数量错误.");
					return rate;
				}

				int hit = 0;
				std::vector<int> bins(numBins, 0);

				for (size_t i = 0; i < points.size(); ++i)
				{
					float fvalue = points[i][fieldIdx];

					int index = std::floor(((double)fvalue - fmin) / step);
					index = clamp(index, 0, numBins - 1);

					if (bins[index] == 0)
						hit++;

					bins[index]++;
				}

				return (double)hit / (double)numBins;
			}

			// 悬垂绝缘子格网单元生长条件
			bool SuspensionCondition(const AttachmentClassifyOptions& options,
									 const PylonGrid3D& grid,
									 const PylonGrid3D::Index& index)
			{
				double x_thr = options.x_fillrate;
				double y_thr = options.y_fillrate;
				double radius_thr = options.xy_fillrate;
				double x_length_thr = options.x_length;
				double y_length_thr = options.y_length;

				auto cell = grid.get(index);

				if (!cell || cell->GetSize() <= 0)
					return false;

				auto featureCell = grid.GetFeature(index);

				if (!HasPylon(featureCell))
					return false;

				// X 轴方向比较薄
				bool is_thin_X =
					(featureCell->x_length < x_length_thr || featureCell->x_fillrate < x_thr) &&
					featureCell->xy_fillrate < radius_thr;

				// Y 轴方向比较薄
				bool is_thin_Y =
					featureCell->y_length < y_length_thr || featureCell->y_fillrate < y_thr;

				if (is_thin_X && is_thin_Y)
				{
					return true;
				}
				else
					return false;
			};

			// 耐张绝缘子格网单元生长条件
			bool StrainCondition(const AttachmentClassifyOptions& options,
								 const PylonGrid3D& grid,
								 const PylonGrid3D::Index& index)
			{
				const auto& cell = grid.get(index);

				if (!cell || cell->GetSize() <= 0)
					return false;

				const auto& featureCell = grid.GetFeature(index);

				if (!HasPowerline(featureCell))
					return false;

				return true;
			};

			// 引流线格网单元生长条件
			bool LeadwireCondition(const AttachmentClassifyOptions& options,
								   const PylonGrid3D& grid,
								   const PylonGrid3D::Index& index)
			{
				double x_thr = options.leadwire_x_fr;
				double x_length_thr = options.leadwire_x_len;
				double radius_thr = options.xz_fillrate;

				const auto& cell = grid.get(index);

				if (!cell || cell->GetSize() <= 0)
					return false;

				const auto& featureCell = grid.GetFeature(index);

				if (HasPowerline(featureCell))
					return false;

				bool is_thin_X =
					(featureCell->x_length < x_length_thr || featureCell->x_fillrate < x_thr) &&
					featureCell->xz_fillrate < radius_thr;

				if (is_thin_X)
				{
					return true;
				}
				else
					return false;
			};

			bool IndexLess(const PylonGrid3D::Index& lhs, const PylonGrid3D::Index& rhs) 
			{
				if (lhs[0] < rhs[0])
					return true;
				else if (lhs[0] > rhs[0])
					return false;
				else
				{
					if (lhs[1] < rhs[1])
						return true;
					else if (lhs[1] > rhs[1])
						return false;
					else
						return lhs[2] < rhs[2];
				}
			}
		}


		// PylonGrid3D 杆塔点云格网数据
		//////////////////////////////////////////////////////////////////////////
		PylonGrid3D::PylonGrid3D() : Grid3D() {}

		PylonGrid3D::PylonGrid3D(int xSize, int ySize, int zSize) : Grid3D(xSize, ySize, zSize) {}

		PylonGrid3D::PylonGrid3D(const PylonGrid3D& rhs)
			: Grid3D(rhs), _feature_cells(rhs._feature_cells)
		{
		}

		PylonGrid3D::CellFeaturePtr& PylonGrid3D::GetOrCreateFeature(int xi, int yi, int zi)
		{
			return _feature_cells[{ xi, yi, zi }];
		}

		PylonGrid3D::CellFeaturePtr& PylonGrid3D::GetOrCreateFeature(const Index& index)
		{
			return _feature_cells[index];
		}

		PylonGrid3D::CellFeaturePtr PylonGrid3D::GetFeature(int xi, int yi, int zi) const
		{
			Index index = { xi, yi, zi };

			auto it = _feature_cells.find(index);
			return it == _feature_cells.end() ? nullptr : it->second;
		}

		PylonGrid3D::CellFeaturePtr PylonGrid3D::GetFeature(const Index& index) const
		{
			auto it = _feature_cells.find(index);
			return it == _feature_cells.end() ? nullptr : it->second;
		}

		void PylonGrid3D::Visit(std::function<void(const CellFeaturePtr&)> visitor) const
		{
			if (visitor)
			{
				for (auto it = _feature_cells.begin(); it != _feature_cells.end(); ++it)
					visitor(it->second);
			}
		}

		void PylonGrid3D::Visit(std::function<void(Index)> visitor) const
		{
			if (visitor)
			{
				for (auto it = _feature_cells.begin(); it != _feature_cells.end(); ++it)
					visitor(it->first);
			}
		}

		void PylonGrid3D::ComputeCellFeatures()
		{
			int xSize = _xSize;
			int ySize = _ySize;
			int zSize = _zSize;

			const int X_DIM = 0;
			const int Y_DIM = 1;
			const int Z_DIM = 2;

			double step = 0.1 * cellsize;

			std::map<int, double> x_fillrate_map;
			std::map<int, double> y_fillrate_map;
			std::map<int, double> z_fillrate_map;
			std::map<int, double> x_length_map;
			std::map<int, double> y_length_map;
			std::map<int, double> z_length_map;

			auto visitor = [&](Index index) {
				auto cell = get(index);
				auto featureCell = GetFeature(index);

				if (!cell || !featureCell)
					return;

				if (cell->GetSize() <= 0)
					return;

				// 计算当前单元位置水平方向(X方向)的填充率
				int xi = index[0];
				int yi = index[1];
				int zi = index[2];

				// 计算水平填充率特征
				int idx_yz = zi * ySize + yi;

				if (x_fillrate_map.find(idx_yz) == x_fillrate_map.end())
				{
					double xmin = DBL_MAX;
					double xmax = -DBL_MAX;
					std::vector<osg::Vec3d> points;

					for (int xii = 0; xii < xSize; ++xii)
					{
						auto x_cell = get(xii, yi, zi);

						if (!x_cell || x_cell->GetSize() <= 0)
							continue;

						const auto& x_featureCell = GetFeature(xii, yi, zi);

						// 不计算仅包含电力线的格网在Y轴方向上的“厚度”
						if (!x_featureCell || !HasPylon(x_featureCell))
							continue;

						const auto& cloud = x_cell->GetInput();
						const auto& cellIndices = x_cell->GetIndices();

						for (size_t i = 0; i < cellIndices.size(); ++i)
						{
							const auto& p = cloud->points[cellIndices[i]];
							osg::Vec3d local = rotation.preMult(osg::Vec3d(p.x, p.y, p.z) - center);
							points.push_back(local);
						}
					}

					double x_fill_rate = 0.0;
					double x_length = 0.0;

					if (!points.empty())
					{
						// 10cm 分仓统计填充率
						x_fill_rate = GetFillRate(points, X_DIM, step, xmin, xmax);
						x_length = (xmax - xmin);
					}

					x_length_map[idx_yz] = x_length;
					x_fillrate_map[idx_yz] = x_fill_rate;
				}

				featureCell->x_length = x_length_map[idx_yz];
				featureCell->x_fillrate = x_fillrate_map[idx_yz];

				// 计算Y方向上杆塔点云的“厚度”、填充率
				int idx_xz = zi * xSize + xi;

				if (y_length_map.find(idx_xz) == y_length_map.end())
				{
					double ymin = DBL_MAX;
					double ymax = -DBL_MAX;
					std::vector<osg::Vec3d> points;

					for (int yii = 0; yii < ySize; ++yii)
					{
						auto y_cell = get(xi, yii, zi);

						if (!y_cell || y_cell->GetSize() <= 0)
							continue;

						const auto& y_featureCell = GetFeature(xi, yii, zi);

						// 不计算仅包含电力线的格网在Y轴方向上的“厚度”
						if (!y_featureCell || !HasPylon(y_featureCell))
							continue;

						const auto& cloud = y_cell->GetInput();
						const auto& cellIndices = y_cell->GetIndices();

						for (size_t i = 0; i < cellIndices.size(); ++i)
						{
							const auto& p = cloud->points[cellIndices[i]];
							osg::Vec3d local = rotation.preMult(osg::Vec3d(p.x, p.y, p.z) - center);
							points.push_back(local);
						}
					}

					double y_fill_rate = 0.0;
					double y_length = 0.0;

					if (!points.empty())
					{
						// 10cm 分仓统计填充率
						y_fill_rate = GetFillRate(points, Y_DIM, step, ymin, ymax);
						y_length = (ymax - ymin);
					}

					y_length_map[idx_xz] = y_length;
					y_fillrate_map[idx_xz] = y_fill_rate;
				}

				featureCell->y_length = y_length_map[idx_xz];
				featureCell->y_fillrate = y_fillrate_map[idx_xz];

				// 计算邻域填充率特征
				{
					int hit = 0;
					int nNbr = 2;
					int nNbrZ = 1;

					for (int zii = zi - nNbrZ; zii <= zi + nNbrZ; ++zii)
					{
						for (int yii = yi - nNbr; yii <= yi + nNbr; ++yii)
						{
							for (int xii = xi - nNbr; xii <= xi + nNbr; ++xii)
							{
								if (xii < 0 || xii >= xSize || yii < 0 || yii >= ySize || zii < 0 ||
									zii >= zSize)
								{
									continue;
								}

								const auto& nbrCell = get(xii, yii, zii);

								if (!nbrCell || nbrCell->GetSize() <= 0)
									continue;

								const auto& nbrFeatureCell = GetFeature(xii, yii, zii);

								if (!nbrFeatureCell || !HasPylon(nbrFeatureCell))
									continue;

								++hit;
							}
						}
					}

					int total = (2 * nNbr + 1) * (2 * nNbr + 1) * (2 * nNbrZ + 1);

					double fill_rate = (double)hit / (double)(total);
					featureCell->xy_fillrate = fill_rate;
				}

				// 计算邻域填充率特征
				{
					int hit = 0;
					int nNbr = 2;
					int nNbrY = 2;

					for (int zii = zi - nNbr; zii <= zi + nNbr; ++zii)
					{
						for (int yii = yi - nNbrY; yii <= yi + nNbrY; ++yii)
						{
							for (int xii = xi - nNbr; xii <= xi + nNbr; ++xii)
							{
								if (xii < 0 || xii >= xSize || yii < 0 || yii >= ySize || zii < 0 ||
									zii >= zSize)
								{
									continue;
								}

								const auto& nbrCell = get(xii, yii, zii);

								if (!nbrCell || nbrCell->GetSize() <= 0)
									continue;

								const auto& nbrFeatureCell = GetFeature(xii, yii, zii);

								if (!nbrFeatureCell || !HasPylon(nbrFeatureCell))
									continue;

								++hit;
							}
						}
					}

					int total = (2 * nNbr + 1) * (2 * nNbr + 1) * (2 * nNbrY + 1);

					double fill_rate = (double)hit / (double)(total);
					featureCell->xz_fillrate = fill_rate;
				}
			};

			Visit(visitor);
		}

		// AttachmentClassifier
		//////////////////////////////////////////////////////////////////////////
		AttachmentClassifier::AttachmentClassifier(const AttachmentClassifyOptions& options,
												   PointCloudViewPtr input,
												   const std::vector<int>& indices)
			: _options(options), _input(input), _indices(indices)
		{
			std::array<int, 3> d = { -1, 0, 1 };

			for (int z = 0; z < 3; ++z)
			{
				for (int y = 0; y < 3; ++y)
				{
					for (int x = 0; x < 3; ++x)
					{
						if (x == y && y == z && z == 1)
							continue;

						if (z > 0)
							_GROW_UP.push_back({ d[x], d[y], d[z] }); // 只能向上生长

						if (y > 0)
							_GROW_FRONT.push_back({ d[x], d[y], d[z] }); // 只能向前生长

						if (y < 2)
							_GROW_BACK.push_back({ d[x], d[y], d[z] }); // 只能向后生长

						_GROW_ALL.push_back({ d[x], d[y], d[z] }); // 27邻域
					}
				}
			}
		}

		AttachmentClassifier::~AttachmentClassifier() {}

		void AttachmentClassifier::SetPylonPositions(const Positions& pylonPositions)
		{
			_pylonPositions = pylonPositions;
		}

		void AttachmentClassifier::SetPylonPolygons(const Polygons& pylonPolygons)
		{
			_pylonPolygons = pylonPolygons;
		}

		void AttachmentClassifier::SetPylonSides(const Vectors& pylonSides)
		{
			_pylonSides = pylonSides;
		}

		void AttachmentClassifier::SetPylonConnections(const PylonConnections& pylonConnections)
		{
			_pylonConnections = pylonConnections;
		}


		void AttachmentClassifier::Segment()
		{
			if (!_options.enable)
			{
				PCS_WARN("[AttachmentClassifier] 绝缘子分类未启用，跳过绝缘子分类.");
				return;
			}

			if (_pylonPositions.empty() || _pylonPolygons.empty())
				return;

			if (_pylonPositions.size() < 2)
				return;

			CHECK(_pylonPositions.size() == _pylonPolygons.size());
			CHECK(_pylonPositions.size() == _pylonConnections.size());

			double cellsize = 5.0;
			double voxelsize = _options.voxel_size;

			// 处理每一个杆塔
			const Positions& pylonPositions = _pylonPositions;
			const Polygons& pylonPolygons = _pylonPolygons;
			const PylonConnections& pylonConnections = _pylonConnections;

			// 创建局部格网
			Grid2D grid;
			ComputeLocalGridCells(cellsize, _input, _indices, grid);

//#ifdef _OPENMP
//#pragma omp parallel for
//#endif
			for (int i = 0; i < (int)pylonPositions.size(); ++i)
			{
				const auto& pos = pylonPositions.at(i);
				const auto& polygon = pylonPolygons.at(i);
				PylonConnection connection = pylonConnections.at(i);

				if (connection.empty())
					continue;

				// 获得杆塔边界内的杆塔点云索引
				std::vector<int> pylonIndices, powerlineIndices;
				GetIndices(grid, polygon, pylonIndices, powerlineIndices);

				std::vector<int> indices;
				indices.insert(indices.end(), pylonIndices.begin(), pylonIndices.end());
				indices.insert(indices.end(), powerlineIndices.begin(), powerlineIndices.end());

				osg::Vec3d center;
				osg::Matrix rotation;
				osg::BoundingBox obb;

				// 获得杆塔OBB包围盒
				GetPylonOBB(pylonIndices, powerlineIndices, i, obb, center, rotation);

				PylonGrid3D voxels;
				ComputePylonGrid(voxelsize, center, obb, rotation, _input, indices, voxels);

				// 计算格网单元特征值
				voxels.ComputeCellFeatures();

				// WARNING: 移除最顶部的连接点
				RemoveConnectionIgnored(pos, connection);

				int numCluster0 = 0;
				int numCluster1 = 0;
				int numCluster2 = 0;

				std::vector<int> powerlines = GetPowerlineSurround(voxels, connection);
				std::vector<int> leadwires;

				// 处理每个连接点位置
				for (int c = 0; c < connection.size(); ++c)
				{
					const auto& connectPoint = connection[c];

					PylonGrid3D::Index index;

					// 获得离连接点位置最近
					if (!GetNearestIndex(voxels, connectPoint, index, HasPylon))
					{
						PCS_WARN("[AttachmentClassifier] 杆塔 %d 的连接点 %d 处不存在悬垂串候选单元.",
								 i,
								 c);

						continue;
					}

					// 识别悬垂串：格网索引作为种子，自下向上区域生长
					AttachmentCluster cluster =
						Growing(voxels, index, _GROW_UP, SuspensionCondition);

					if (!cluster.empty())
					{
						++numCluster0;

						if (IsSuspension(voxels, cluster, connectPoint))
						{
							++numCluster1;
							SetClusterLabel(voxels, cluster, 1); // 设置格网单元类标签
							std::vector<int> attachments = GetIndices(voxels, cluster);

							// 修复距离电力特别近的点为绝缘子类别
							FixSuspensionAttachment(voxels, powerlines, attachments);

							// 设置点云类型
							for (size_t j = 0; j < attachments.size(); ++j)
							{
								auto& p = _input->points[attachments[j]];
								p.label = eIsulator;
							}
						}
						else if (IsStrain(voxels, cluster))
						{
							++numCluster2;
							// 重置索引和类簇
							index = PylonGrid3D::Index();

							if (GetNearestIndex(voxels, connectPoint, index, HasPowerline))
							{
								std::vector<int> attachments;
								std::set<int> strainMask;
								AttachmentCluster strain, leadwire;
								std::vector<AttachmentCluster> suspensions;

								// 耐张绝缘子串细化分类
								SegmentStrain(voxels,
											  index,
											  strain,
											  leadwire,
											  suspensions,
											  strainMask);

								// 获得耐张绝缘子点云
								attachments = GetIndices(voxels, strain);

								// 设置绝缘子点云类别
								if (!attachments.empty())
								{
									// 设置耐张绝缘子点云类型
									for (size_t j = 0; j < attachments.size(); ++j)
									{
										auto& p = _input->points[attachments[j]];

										if (p.label == ePowerline)
											p.label = eIsulator;
									};

									// 修复为被成功分类为绝缘子的电力线点云
									std::vector<int> nearPowerlines;
									FixPowerlineNearConnectPoint(voxels,
																 connectPoint,
																 voxelsize,
																 nearPowerlines);

									for (size_t j = 0; j < nearPowerlines.size(); ++j)
									{
										auto& p = _input->points[nearPowerlines[j]];
										p.label = ePylon;
									};
								}

								FixLeadwires(voxels, index, leadwire, suspensions, strainMask);
							}
							else
							{
								PCS_WARN("[AttachmentClassifier] 杆塔 %d 的连接点 %d "
										 "处不存在耐张传候选单元.",
										 i,
										 c);
							}
						}
						else 
						{
							SetClusterLabel(voxels, cluster, 0); // 设置格网单元类标签
						}
					}
				}

				PCS_DEBUG("[AttachmentClassifier] 检测到杆塔 %d 绝缘子类簇 %d -> %d(%d 耐张).",
						  i,
						  numCluster0,
						  numCluster1,
						  numCluster2);

#ifdef RENDER_PYLON_OBB
				RenderPylonGrid(voxels, _input);
#endif

#ifdef WRITE_LAS

				/*if (!leadwires.empty())
				{
					std::string filename = DebugDirectory + StringPrintf("leadwire_%d.las", i);
					PCS_DEBUG("[AttachmentClassifier] 正在写入 %s.", filename.c_str());
					writePointCloud(filename, *_input, leadwires);
				}*/

				if (!pylonIndices.empty())
				{
					std::vector<int> bodyIndices;
					bodyIndices.reserve(pylonIndices.size());

					for (size_t j = 0; j < pylonIndices.size(); ++j)
					{
						const auto& p = _input->points[pylonIndices[j]];

						if (p.label == ePylon)
							bodyIndices.push_back(pylonIndices[j]);
					}

					bodyIndices.shrink_to_fit();

					std::string filename = DebugDirectory + StringPrintf("pylon_%d.las", i);
					PCS_DEBUG("[AttachmentClassifier] 正在写入 %s.", filename.c_str());
					writePointCloud(filename, *_input, bodyIndices);
				}
#endif
			}
		}


		void AttachmentClassifier::SegmentStrain(const PylonGrid3D& grid,
												 const PylonGrid3D::Index& index,
												 AttachmentCluster& strain,
												 AttachmentCluster& leadwire,
												 std::vector<AttachmentCluster>& suspensions,
												 std::set<int>& strainMask)
		{
			int expand = _options.ignore_mask_distance;
			int range = _options.connection_mask_distance;
			int xSize = grid.getXSize();
			int ySize = grid.getYSize();
			int zSize = grid.getZSize();

			AttachmentCluster clusters[2];
			GrowDirection GROW_DIR[2] = { _GROW_FRONT, _GROW_BACK };
			GrowDirection GROW_DIR_INV[2] = { _GROW_BACK, _GROW_FRONT };

			strainMask.clear();

			for (int i = 0; i < 2; ++i) 
			{
				clusters[i] = Growing(grid, index, GROW_DIR[i], StrainCondition);

				for (size_t j = 0; j < clusters[i].size(); ++j) 
				{
					const auto& sIndex = clusters[i][j];
					size_t idx = sIndex[2] * xSize * ySize + sIndex[1] * xSize + sIndex[0];
					strainMask.insert(idx);
				}
			}

			// 引流线提取
			{
				leadwire.clear();

				for (int i = 0; i < 2; ++i)
				{
					AttachmentCluster leadwireSeeds, temp;

					// 获得引流线候选格网单元
					leadwireSeeds = RemoveClusterInRadius(clusters[i], index, range);

					// 反向生长
					temp = Growing(grid, leadwireSeeds, GROW_DIR_INV[i], LeadwireCondition, true);

					// WARNING: 可能存在重复
					leadwire.insert(leadwire.end(), temp.begin(), temp.end());
				}

				std::sort(leadwire.begin(), leadwire.end(), IndexLess);

				// 去除重复
				leadwire.erase(std::unique(leadwire.begin(),
										   leadwire.end(),
										   [](const auto& lhs, const auto& rhs) {
											   return lhs[0] == rhs[0] && lhs[1] == rhs[1] &&
													  lhs[2] == rhs[2];
										   }),
							   leadwire.end());



				// 从引流线格网中提取可能存在的悬垂绝缘子串
				AttachmentCluster seeds;
				GetSuspensionSeeds(grid, leadwire, seeds);

#if 1
				suspensions.clear();
				AttachmentClustering(grid, seeds, suspensions);

				for (int i = (int)suspensions.size() - 1; i >= 0; --i) 
				{
					const auto& suspension = suspensions.at(i);

					if (!IsSuspension(grid, suspension)) 
					{
						leadwire.insert(leadwire.end(), suspension.begin(), suspension.end());
						suspensions.erase(suspensions.begin() + i);
					}
				}

				for (size_t i = 0; i < suspensions.size(); ++i)
				{
					const auto& suspension = suspensions.at(i);

					int xmin = xSize;
					int ymin = ySize;
					int zmin = zSize;
					int xmax = 0;
					int ymax = 0;
					int zmax = 0;

					for (size_t j = 0; j < suspension.size(); ++j)
					{
						const auto& sIndex = suspension.at(j);

						xmin = std::min(xmin, sIndex[0]);
						ymin = std::min(ymin, sIndex[1]);
						zmin = std::min(zmin, sIndex[2]);
						xmax = std::max(xmax, sIndex[0]);
						ymax = std::max(ymax, sIndex[1]);
						zmax = std::max(zmax, sIndex[2]);
					}

					for (int j = (int)leadwire.size() - 1; j >= 0; --j)
					{
						const auto& leadwireIndex = leadwire.at(j);
						int xj = leadwireIndex[0];
						int yj = leadwireIndex[1];
						int zj = leadwireIndex[2];

						// 排除挂串上方一定范围内的塔体部分
						if (zj >= zmax && zj < zmax + expand) 
						{
							if (xj > xmin - expand && xj < xmax + expand && yj > ymin - expand &&
								yj < ymax + expand)
							{
								leadwire.erase(leadwire.begin() + j);
							}
						}
					}

					SetClusterLabel(grid, suspension, 1);
				}
#endif
				SetClusterLabel(grid, leadwire, 2);
			}

#if 1
			// 耐张绝缘子串提取
			{
				strain.clear();

				for (int i = 0; i < 2; ++i)
				{
					// 修复耐张绝缘子串
					FixStrainAttachment(grid, clusters[i]);
					strain.insert(strain.end(), clusters[i].begin(), clusters[i].end());
				}

				std::sort(strain.begin(), strain.end(), IndexLess);

				// 去除重复
				strain.erase(std::unique(strain.begin(),
										 strain.end(),
										 [](const auto& lhs, const auto& rhs) {
											 return lhs[0] == rhs[0] && lhs[1] == rhs[1] &&
													lhs[2] == rhs[2];
										 }),
							 strain.end());

				SetClusterLabel(grid, strain, 1);
			}
#endif
		}

		void AttachmentClassifier::SetClusterLabel(const PylonGrid3D& grid,
												   const AttachmentCluster& cluster,
												   int label)
		{
			for (size_t i = 0; i < cluster.size(); ++i)
			{
				const auto& idx = cluster.at(i);
				auto featureCell = grid.GetFeature(idx);
				CHECK(featureCell.get());

				featureCell->cluster_id = label;
			}
		}

		void AttachmentClassifier::GetIndices(const Grid2D& grid,
											  const Polygon& polygon,
											  std::vector<int>& pylonIndices,
											  std::vector<int>& powerlineIndices)
		{
			pylonIndices.clear();
			powerlineIndices.clear();
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

					if (!pointInPolygon(polygon, p.x, p.y))
						continue;

					if (p.label == ePylon)
						pylonIndices.push_back(cellIndices[j]);

					if (p.label == ePowerline)
						powerlineIndices.push_back(cellIndices[j]);
				}
			}
		}


		std::vector<int> AttachmentClassifier::GetIndices(const PylonGrid3D& grid,
														  const AttachmentCluster& cluster)
		{
			std::vector<int> attachments;

			for (size_t i = 0; i < cluster.size(); ++i)
			{
				const auto& index = cluster.at(i);
				const auto& cell = grid.get(index);

				if (!cell || cell->GetSize() <= 0)
					continue;

				attachments.insert(attachments.end(),
								   cell->GetIndices().begin(),
								   cell->GetIndices().end());
			}

			return attachments;
		}

		std::vector<int> AttachmentClassifier::GetPowerlineSurround(
			const PylonGrid3D& grid,
			const PylonConnection& connection)
		{
			double distanceThr = grid.cellsize;
			std::vector<int> powerlines;

			for (int c = 0; c < connection.size(); ++c)
			{
				osg::Vec3d v = grid.rotation.preMult(connection[c] - grid.center);

				auto visitor = [&](PylonGrid3D::Index index) {
					const auto& cell = grid.get(index);

					if (!cell || cell->GetSize() <= 0)
						return;

					const auto& o = cell->center();

					if ((o - v).length() > 4.0 * distanceThr)
						return;

					const auto& cellIndices = cell->GetIndices();

					for (size_t i = 0; i < cellIndices.size(); ++i)
					{
						const auto& p = _input->points[cellIndices[i]];

						if (p.label == ePowerline)
							powerlines.push_back(cellIndices[i]);
					}
				};

				grid.Visit(visitor);
			}

			return powerlines;
		}

		void AttachmentClassifier::GetPylonOBB(const std::vector<int>& pylonIndices,
											   const std::vector<int>& powerlineIndices,
											   int idx,
											   osg::BoundingBox& obb,
											   osg::Vec3d& center,
											   osg::Matrix& rotation)
		{
			const auto& pylonSides = _pylonSides;

			osg::Vec3d side = pylonSides.at(idx);

			center = osg::Vec3d(); // 默认使用原点，因为经过偏移，点云坐标数量级并不大
			rotation = osg::Matrix::rotate(side, osg::Vec3d(1.0, 0.0, 0.0));
			obb.init();

			// 计算OBB包围盒
			for (size_t i = 0; i < pylonIndices.size(); ++i)
			{
				const auto& p = _input->points[pylonIndices[i]];
				osg::Vec3d local = rotation.preMult(osg::Vec3d(p.x, p.y, p.z) - center);
				obb.expandBy(local);
			}

			for (size_t i = 0; i < powerlineIndices.size(); ++i)
			{
				const auto& p = _input->points[powerlineIndices[i]];
				osg::Vec3d local = rotation.preMult(osg::Vec3d(p.x, p.y, p.z) - center);
				obb.expandBy(local);
			}
		}

		void AttachmentClassifier::RemoveConnectionIgnored(const osg::Vec3d& pos,
														   PylonConnection& connection)
		{
			// 按垂直方向排序
			std::sort(
				connection.begin(),
				connection.end(),
				[&](const osg::Vec3d& lhs, const osg::Vec3d& rhs) { return lhs.z() < rhs.z(); });


			// 合并垂直方向距离接近的点
			double zThr = 2.0 * _options.voxel_size;
			std::vector<std::vector<size_t>> clusters;
			std::set<size_t> processed;

			size_t numConnections = connection.size();
			for (size_t i = 0; i < numConnections; ++i)
			{
				if (processed.find(i) != processed.end())
					continue;

				std::vector<size_t> seed_queue;

				int sq_idx = 0;
				seed_queue.push_back(i);
				processed.insert(i);

				while (sq_idx < (int)seed_queue.size())
				{
					size_t j = seed_queue[sq_idx];
					const auto& p = connection[j];

					// 遍历相邻接节点
					for (int k = (int)j - 1; k <= (int)j + 1; ++k)
					{
						// 565503 自动分类：磁盘剩余空间小于工程导入的las文件时，自动分类软件概率性闪退 V1.6.0.63
						if (k < 0 || k >= numConnections)
							continue;

						if (processed.find(k) != processed.end())
							continue;

						const auto& q = connection[k];

						if (fabs(q.z() - p.z()) < zThr)
						{
							seed_queue.push_back(k);
							processed.insert(k);
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

				if (i == 0 && cluster.size() == 1)
					continue;

				if (i == clusters.size() - 1)
				{
					double zmean = 0.0;

					for (size_t j = 0; j < cluster.size(); ++j)
					{
						const auto& connectInst = connection.at(cluster[j]);
						zmean += connectInst.z() / (double)cluster.size();
					}

					if (zmean > pos.z())
						continue;
				}

				for (size_t j = 0; j < cluster.size(); ++j)
				{
					const auto& connectInst = connection.at(cluster[j]);
					newConnection.push_back(connectInst);
				}
			}

			std::swap(connection, newConnection);
		}

		AttachmentCluster AttachmentClassifier::RemoveClusterInRadius(
			const AttachmentCluster& cluster,
			const PylonGrid3D::Index& index,
			int distance)
		{
			AttachmentCluster clusterFiltered;

			for (size_t i = 0; i < cluster.size(); ++i)
			{
				const auto& cellIndex = cluster.at(i);

				int dx = std::abs(cellIndex[0] - index[0]);
				int dy = std::abs(cellIndex[1] - index[1]);


				if (dx < distance && dy < distance)
					continue;

				clusterFiltered.push_back(cellIndex);
			}

			return clusterFiltered;
		}


		bool AttachmentClassifier::FindNeighbors(const PylonGrid3D& grid,
												 int numNeighbor,
												 PylonGrid3D::Index& index,
												 FeatureCondition condition)
		{
			bool bFind = false;
			int nNbr = numNeighbor;
			int xi = index[0];
			int yi = index[1];
			int zi = index[2];

			int xSize = grid.getXSize();
			int ySize = grid.getYSize();
			int zSize = grid.getZSize();

			std::vector<PylonGrid3D::Index> neighbors;

			for (int zii = zi; zii <= zi + nNbr; ++zii)
			{
				for (int yii = yi - nNbr; yii <= yi + nNbr; ++yii)
				{
					for (int xii = xi - nNbr; xii <= xi + nNbr; ++xii)
					{
						if (xii == xi && yii == yi && zii == zi)
							continue;

						int xx = clamp(xii, 0, xSize - 1);
						int yy = clamp(yii, 0, ySize - 1);
						int zz = clamp(zii, 0, zSize - 1);

						neighbors.push_back({ xx, yy, zz });
					}
				}
			}

			// 按照距离排序
			std::sort(neighbors.begin(),
					  neighbors.end(),
					  [&](const PylonGrid3D::Index& lhs, const PylonGrid3D::Index& rhs) {
						  int distance0 = (lhs[0] - xi) * (lhs[0] - xi) +
										  (lhs[1] - yi) * (lhs[1] - yi) +
										  (lhs[2] - zi) * (lhs[2] - zi);

						  int distance1 = (rhs[0] - xi) * (rhs[0] - xi) +
										  (rhs[1] - yi) * (rhs[1] - yi) +
										  (rhs[2] - zi) * (rhs[2] - zi);

						  return distance0 < distance1;
					  });


			// 按距离顺序，依次向外查找
			for (size_t i = 0; i < neighbors.size(); ++i)
			{
				int xii = neighbors[i][0];
				int yii = neighbors[i][1];
				int zii = neighbors[i][2];

				const auto& cell = grid.get(xii, yii, zii);

				if (!cell || cell->GetSize() <= 0)
					continue;

				const auto& featureCell = grid.GetFeature(xii, yii, zii);

				if (condition && !condition(featureCell))
					continue;

				xi = xii;
				yi = yii;
				zi = zii;
				bFind = true;
				break;
			}

			index = { xi, yi, zi };

			return bFind;
		}

		bool AttachmentClassifier::GetNearestIndex(const PylonGrid3D& grid,
												   const osg::Vec3d& connectPoint,
												   PylonGrid3D::Index& index,
												   FeatureCondition condition)
		{

			osg::Vec3d local = grid.rotation.preMult(connectPoint - grid.center);

			int xSize = grid.getXSize();
			int ySize = grid.getYSize();
			int zSize = grid.getZSize();

			int xi = std::floor((local.x() - grid.obb.xMin()) / grid.cellsize);
			int yi = std::floor((local.y() - grid.obb.yMin()) / grid.cellsize);
			int zi = std::floor((local.z() - grid.obb.zMin()) / grid.cellsize);
			xi = clamp(xi, 0, xSize - 1);
			yi = clamp(yi, 0, ySize - 1);
			zi = clamp(zi, 0, zSize - 1);

			PylonGrid3D::Index seed = { xi, yi, zi };

			return GetNearestIndex(grid, seed, index, condition);
		}

		bool AttachmentClassifier::GetNearestIndex(const PylonGrid3D& grid,
												   const PylonGrid3D::Index& index,
												   PylonGrid3D::Index& nearestIndex,
												   FeatureCondition condition)
		{
			int search_radius = 3; // 邻域搜索范围

			auto cell = grid.get(index);
			auto featureCell = grid.GetFeature(index);

			PylonGrid3D::Index tempIndex = index;

			if (!cell || cell->GetSize() <= 0 || (condition && !condition(featureCell)))
			{
				if (FindNeighbors(grid, search_radius, tempIndex, condition))
				{
					cell = grid.get(tempIndex);
				}
			}

			if (!cell || cell->GetSize() <= 0)
			{
				return false;
			}
			else
			{
				nearestIndex = tempIndex;
				return true;
			}
		}

		bool AttachmentClassifier::GetFarthestIndex(const PylonGrid3D& grid,
													const AttachmentCluster& cluster,
													const osg::Vec3d& connectPoint,
													PylonGrid3D::Index& farthestIndex)
		{
			if (cluster.empty())
				return false;

			osg::Vec3d local = grid.rotation.preMult(connectPoint - grid.center);
			double distance = 0.0;

			for (size_t i = 0; i < cluster.size(); ++i)
			{
				const auto& index = cluster.at(i);
				const auto& cell = grid.get(index);

				if (!cell || cell->GetSize() <= 0)
					continue;

				double d = (local - cell->center()).length();

				if (d > distance)
				{
					farthestIndex = index;
					distance = d;
				}
			}

			const auto& cell = grid.get(farthestIndex);
			return (cell && cell->GetSize() > 0);
		}

		void AttachmentClassifier::GetSuspensionSeeds(const PylonGrid3D& grid,
												AttachmentCluster& cluster,
												AttachmentCluster& suspension)
		{
			if (cluster.empty())
				return;

			double radius = _options.search_radius;
			double suspensionThr = _options.suspension_thr;
			double subsampleSize = _options.subsample_size;
			std::string method = _options.suspension_method;
			int numPtsThr = 20;

			// 5cm 抽稀
			std::vector<PointCloudView<PointPCLH>::PointCloud> clouds;

			if (subsampleSize > 0.0 && subsampleSize < grid.cellsize)
			{
				clouds.resize(cluster.size());

				for (size_t i = 0; i < cluster.size(); ++i)
				{
					const auto& cell = grid.get(cluster.at(i));

					if (!cell || cell->GetSize() <= 0)
						continue;

					const auto& cellIndices = cell->GetIndices();
					const auto& bbox = *cell;

					for (size_t j = 0; j < cellIndices.size(); ++j)
					{
						const auto& p = _input->points[cellIndices[j]];
						clouds[i].push_back(p);
					}

					// 体素抽稀
					VoxelGridSample(clouds[i],
									Eigen::Vector3f(bbox.xMin(), bbox.yMin(), bbox.zMin()),
									Eigen::Vector3f(bbox.xMax(), bbox.yMax(), bbox.zMax()),
									subsampleSize);
				}
			}

			CHECK(clouds.size() == cluster.size());

			std::vector<double> features;
			std::vector<int> cloudIndexs;
			std::vector<int> numbers(clouds.size(), 0);
			std::vector<osg::Vec3d> pts;

			for (size_t i = 0; i < clouds.size(); ++i)
			{
				const auto& points = clouds.at(i);

				if (points.empty())
					continue;

				for (size_t j = 0; j < points.size(); ++j)
				{
					const auto& p = points[j];
					pts.push_back(osg::Vec3d(p.x, p.y, p.z));
					cloudIndexs.push_back(i);
				}
			}

			// 计算点云特征
			{
				features.resize(pts.size(), 0.0);

				// 计算各点半径范围内的邻域点
				std::vector<std::vector<int>> neighborsList;
				std::vector<std::vector<float>> distancesList;
				radiusSearch(pts, radius, neighborsList, distancesList);

				CHECK(neighborsList.size() == pts.size());

				for (size_t i = 0; i < pts.size(); ++i)
				{
					const auto& nn_indices = neighborsList[i];

					std::vector<osg::Vec3d> neighbors;
					neighbors.reserve(nn_indices.size());

					for (size_t j = 0; j < nn_indices.size(); ++j)
					{
						const auto& p = pts[nn_indices[j]];
						neighbors.push_back(p);
					}

					features[i] = ComputeFeature(method, neighbors);
				}
			}

			CHECK(features.size() == pts.size());
			CHECK(features.size() == cloudIndexs.size());

			// 统计格网单元内符合条件的点数量
			for (size_t i = 0; i < features.size(); ++i)
			{
				if (features[i] > suspensionThr)
					numbers[cloudIndexs[i]]++;
			}

			// 统计杆塔格网单元内点数量离群值阈值
			std::vector<int> numbersNoEmpty;
			for (size_t i = 0; i < numbers.size(); ++i)
			{
				const auto& points = clouds.at(i);

				if (points.empty())
					continue;

				if (numbers[i] > 0)
					numbersNoEmpty.push_back(numbers[i]);
			}

			numPtsThr = numbersNoEmpty.empty() ? 0 : (int)Mean(numbersNoEmpty);

			AttachmentCluster seeds; // 满足垂直系数的格网，作为检测引流线挂串的种子
			AttachmentCluster leadwire;

			for (size_t i = 0; i < cluster.size(); ++i)
			{
				const auto& points = clouds.at(i);

				if (points.empty())
					continue;

				if (numbers[i] > numPtsThr) // 满足条件的杆塔格网单元
				{
					seeds.push_back(cluster.at(i));
				}
				else
				{
					leadwire.push_back(cluster.at(i));
				}
			}

			cluster = leadwire;
			suspension = seeds;
		}

		AttachmentCluster AttachmentClassifier::Growing(const PylonGrid3D& grid,
														const PylonGrid3D::Index& seedIndex,
														const GrowDirection& growDirect,
														const GrowCondition& condition)
		{
			int xSize = grid.getXSize();
			int ySize = grid.getYSize();
			int zSize = grid.getZSize();

			auto currentCell = grid.get(seedIndex);
			auto featureCell = grid.GetFeature(seedIndex);

			if (!currentCell || currentCell->GetSize() <= 0)
				return AttachmentCluster();

			size_t i = seedIndex[2] * xSize * ySize + seedIndex[1] * xSize + seedIndex[0];

			std::set<size_t> processed;
			AttachmentCluster seed_queue;

			int sq_idx = 0;
			seed_queue.push_back(seedIndex);
			processed.insert(i);

			while (sq_idx < (int)seed_queue.size())
			{
				PylonGrid3D::Index index = seed_queue[sq_idx];
				auto cell = grid.get(index[0], index[1], index[2]);

				if (!cell || cell->GetSize() <= 0)
				{
					sq_idx++;
					continue;
				}

				// 遍历相邻接节点
				for (size_t oi = 0; oi < growDirect.size(); ++oi)
				{
					const auto& offset = growDirect[oi];
					int xii = clamp(index[0] + offset[0], 0, xSize - 1);
					int yii = clamp(index[1] + offset[1], 0, ySize - 1);
					int zii = clamp(index[2] + offset[2], 0, zSize - 1);
					size_t j = zii * xSize * ySize + yii * xSize + xii;

					if (processed.find(j) != processed.end())
						continue;

					auto cellNbr = grid.get(xii, yii, zii);

					if (!cellNbr || cellNbr->GetSize() <= 0)
						continue;

					if (condition && !condition(_options, grid, { xii, yii, zii }))
						continue;

					seed_queue.push_back({ xii, yii, zii });
					processed.insert(j);
				}

				sq_idx++;

			} // while 结束

			return seed_queue;
		}

		AttachmentCluster AttachmentClassifier::Growing(const PylonGrid3D& grid,
														const AttachmentCluster& seeds,
														const GrowDirection& growDirect,
														const GrowCondition& condition,
														bool ignoreSeed)
		{
			int xSize = grid.getXSize();
			int ySize = grid.getYSize();
			int zSize = grid.getZSize();

			AttachmentCluster cluster;
			std::set<size_t> processed;

			for (size_t i = 0; i < seeds.size(); ++i)
			{
				const auto& seedIndex = seeds.at(i);
				size_t si = seedIndex[2] * xSize * ySize + seedIndex[1] * xSize + seedIndex[0];

				if (processed.find(si) != processed.end())
					continue;

				AttachmentCluster seed_queue;

				int sq_idx = 0;
				processed.insert(si);
				seed_queue.push_back(seedIndex);

				if (!ignoreSeed)
					cluster.push_back(seedIndex);

				while (sq_idx < (int)seed_queue.size())
				{
					PylonGrid3D::Index index = seed_queue[sq_idx];
					auto cell = grid.get(index[0], index[1], index[2]);

					if (!cell || cell->GetSize() <= 0)
					{
						sq_idx++;
						continue;
					}

					// 遍历相邻接节点
					for (size_t oi = 0; oi < growDirect.size(); ++oi)
					{
						const auto& offset = growDirect[oi];
						int xii = clamp(index[0] + offset[0], 0, xSize - 1);
						int yii = clamp(index[1] + offset[1], 0, ySize - 1);
						int zii = clamp(index[2] + offset[2], 0, zSize - 1);
						size_t sj = zii * xSize * ySize + yii * xSize + xii;

						if (processed.find(sj) != processed.end())
							continue;

						auto cellNbr = grid.get(xii, yii, zii);

						if (!cellNbr || cellNbr->GetSize() <= 0)
							continue;

						if (condition && !condition(_options, grid, { xii, yii, zii }))
							continue;

						processed.insert(sj);
						seed_queue.push_back({ xii, yii, zii });
						cluster.push_back({ xii, yii, zii });
					}

					sq_idx++;

				} // while 结束
			}

			return cluster;
		}

		bool AttachmentClassifier::IsSuspension(const PylonGrid3D& grid,
												const AttachmentCluster& cluster,
												const osg::Vec3d& connectPoint) const
		{
			double lengthThr = _options.z_length;
			double dmax = -DBL_MAX;

			for (size_t i = 0; i < cluster.size(); ++i)
			{
				const auto& cell = grid.get(cluster.at(i));

				if (!cell || cell->GetSize() <= 0)
					continue;

				const auto& cellIndices = cell->GetIndices();

				for (size_t j = 0; j < cellIndices.size(); ++j)
				{
					const auto& p = _input->points[cellIndices[j]];
					dmax = std::max(dmax, (double)p.z - connectPoint.z());
				}
			}

			return dmax >= lengthThr;
		}

		bool AttachmentClassifier::IsSuspension(const PylonGrid3D& grid,
												const AttachmentCluster& cluster) const
		{
			double suspensionThr = _options.suspension_thr;
			std::string method = _options.suspension_method;
			std::vector<osg::Vec3d> pts;

			for (size_t i = 0; i < cluster.size(); ++i)
			{
				const auto& cell = grid.get(cluster.at(i));

				if (!cell || cell->GetSize() <= 0)
					continue;

				const auto& cellIndices = cell->GetIndices();

				for (size_t j = 0; j < cellIndices.size(); ++j)
				{
					const auto& p = _input->points[cellIndices[j]];
					pts.push_back(osg::Vec3d(p.x, p.y, p.z));
				}
			}

			
			double verticality = ComputeFeature(method, pts);

			return verticality > suspensionThr;
		}

		bool AttachmentClassifier::IsStrain(const PylonGrid3D& grid,
											const AttachmentCluster& cluster)
		{
			double x_thr = _options.x_fillrate;
			double y_thr = _options.y_fillrate;
			double x_length_thr = _options.x_length;
			double y_length_thr = _options.y_length;
			double radius_thr = _options.xy_fillrate;

			for (size_t i = 0; i < cluster.size(); ++i)
			{
				const auto& cell = grid.get(cluster.at(i));

				if (!cell || cell->GetSize() <= 0)
					continue;

				const auto& featureCell = grid.GetFeature(cluster.at(i));

				if (!HasPylon(featureCell))
					continue;

				// X 轴方向比较薄
				bool is_thin_X =
					(featureCell->x_length < x_length_thr || featureCell->x_fillrate < x_thr) &&
					(featureCell->xy_fillrate < radius_thr);

				// Y 轴方向比较薄
				bool is_thin_Y =
					(featureCell->y_length < y_length_thr) || (featureCell->y_fillrate < y_thr); 

				if (!is_thin_X || !is_thin_Y)
					return true;
			}

			return false;
		}

		void AttachmentClassifier::FixSuspensionAttachment(const PylonGrid3D& grid,
														   const std::vector<int>& powerlines,
														   std::vector<int>& attachments)
		{
			if (attachments.empty())
				return;

			double radius = _options.voxel_size;

			// 计算绝缘子类簇的最低点坐标
			double xmean = 0.0;
			double ymean = 0.0;
			double zmin = DBL_MAX;
			double numInverse = 1.0 / (double)attachments.size();

			for (size_t i = 0; i < attachments.size(); ++i)
			{
				const auto& p = _input->points[attachments[i]];

				xmean += p.x * numInverse;
				ymean += p.y * numInverse;
				zmin = std::min(zmin, (double)p.z);
			}

			int numFix = 0;

			// 修正一些电力线点，靠近悬垂绝缘子的电力线点类别设置为绝缘子
			for (size_t i = 0; i < powerlines.size(); ++i)
			{
				const auto& p = _input->points[powerlines[i]];

				if (p.z > zmin)
					continue;

				double distance =
					std::sqrt((p.x - xmean) * (p.x - xmean) + (p.y - ymean) * (p.y - ymean));

				if (distance <= radius && (zmin - p.z) < 2.0 * radius)
				{
					attachments.push_back(powerlines[i]);
					++numFix;
				}
			}
		}

		void AttachmentClassifier::FixStrainAttachment(const PylonGrid3D& grid,
													   AttachmentCluster& cluster)
		{
			if (cluster.empty())
				return;

			double radius = _options.search_radius;
			double strainThr = _options.strain_thr;
			double subsampleSize = _options.subsample_size;
			std::string method = _options.strain_method;
			int numPtsThr = 20;

			// 5cm 抽稀
			std::vector<PointCloudView<PointPCLH>::PointCloud> clouds;

			if (subsampleSize > 0.0 && subsampleSize < grid.cellsize)
			{
				clouds.resize(cluster.size());

				for (size_t i = 0; i < cluster.size(); ++i)
				{
					const auto& cell = grid.get(cluster.at(i));

					if (!cell || cell->GetSize() <= 0)
						continue;

					const auto& cellIndices = cell->GetIndices();
					const auto& bbox = *cell;

					for (size_t j = 0; j < cellIndices.size(); ++j)
					{
						const auto& p = _input->points[cellIndices[j]];

						if (p.label != ePowerline) // 只统计电力线
							continue;

						clouds[i].push_back(p);
					}

					// 体素抽稀
					VoxelGridSample(clouds[i],
									Eigen::Vector3f(bbox.xMin(), bbox.yMin(), bbox.zMin()),
									Eigen::Vector3f(bbox.xMax(), bbox.yMax(), bbox.zMax()),
									subsampleSize);
				}
			}

			CHECK(clouds.size() == cluster.size());

			std::vector<double> features;
			std::vector<int> cloudIndexs;
			std::vector<int> numbers(clouds.size(), 0);
			std::vector<osg::Vec3d> pts;

			for (size_t i = 0; i < clouds.size(); ++i)
			{
				const auto& points = clouds.at(i);

				if (points.empty())
					continue;

				for (size_t j = 0; j < points.size(); ++j)
				{
					const auto& p = points[j];
					pts.push_back(osg::Vec3d(p.x, p.y, p.z));
					cloudIndexs.push_back(i);
				}
			}

			// 计算主方向
			osg::Vec3d v[3];
			getEigenVectors(pts, v[0], v[1], v[2]);

			// 计算点云特征
			{
				features.resize(pts.size(), 0.0);

				// 计算各点半径范围内的邻域点
				std::vector<std::vector<int>> neighborsList;
				std::vector<std::vector<float>> distancesList;
				radiusSearch(pts, radius, neighborsList, distancesList);

				CHECK(neighborsList.size() == pts.size());

				for (size_t i = 0; i < pts.size(); ++i)
				{
					const auto& nn_indices = neighborsList[i];

					std::vector<osg::Vec3d> neighbors;
					neighbors.reserve(nn_indices.size());

					for (size_t j = 0; j < nn_indices.size(); ++j)
					{
						const auto& p = pts[nn_indices[j]];
						neighbors.push_back(p);
					}

					features[i] = ComputeFeature(method, neighbors);
				}
			}

			CHECK(features.size() == pts.size());
			CHECK(features.size() == cloudIndexs.size());

			// 统计格网单元内符合条件的点数量
			for (size_t i = 0; i < features.size(); ++i)
			{
				if (features[i] < strainThr)
					numbers[cloudIndexs[i]]++;
			}

			// 根据耐张串点云特征筛选格网，并计算点云在其主方向上的投影范围
			double minProj = DBL_MAX;
			double maxProj = -DBL_MAX;
			AttachmentCluster newCluster;

			// 统计杆塔格网单元内点数量离群值阈值
			std::vector<int> numbersNoEmpty;
			for (size_t i = 0; i < numbers.size(); ++i)
			{
				const auto& points = clouds.at(i);

				if (points.empty())
					continue;

				if (numbers[i] > 0)
					numbersNoEmpty.push_back(numbers[i]);
			}

			numPtsThr = numbersNoEmpty.empty() ? 0 : (int)Mean(numbersNoEmpty);

			for (size_t i = 0; i < numbers.size(); ++i)
			{
				const auto& points = clouds.at(i);

				if (points.empty())
					continue;

				if (numbers[i] > numPtsThr) // 满足条件的杆塔格网单元
				{
					for (size_t j = 0; j < points.size(); ++j)
					{
						const auto& p = points[j];
						osg::Vec3d pt(p.x, p.y, p.z);
						maxProj = std::max(maxProj, pt * v[0]);
						minProj = std::min(minProj, pt * v[0]);
					}
				}
			}

			// 最后，过滤出在投影范围内的格网
			for (size_t i = 0; i < cluster.size(); ++i)
			{
				const auto& index = cluster.at(i);
				const auto& cell = grid.get(index);

				if (!cell || cell->GetSize() <= 0)
					continue;

				auto matrixInverse =
					osg::Matrix::inverse(grid.rotation) * osg::Matrix::translate(grid.center);

				auto center = matrixInverse.preMult(cell->center());
				double proj = center * v[0];

				if (proj > minProj && proj < maxProj)
				{
					newCluster.push_back(cluster.at(i));
				}
			}

			cluster = newCluster;
		}

		void AttachmentClassifier::FixPowerlineNearConnectPoint(const PylonGrid3D& grid,
																const osg::Vec3d& connectPoint,
																double radius,
																std::vector<int>& powerlines)
		{
			powerlines.clear();
			osg::Vec3d o = grid.rotation.preMult(connectPoint - grid.center);

			auto visitor = [&](PylonGrid3D::Index index) {
				const auto& cell = grid.get(index);

				if (!cell || cell->GetSize() <= 0)
					return;

				if ((o - cell->center()).length() > radius * 3.0)
					return;

				const auto& cellIndices = cell->GetIndices();

				for (size_t i = 0; i < cellIndices.size(); ++i)
				{
					const auto& p = _input->points[cellIndices[i]];

					if (p.label == ePowerline)
						powerlines.push_back(cellIndices[i]);
				}
			};

			grid.Visit(visitor);
		}

		void AttachmentClassifier::FixLeadwires(const PylonGrid3D& grid,
												const PylonGrid3D::Index& connectIndex,
												const AttachmentCluster& leadwire,
												const std::vector<AttachmentCluster>& suspensions,
												const std::set<int>& strainMask)
		{ 
			int N = 2;
			int xSize = grid.getXSize();
			int ySize = grid.getYSize();
			int zSize = grid.getZSize();

			// 获得引流线点云
			auto leadwireIndices = GetIndices(grid, leadwire);

			// 设置引流线点云类型
			for (size_t i = 0; i < leadwireIndices.size(); ++i)
			{
				auto& p = _input->points[leadwireIndices[i]];

				if (p.label == ePylon)
					p.label = eLeadwire;
			};

#if 1
			int range = _options.connection_mask_distance;
			std::set<size_t> leadwireMask, suspensionMask;
		
			for (size_t i = 0; i < leadwire.size(); ++i)
			{
				const auto& index = leadwire.at(i);
				size_t idx = index[2] * xSize * ySize + index[1] * xSize + index[0];
				leadwireMask.insert(idx);
			}

			for (size_t i = 0; i < suspensions.size(); ++i)
			{
				const auto& suspension = suspensions.at(i);

				for (size_t j = 0; j < suspension.size(); ++j) 
				{
					const auto& index = suspension.at(j);
					size_t idx = index[2] * xSize * ySize + index[1] * xSize + index[0];
					suspensionMask.insert(idx);
				}
			}

			for (size_t i = 0; i < leadwire.size(); ++i) 
			{
				const auto& index = leadwire.at(i);
				int xi = index[0];
				int yi = index[1];
				int zi = index[2];

				bool hasStrain = false;
				bool hasSuspension = false;
				std::vector<int>  pylonIndices;

				// 查找邻域包含杆塔点云的格网
				for (int zii = zi - N; zii <= zi + N; ++zii) 
				{
					for (int yii = yi - N; yii <= yi + N; ++yii) 
					{
						for (int xii = xi - N; xii < xi + N; ++xii) 
						{
							if (zii < 0 || zii >= zSize || yii < 0 || yii >= ySize || xii < 0 ||
								xii >= xSize)
							{
								continue;
							}

							if (xii == xi && yii == yi && zii == zi)
								continue;

							if (std::abs(xii - connectIndex[0]) < range &&
								std::abs(yii - connectIndex[1]) < range &&
								std::abs(zii - connectIndex[2]) < range)
							{
								continue;
							}

							const auto& cell = grid.get({ xii, yii, zii });

							if (!cell || cell->GetSize() <= 0)
								continue;

							size_t idx = zii * xSize * ySize + yii * xSize + xii;
							
							if (leadwireMask.find(idx) != leadwireMask.end())
								continue;

							if (!hasSuspension)
							{
								hasSuspension =
									(suspensionMask.find(idx) != suspensionMask.end());
							}

							if (!hasStrain)
							{
								hasStrain = (strainMask.find(idx) != strainMask.end());
							}

							const auto& featureCell = grid.GetFeature({ xii, yii, zii });

							if (HasPylon(featureCell))
							{
								const auto& cellIndices = cell->GetIndices();

								pylonIndices.insert(pylonIndices.end(),
													cellIndices.begin(),
													cellIndices.end());
							}
						}
					}
				}

				if (hasStrain && !hasSuspension) 
				{
					for (size_t j = 0; j < pylonIndices.size(); ++j)
					{
						auto& p = _input->points[pylonIndices[j]];

						if (p.label == ePylon)
							p.label = eLeadwire;
					}
				}
			}
#endif

			// 修复引流线中的悬垂绝缘子串点云
			FixSuspensionInLeadwire(grid, leadwireIndices, suspensions);
		}

		void AttachmentClassifier::FixSuspensionInLeadwire(
			const PylonGrid3D& grid,
			const std::vector<int>& leadwireIndices,
			const std::vector<AttachmentCluster>& suspensions)
		{
			double radius = 0.5 * grid.cellsize;

			// 获得引流线挂串点云
			for (size_t t = 0; t < suspensions.size(); ++t)
			{
				auto suspensionIndices = GetIndices(grid, suspensions.at(t));

				double zmin = DBL_MAX;
				double zmax = -DBL_MAX;
				std::vector<double> xvals, yvals;

				for (size_t j = 0; j < suspensionIndices.size(); ++j)
				{
					auto& p = _input->points[suspensionIndices[j]];

					zmin = std::min(zmin, (double)p.z);
					zmax = std::max(zmax, (double)p.z);
					xvals.push_back(p.x);
					yvals.push_back(p.y);

					if (p.label == ePylon)
						p.label = eIsulator;
				};

				// 中心轴半径范围内的挂串点修复
				double xmid = Median(xvals);
				double ymid = Median(yvals);

				for (size_t j = 0; j < leadwireIndices.size(); ++j)
				{
					auto& p = _input->points[leadwireIndices[j]];

					if (p.z < zmin || p.z > zmax)
						continue;

					double distance =
						std::sqrt((p.x - xmid) * (p.x - xmid) + (p.y - ymid) * (p.y - ymid));

					if (distance < radius)
						p.label = eIsulator;
				};
			}
		}

		void AttachmentClassifier::AttachmentClustering(const PylonGrid3D& grid,
														const AttachmentCluster& seeds,
														std::vector<AttachmentCluster>& clusters)
		{
			// 生成聚类源码
			std::set<size_t> mask;
			std::set<size_t> processed;

			int xSize = grid.getXSize();
			int ySize = grid.getYSize();
			int zSize = grid.getZSize();

			for (size_t i = 0; i < seeds.size(); ++i)
			{
				const auto& index = seeds.at(i);
				size_t idx = index[2] * xSize * ySize + index[1] * xSize + index[0];
				mask.insert(idx);
			}

			clusters.clear();
			const auto& dimOffset = _GROW_ALL;

			for (size_t i = 0; i < seeds.size(); ++i)
			{
				const auto& index = seeds.at(i);
				const auto& cell = grid.get(index);
			
				if (!cell || cell->GetSize() <= 0)
					continue;

				size_t si = index[2] * xSize * ySize + index[1] * xSize + index[0];
				if (processed.find(si) != processed.end())
					continue;

				std::vector<Grid3D::Index> seed_queue;

				int sq_idx = 0;
				seed_queue.push_back(index);
				processed.insert(si);

				while (sq_idx < (int)seed_queue.size())
				{
					Grid3D::Index index = seed_queue[sq_idx];
					const auto& seedCell = grid.get(index[0], index[1], index[2]);

					if (!seedCell || seedCell->GetSize() <= 0)
					{
						sq_idx++;
						continue;
					}

					// 遍历相邻接节点
					for (size_t oi = 0; oi < dimOffset.size(); ++oi)
					{
						const auto& offset = dimOffset[oi];
						int xii = clamp(index[0] + offset[0], 0, xSize - 1);
						int yii = clamp(index[1] + offset[1], 0, ySize - 1);
						int zii = clamp(index[2] + offset[2], 0, zSize - 1);

						size_t sj = zii * xSize * ySize + yii * xSize + xii;

						if (mask.find(sj) == mask.end())
							continue;

						if (processed.find(sj) != processed.end())
							continue;

						auto cellNbr = grid.get(xii, yii, zii);

						if (cellNbr && cellNbr->GetSize() > 0)
						{
							seed_queue.push_back({ xii, yii, zii });
							processed.insert(sj);
						}
					}

					sq_idx++;

				} // end of while

				if (!seed_queue.empty())
					clusters.push_back(seed_queue);
			}
		}

		void ComputePylonGrid(double cellsize,
							  const osg::Vec3d& center,
							  const osg::BoundingBox& obb,
							  const osg::Matrix& rotation,
							  PointCloudViewPtr input,
							  const std::vector<int>& indices,
							  PylonGrid3D& grid)
		{
			CHECK_MSG(input, "无效点云数据.");
			CHECK_MSG(!indices.empty(), "空的点云数据.");

			double xmin = obb.xMin();
			double xmax = obb.xMax();
			double ymin = obb.yMin();
			double ymax = obb.yMax();
			double zmin = obb.zMin();
			double zmax = obb.zMax();

			int xSize = std::floor((xmax - xmin) / cellsize) + 1;
			int ySize = std::floor((ymax - ymin) / cellsize) + 1;
			int zSize = std::floor((zmax - zmin) / cellsize) + 1;

			CHECK_MSG(xSize > 0, StringPrintf("xmin=%lf, xmax=%lf.", xmin, xmax));
			CHECK_MSG(ySize > 0, StringPrintf("ymin=%lf, ymax=%lf.", ymin, ymax));
			CHECK_MSG(zSize > 0, StringPrintf("zmin=%lf, zmax=%lf.", zmin, zmax));

			grid.resize(xSize, ySize, zSize);
			grid.cellsize = cellsize;
			grid.center = center;
			grid.rotation = rotation;
			grid.obb = obb;

			size_t maxsize = 0;

			for (int i = 0; i < (int)indices.size(); ++i)
			{
				const auto& p = input->points[indices[i]];
				osg::Vec3d local = rotation.preMult(osg::Vec3d(p.x, p.y, p.z) - center);

				int xi = std::floor((local.x() - xmin) / cellsize);
				int yi = std::floor((local.y() - ymin) / cellsize);
				int zi = std::floor((local.z() - zmin) / cellsize);

				xi = clamp(xi, 0, xSize - 1);
				yi = clamp(yi, 0, ySize - 1);
				zi = clamp(zi, 0, zSize - 1);

				std::shared_ptr<GridCell>& cell = grid.getOrCreate(xi, yi, zi);

				if (!cell)
				{
					cell = std::make_shared<GridCell>();
					cell->SetInput(input);
					cell->_min = osg::Vec3d(xmin + xi * cellsize,
											ymin + yi * cellsize,
											zmin + zi * cellsize);
					cell->_max = osg::Vec3d(xmin + (xi + 1) * cellsize,
											ymin + (yi + 1) * cellsize,
											zmin + (zi + 1) * cellsize);
				}

				cell->AddPoint(indices[i]);

				PylonGrid3D::CellFeaturePtr& featureCell = grid.GetOrCreateFeature(xi, yi, zi);

				if (!featureCell)
				{
					// 初始化
					featureCell = std::make_shared<PylonGrid3D::CellFeature>();
					featureCell->cluster_id = -1;
					featureCell->has_pylon = false;
					featureCell->has_powerline = false;
					featureCell->x_length = 0.0;
					featureCell->y_length = 0.0;
					featureCell->x_fillrate = 0.0;
					featureCell->y_fillrate = 0.0;
					featureCell->xy_fillrate = 0.0;
					featureCell->xz_fillrate = 0.0;
				}

				if (!featureCell->has_pylon && p.label == ePylon)
					featureCell->has_pylon = true;

				if (!featureCell->has_powerline && p.label == ePowerline)
					featureCell->has_powerline = true;
			}
		}

	}
}