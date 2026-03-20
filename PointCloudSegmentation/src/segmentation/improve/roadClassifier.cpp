#include <src/segmentation/improve/roadClassifier.h>
#include <src/algorithm/math.h>
#include <src/algorithm/geometry2d_op.h>
#include <src/core/private/rasterProcess.h>
#include <src/core/private/cloudProcess.h>
#include <src/core/private/gdalProcess.h>
#include <src/plot/geomCreator.h>
#include <src/plot/plotHandle.h>
#include <src/service/ValueBuffer.h>

#include <mutex>

// #define RENDER_VIRTUAL_GRID
// #define RENDER_ROAD_VECTOR

namespace d3s {
	namespace pcs {

		VirtualGrid::VirtualGrid() : _nRows(0), _nColumns(0) {}

		VirtualGrid::VirtualGrid(int width, int height) { resize(width, height); }

		VirtualGrid::VirtualGrid(const VirtualGrid& rhs)
			: _nRows(rhs._nRows), _nColumns(rhs._nColumns), _cells(rhs._cells)
		{
		}

		VirtualGrid::~VirtualGrid()
		{
			for (size_t i = 0; i < _cells.size(); ++i)
			{
				VirtualGrid::CellPtr& cell = _cells[i];

				if (!cell)
					continue;

				delete cell;
			}
		}

		void VirtualGrid::resize(int width, int height)
		{
			_nColumns = width;
			_nRows = height;
			_cells.resize(_nRows * _nColumns);
		}

		VirtualGrid::CellPtr& VirtualGrid::at(int row, int col)
		{
			return _cells[row * _nColumns + col];
		}

		const VirtualGrid::CellPtr& VirtualGrid::at(int row, int col) const
		{
			return _cells.at(row * _nColumns + col);
		}

		VirtualGrid::CellPtr& VirtualGrid::at(int npos) { return _cells[npos]; }

		const VirtualGrid::CellPtr& VirtualGrid::at(int npos) const { return _cells.at(npos); }

		VirtualGrid::CellPtr& VirtualGrid::operator[](int npos) { return _cells[npos]; }

		const VirtualGrid::CellPtr& VirtualGrid::operator[](int npos) const
		{
			return _cells.at(npos);
		}


		void ComputeVirtualGridCells(double cellsize,
									 PointCloudViewPtr input,
									 const std::vector<int>& indices,
									 VirtualGrid& grid)
		{
			CHECK_MSG(input, "无效点云数据.");

			double xmin = input->bbox.xMin();
			double xmax = input->bbox.xMax();
			double ymin = input->bbox.yMin();
			double ymax = input->bbox.yMax();
			double zmin = input->bbox.zMin();
			double zmax = input->bbox.zMax();

			int numRow = std::floor((ymax - ymin) / cellsize) + 1;
			int numColumn = std::floor((xmax - xmin) / cellsize) + 1;

			// 网格内存分配
			grid.resize(numColumn, numRow);

			// 初始化格网
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

				int mutexIndex = cellsize < 1.0 ? 0 : ri * numColumn + ci;
				std::unique_lock<std::mutex> lock(gridMutex[mutexIndex]);
				auto& cell = grid.at(ri, ci);

				if (!cell)
				{
					cell = new GridCell();
					cell->SetInput(input);

					cell->xMin() = xmin + ci * cellsize;
					cell->yMin() = ymin + ri * cellsize;
					cell->xMax() = xmin + (ci + 1) * cellsize;
					cell->yMax() = ymin + (ri + 1) * cellsize;
				}

				cell->AddPoint(indices[i]);
			}

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)grid.size(); ++i)
			{
				auto& cell = grid[i];

				if (!cell || cell->GetSize() <= 0)
					continue;

				const auto& indices = cell->GetIndices();

				double hmax = -DBL_MAX;
				double zmax = -DBL_MAX;
				double zmin = DBL_MAX;

				for (size_t i = 0; i < indices.size(); ++i)
				{
					const auto& p = input->points[indices[i]];
					zmax = std::max(zmax, (double)p.z);
					zmin = std::min(zmin, (double)p.z);
				}

				cell->zMin() = zmin;
				cell->zMax() = zmax;
			}

			PCS_INFO("[ComputeGridCells] 格网划分完成(%lf).", cellsize);
		}

		// RoadClassifier 道路点云分类器
		//////////////////////////////////////////////////////////////////////////
		RoadClassifier::RoadClassifier(const RoadClassifyOptions& options,
									   PointCloudViewPtr input,
									   const std::vector<int>& indices)
			: _options(options), _input(input), _indices(indices)
		{
		}

		RoadClassifier::~RoadClassifier() {}

		void RoadClassifier::SetRoadVectorize(d3s::share_ptr<IValueBuffer> roadVectorize)
		{
			_roadVectorize = roadVectorize;
		}

		void RoadClassifier::Segment()
		{
			if (_roadVectorize.valid() && _input->epsg > 0)
			{
				PCS_INFO("[RoadClassifier] 检测到道路矢量数据.");
				GetRoadCandiateIndices(_indices);

				_options.cell_size = 0.5;
				_options.height_threshold = 0.5;
				_options.slope_threshold = 0.05;
				_options.linearity_threshold = 0.85;
				_options.min_area = 200.0;
				_options.max_area = 1000000.0;
			}

			SegmentNormal();
		}

		void RoadClassifier::GetRoadCandiateIndices(std::vector<int>& indices)
		{
			CHECK(_input->epsg > 0);

			d3s::share_ptr<CRoadVectorizeBuffer> roadVectorize =
				dynamic_cast<CRoadVectorizeBuffer*>(_roadVectorize.get());

			if (!roadVectorize.valid())
				return;

			int epsg = roadVectorize->GetEPSG();

			double xmin, ymin, xmax, ymax, zmin;
			xmin = _input->bbox.xMin();
			ymin = _input->bbox.yMin();
			xmax = _input->bbox.xMax();
			ymax = _input->bbox.yMax();
			zmin = _input->bbox.zMin();

			BoundingBox2D srcBound(xmin, ymin, xmax, ymax);

			// 转换到原始坐标
			xmin += _input->offset_xyz.x();
			ymin += _input->offset_xyz.y();
			xmax += _input->offset_xyz.x();
			ymax += _input->offset_xyz.y();

			coordinateSystemTransform(_input->epsg, epsg, &xmin, &ymin);
			coordinateSystemTransform(_input->epsg, epsg, &xmax, &ymax);

			BoundingBox2D dstBound(xmin, ymin, xmax, ymax);

			std::mutex mtx;
			std::vector<std::vector<osg::Vec3d>> roads;
			int numRoads = roadVectorize->GetRoadCount();

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < numRoads; ++i)
			{
				const VectorLineString* lineString = roadVectorize->GetRoad(i);

				if (!lineString)
					continue;

				if (dstBound.intersects(lineString->bound))
				{
					std::vector<osg::Vec3d> roadLine;

					for (size_t j = 0; j < lineString->verts.size(); ++j)
					{
						const auto& vert = lineString->verts.at(j);

						double px = vert.x();
						double py = vert.y();
						coordinateSystemTransform(epsg, _input->epsg, &px, &py);

						px -= _input->offset_xyz.x();
						py -= _input->offset_xyz.y();

						roadLine.push_back(osg::Vec3d(px, py, zmin));
					}

					std::unique_lock<std::mutex> lock(mtx);

					if (!roadLine.empty())
						roads.push_back(roadLine);
				}
			}

			// 根据点云边界裁剪道路矢量
			ClipingWithBound(srcBound, roads);

#ifdef RENDER_ROAD_VECTOR
			for (size_t i = 0; i < roads.size(); ++i)
			{
				std::string roadname = StringPrintf("road_%d", i);
				RenderLinePath(roadname, roads.at(i), _input, osg::Vec4(1.f, 0.5f, 1.f, 1.f));
			}
#endif

			Rasterd roadRaster;
			createRoadRaster(*_input, roads, roadRaster);

			double half = roadRaster.edgeLength() / 2.0;
			double eps = roadRaster.edgeLength() * .000001;

			std::vector<int> roadCandidate;
			roadCandidate.reserve(_indices.size());

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)_indices.size(); ++i)
			{
				auto& p = _input->points[_indices[i]];

				if (p.label != eGround)
					continue;

				int xi = roadRaster.xCell(p.x + half - eps);
				int yi = roadRaster.yCell(p.y + half - eps);
				xi = clamp(xi, 0, roadRaster.width() - 1);
				yi = clamp(yi, 0, roadRaster.height() - 1);

				if (roadRaster.at(xi, yi) == 1.0)
				{
					// p.label = eRoad;
					std::unique_lock<std::mutex> lock(mtx);
					roadCandidate.push_back(_indices[i]);
				}
			}

			roadCandidate.shrink_to_fit();
			std::swap(indices, roadCandidate);

			PCS_INFO("[RoadClassifier] 相交道路矢量: %d.", roads.size());
		}

		void RoadClassifier::SegmentNormal()
		{
			double cellsize = _options.cell_size;

			// 创建虚拟格网
			GridPtr grid = std::make_shared<VirtualGrid>();

			// 创建网格
			ComputeVirtualGridCells(cellsize, _input, _indices, *grid);

			// 移除高差过大的格网
			RemoveLowHeightCells(grid);

			// 按照坡度约束进行格网聚类
			Clusters clusters;
			Clustering(grid, clusters);

			// 通过面积阈值, 将太小的类簇过滤掉
			Filtering(grid, clusters);

			PCS_INFO("[RoadClassifier] 格网聚类数: %d.", clusters.size());

#ifdef RENDER_VIRTUAL_GRID
			RenderVirtualGridClusters(grid, clusters);
#endif

			// 更新点类别
#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)clusters.size(); ++i)
			{
				const auto& cluster = clusters.at(i);

				for (size_t j = 0; j < cluster.size(); ++j)
				{
					const auto& index = cluster.at(j);
					const auto& cell = grid->at(index[0], index[1]);

					if (!cell || cell->GetSize() <= 0)
						continue;

					const auto& cellIndices = cell->GetIndices();

					for (size_t k = 0; k < cellIndices.size(); ++k)
					{
						auto& p = _input->points[cellIndices[k]];

						if (p.label == eGround)
							p.label = eRoad;
					}
				}
			}
		}

		void RoadClassifier::RemoveLowHeightCells(GridPtr grid)
		{
			if (!grid)
				return;

			double Th = _options.height_threshold;

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)grid->size(); ++i)
			{
				auto& cell = grid->at(i);

				if (!cell || cell->GetSize() <= 0)
					continue;

				double dZ = cell->zMax() - cell->zMin();

				if (dZ > Th)
					cell->Clear();
			}
		}

		void RoadClassifier::Clustering(GridPtr grid, Clusters& clusters)
		{
			if (!grid)
				return;

			double Ts = _options.slope_threshold;

			int nRows = grid->getNumRows();
			int nCols = grid->getNumColumns();

			std::set<size_t> processed;

			for (int r = 0; r < nRows; ++r)
			{
				for (int c = 0; c < nCols; ++c)
				{
					const auto& currentCell = grid->at(r, c);

					if (!currentCell || currentCell->GetSize() <= 0)
						continue;

					size_t i = r * nCols + c;

					// 已处理过的，不再处理
					if (processed.find(i) != processed.end())
						continue;

					std::vector<VirtualGrid::Index> seed_queue;

					int sq_idx = 0;
					seed_queue.push_back({ r, c });
					processed.insert(i);

					while (sq_idx < (int)seed_queue.size())
					{
						VirtualGrid::Index index = seed_queue[sq_idx];
						const int rr = index[0];
						const int cc = index[1];

						const auto& cell = grid->at(rr, cc);

						// 跳过空的格网
						if (!cell || cell->GetSize() <= 0)
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
								size_t j = ri * nCols + ci;

								if (processed.find(j) != processed.end())
									continue;

								const auto& cellNbr = grid->at(ri, ci);

								if (!cellNbr || cellNbr->GetSize() <= 0)
									continue;

								double hDelta = fabs(cell->zMax() - cellNbr->zMax());

								if (hDelta < Ts)
								{
									seed_queue.push_back({ ri, ci });
									processed.insert(j);
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

		void RoadClassifier::Filtering(GridPtr grid, Clusters& clusters)
		{
			if (clusters.empty())
				return;

			double Ta0 = _options.min_area;
			double Ta1 = _options.max_area;
			double Ts = _options.shape_threshold;
			double Tl = _options.linearity_threshold;

			// 类簇是否有效
			std::vector<bool> clusterValids(clusters.size(), true);

			// 创建栅格化
#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)clusters.size(); ++i)
			{
				const auto& cluster = clusters.at(i);

				double area = ComputeArea(grid, cluster);

				if (area < Ta0 || area >= Ta1)
				{
					clusterValids[i] = false;
					continue;
				}

				/*double shapeness = ComputeShapeness(grid, cluster);

				if (shapeness > Ts)
				{
					clusterValids[i] = false;
					continue;
				}*/

				double linearity = ComputeLinearity(grid, cluster);

				if (linearity < Tl)
				{
					clusterValids[i] = false;
					continue;
				}

				PCS_INFO("[RoadClassifier] 类簇 %d，面积 %lf ㎡，线性系数 %lf.",
						 i,
						 area,
						 linearity);
			}

			Clusters valids;

			for (size_t i = 0; i < clusterValids.size(); ++i)
			{
				if (clusterValids.at(i))
					valids.push_back(clusters.at(i));
			}

			std::swap(valids, clusters);
		}

		void RoadClassifier::ClipingWithBound(const BoundingBox2D& bound,
											  std::vector<std::vector<osg::Vec3d>>& roads)
		{
			// 矩形边界角点
			auto RectangleClip = [](const BoundingBox2D& rectangle, osg::Vec2& p0, osg::Vec2& p1) {
				if (rectangle.contains(p0) && rectangle.contains(p1))
				{
					// 线段两点均在矩形边界内
					return true;
				}
				else
				{
					osg::Vec2* pInside = nullptr;
					osg::Vec2* pOutside = nullptr;

					if (rectangle.contains(p0))
					{
						pInside = &p0;
						pOutside = &p1;
					}
					else if (rectangle.contains(p1))
					{
						pInside = &p1;
						pOutside = &p0;
					}

					bool bIntersect = false;

					if (pInside && pOutside)
					{
						// 有一个点在矩形内，则求线段与矩形各边的角点
						for (int i = 0; i < 4; ++i)
						{
							double x, y;

							// 矩形的四条边
							osg::Vec2 eA = rectangle.corner(i % 4);
							osg::Vec2 eB = rectangle.corner((i + 1) % 4);

							if (lineSegmentIntersection(eA, eB, *pInside, *pOutside, x, y))
							{
								pOutside->x() = x;
								pOutside->y() = y;
								bIntersect = true;
								break;
							}
						}
					}
					else
					{
						std::vector<osg::Vec2> pointOnEdge;

						// 线段两点均在矩形之外，还要判断是否穿过矩形
						for (int i = 0; i < 4; ++i)
						{
							double x, y;

							// 矩形的四条边
							osg::Vec2 eA = rectangle.corner(i % 4);
							osg::Vec2 eB = rectangle.corner((i + 1) % 4);

							if (lineSegmentIntersection(eA, eB, p0, p1, x, y))
							{

								osg::Vec2 p(x, y);
								auto iter = std::find(pointOnEdge.begin(), pointOnEdge.end(), p);

								if (iter == pointOnEdge.end())
									pointOnEdge.push_back(p);
							}
						}

						if ((pointOnEdge.size() >= 2))
						{
							osg::Vec2 dir = p1 - p0;
							dir.normalize();

							p0 = pointOnEdge[0];
							p1 = pointOnEdge[1];

							osg::Vec2 dir2 = p1 - p0;
							dir2.normalize();

							if (dir * dir2 < 0.0)
								std::swap(p1, p0);


							bIntersect = true;
						}
					}

					return bIntersect;
				}
			};

			double zmin = _input->bbox.zMin();

			std::mutex mtx;
			std::vector<std::vector<osg::Vec3d>> clipRoads;

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)roads.size(); ++i)
			{
				const auto& road = roads.at(i);

				std::vector<osg::Vec3d> clipRoad;

				for (size_t j = 0; j < road.size() - 1; ++j)
				{
					osg::Vec2 p0 = osg::Vec2(road[j].x(), road[j].y());
					osg::Vec2 p1 = osg::Vec2(road[j + 1].x(), road[j + 1].y());

					if (RectangleClip(bound, p0, p1))
					{
						clipRoad.push_back(osg::Vec3(p0, zmin));
						clipRoad.push_back(osg::Vec3(p1, zmin));
					}
				}

				if (!clipRoad.empty())
				{
					std::unique_lock<std::mutex> lock(mtx);
					clipRoads.push_back(clipRoad);
				}
			}

			std::swap(roads, clipRoads);
		}

		double RoadClassifier::ComputeArea(GridPtr grid, const VirtualGrid::Cluster& cluster)
		{
			double area = 0.0;

			for (size_t i = 0; i < cluster.size(); ++i)
			{
				auto index = cluster.at(i);
				const auto& cell = grid->at(index[0], index[1]);

				if (!cell || cell->GetSize() <= 0)
					continue;

				// 计算面积
				area += sqrt((cell->xMax() - cell->xMin()) * (cell->yMax() - cell->yMin()));
			}

			return area;
		}

		double RoadClassifier::ComputeShapeness(GridPtr grid, const VirtualGrid::Cluster& cluster)
		{
			double shapeness = DBL_MAX;

			if (cluster.empty())
				return shapeness;

			int rmin = INT_MAX;
			int cmin = INT_MAX;
			int rmax = -INT_MAX;
			int cmax = -INT_MAX;

			for (size_t i = 0; i < cluster.size(); ++i)
			{
				const auto& index = cluster.at(i);

				rmin = std::min(rmin, index[0]);
				rmax = std::max(rmax, index[0]);
				cmin = std::min(cmin, index[1]);
				cmax = std::max(cmax, index[1]);
			}

			CHECK(rmin != INT_MAX && cmin != INT_MAX && rmax != -INT_MAX && cmax != -INT_MAX);
			int nRows = rmax - rmin + 1;
			int nCols = cmax - cmin + 1;

			RasterExtents extents(0, 0, nCols, nRows, 1);
			Rasterd clusterMask(extents, "virtual grid cluster", 0.0);

			for (size_t i = 0; i < cluster.size(); ++i)
			{
				const auto& index = cluster.at(i);
				int r = index[0];
				int c = index[1];

				const auto& cell = grid->at(r, c);

				if (!cell || cell->GetSize() <= 0)
					continue;

				clusterMask.at(c - cmin, r - rmin) = 1.0;
			}

			// OpenCV 计算形状系数
			shapeness = compuateShapeness(clusterMask);


			return shapeness;
		}

		double RoadClassifier::ComputeLinearity(GridPtr grid, const VirtualGrid::Cluster& cluster)
		{
			if (cluster.empty())
				return 0.0;

			std::vector<osg::Vec3d> pts;

			for (size_t i = 0; i < cluster.size(); ++i)
			{
				const auto& index = cluster.at(i);
				const auto& cell = grid->at(index[0], index[1]);

				if (!cell || cell->GetSize() <= 0)
					continue;

				osg::Vec3d pt = cell->center();
				pt.z() = 0.0;
				pts.push_back(pt);
			}

			if (pts.size() < 2)
				return 0.0;

			double lamda1 = 0.0;
			double lamda2 = 0.0;
			double lamda3 = 0.0;
			getEigenValues(pts, lamda1, lamda2, lamda3); // λ1 > λ2 > λ3

			if (lamda1 == 0.0)
				return 0.0;

			return 1.0 - (lamda2 / lamda1);
		}

		void RoadClassifier::RenderVirtualGridClusters(GridPtr grid, const Clusters& clusters)
		{
			std::vector<osg::Vec4> colorMap = createColors(clusters.size());

			// DEBUG: 显示格网聚类簇
			RenderGridCells(*grid, clusters, colorMap, _input);
		}
	}
}