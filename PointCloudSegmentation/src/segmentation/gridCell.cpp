//stdafx.h
#include "gridCell.h"

#include <mutex>
#include <set>

#include "../core/pointTypes.hpp"
#include "../core/private/statistics.h"
#include "../algorithm/math.h"
#include "../utils/logging.h"
#include "../utils/stringutil.h"
#include <ClassificationDef.h>

namespace d3s {
	namespace pcs {
		osg::Vec2i NEIGHBORS_8[8] = { { -1, -1 }, { 0, -1 }, { 1, -1 }, { -1, 0 },
										   { 1, 0 },   { -1, 1 }, { 0, 1 },	 { 1, 1 } };

		// GridCell 网格单元
		//////////////////////////////////////////////////////////////////////////
		GridCell::GridCell() :label(eUnclassified), _input(nullptr) {}

		GridCell::GridCell(const osg::Vec3& pmin, const osg::Vec3& pmax)
			: osg::BoundingBox(pmin, pmax),
			  label(eUnclassified),
			  _input(nullptr)

		{
		}

		GridCell::GridCell(double xmin,
						   double ymin,
						   double zmin,
						   double xmax,
						   double ymax,
						   double zmax)
			: osg::BoundingBox(xmin, ymin, zmin, xmax, ymax, zmax),
			  label(eUnclassified),
			  _input(nullptr)
		{
		}

		GridCell::GridCell(const GridCell& rhs)
			: osg::BoundingBox(rhs), label(rhs.label),
			  _input(rhs._input),
			  _indices(rhs._indices)
		{
		}

		GridCell& GridCell::operator=(const GridCell& rhs) 
		{
			_input = rhs._input;
			_indices = rhs._indices;
			label = rhs.label;

			_min = rhs._min;
			_max = rhs._max;

			return *this;
		}

		void GridCell::AddPoint(int pointIndex) { _indices.push_back(pointIndex); }

		void GridCell::Clear() { _indices.clear(); }
	
		// Grid2D 2D网格划分
		//////////////////////////////////////////////////////////////////////////
		Grid2D::Grid2D() : _nRows(0), _nColumns(0) {}

		Grid2D::Grid2D(int width, int height) { resize(width, height); }

		Grid2D::Grid2D(const Grid2D& rhs)
			: _nRows(rhs._nRows), _nColumns(rhs._nColumns), _cells(rhs._cells)
		{
		}

		void Grid2D::resize(int width, int height)
		{
			_nColumns = width;
			_nRows = height;
			_cells.resize(_nRows * _nColumns);
		}

		GridCell& Grid2D::at(int row, int col) { return _cells[row * _nColumns + col]; }

		const GridCell& Grid2D::at(int row, int col) const { return _cells.at(row * _nColumns + col); }

		GridCell& Grid2D::at(int npos) { return _cells[npos]; }

		const GridCell& Grid2D::at(int npos) const { return _cells.at(npos); }

		GridCell& Grid2D::operator[](int npos) { return _cells[npos]; }

		const GridCell& Grid2D::operator[](int npos) const { return _cells.at(npos); }


		// Grid3D 3D网格划分
		//////////////////////////////////////////////////////////////////////////
		Grid3D::Grid3D() : _xSize(0), _ySize(0), _zSize(0) {}

		Grid3D::Grid3D(int xSize, int ySize, int zSize) { resize(xSize, ySize, zSize); }

		Grid3D::Grid3D(const Grid3D& rhs)
			: _xSize(rhs._xSize),
			  _ySize(rhs._ySize),
			  _zSize(rhs._zSize),
			  _sparse_cells(rhs._sparse_cells)
		{
		}

		void Grid3D::resize(int xSize, int ySize, int zSize)
		{
			_xSize = xSize;
			_ySize = ySize;
			_zSize = zSize;
		}

		std::shared_ptr<GridCell>& Grid3D::getOrCreate(int xi, int yi, int zi)
		{
			return _sparse_cells[{xi, yi, zi}];
		}
		
		std::shared_ptr<GridCell> Grid3D::get(int xi, int yi, int zi) const
		{
			Index index = { xi, yi, zi };

			auto it = _sparse_cells.find(index);
			return it == _sparse_cells.end() ? nullptr : it->second;
		}

		std::shared_ptr<GridCell>& Grid3D::getOrCreate(const Index& index)
		{
			return _sparse_cells[index];
		}

		std::shared_ptr<GridCell> Grid3D::get(const Index& index) const
		{
			auto it = _sparse_cells.find(index);
			return it == _sparse_cells.end() ? nullptr : it->second;
		}

		void Grid3D::Visit(std::function<void(const std::shared_ptr<GridCell>&)> visitor) const
		{
			if (visitor)
			{
				for (auto it = _sparse_cells.begin(); it != _sparse_cells.end(); ++it) 
					visitor(it->second);
			}
		}

		void Grid3D::Visit(std::function<void(Index, const std::shared_ptr<GridCell>&)> visitor) const
		{
			if (visitor)
			{
				for (auto it = _sparse_cells.begin(); it != _sparse_cells.end(); ++it)
					visitor(it->first, it->second);
			}
		}


		//////////////////////////////////////////////////////////////////////////

		void ComputeGridCells(double cellsize,
							  PointCloudViewPtr input,
							  const std::vector<int>& indices,
							  Grid2D& grid)
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
					cell._min = osg::Vec3d(xmin + c * cellsize, ymin + r * cellsize, zmin);
					cell._max =
						osg::Vec3d(xmin + (c + 1) * cellsize, ymin + (r + 1) * cellsize, zmax);
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
				ri = clamp(ri, 0, numRow - 1);
				ci = clamp(ci, 0, numColumn - 1);

				// 对应单元加锁
				int mutexIndex = cellsize < 1.0 ? 0 : ri * numColumn + ci;
				std::unique_lock<std::mutex> lock(gridMutex[mutexIndex]);
				GridCell& cell = grid.at(ri, ci);
				cell.AddPoint(indices[i]);
			}

			PCS_INFO("[ComputeGridCells] 格网划分完成(%lf).", cellsize);
		}


		void ComputeLocalGridCells(double cellsize,
								   PointCloudViewPtr input,
								   const std::vector<int>& indices,
								   Grid2D& grid)
		{
			CHECK_MSG(input, "无效点云数据.");

			osg::BoundingBox bbox;
			computeMinMax3D(*input, indices, bbox);

			double xmin = bbox.xMin();
			double xmax = bbox.xMax();
			double ymin = bbox.yMin();
			double ymax = bbox.yMax();
			double zmin = bbox.zMin();
			double zmax = bbox.zMax();

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

					cell._min = osg::Vec3d(xmin + c * cellsize, ymin + r * cellsize, zmin);
					cell._max =
						osg::Vec3d(xmin + (c + 1) * cellsize, ymin + (r + 1) * cellsize, zmax);
				}
			}

			int numMutex = cellsize < 1.0 ? 1 : numRow * numColumn;
			std::vector<std::mutex> gridMutex(numMutex);

#ifdef _OPENMP
#pragma omp parallel for num_threads(10)
#endif
			for (int i = 0; i < (int)indices.size(); ++i)
			{
				const auto& p = input->points[indices[i]];

				int ri = std::floor((p.y - ymin) / cellsize);
				int ci = std::floor((p.x - xmin) / cellsize);
				ri = clamp(ri, 0, numRow - 1);
				ci = clamp(ci, 0, numColumn - 1);

				// 对应单元加锁
				int mutexIndex = cellsize < 1.0 ? 0 : ri * numColumn + ci;
				std::unique_lock<std::mutex> lock(gridMutex[mutexIndex]);
				GridCell& cell = grid.at(ri, ci);
				cell.AddPoint(indices[i]);
			}

			PCS_INFO("[ComputeLocalGridCells] 格网划分完成(%lf).", cellsize);
		}


		void ComputeLocalGridCells(double cellsize,
								   PointCloudViewPtr input,
								   const std::vector<int>& indices,
								   ClassificationType label,
								   Grid3D& grid)
		{
			CHECK_MSG(input, "无效点云数据.");

			osg::BoundingBox bbox;
			computeMinMax3D(*input, indices, bbox);

			double xmin = bbox.xMin();
			double xmax = bbox.xMax();
			double ymin = bbox.yMin();
			double ymax = bbox.yMax();
			double zmin = bbox.zMin();
			double zmax = bbox.zMax();

			int xSize = std::floor((xmax - xmin) / cellsize) + 1;
			int ySize = std::floor((ymax - ymin) / cellsize) + 1;
			int zSize = std::floor((zmax - zmin) / cellsize) + 1;

			CHECK_MSG(xSize > 0, StringPrintf("xmin=%lf, xmax=%lf.", xmin, xmax));
			CHECK_MSG(ySize > 0, StringPrintf("ymin=%lf, ymax=%lf.", ymin, ymax));
			CHECK_MSG(zSize > 0, StringPrintf("zmin=%lf, zmax=%lf.", zmin, zmax));

			grid.resize(xSize, ySize, zSize);

			
			for (int i = 0; i < (int)indices.size(); ++i)
			{
				const auto& p = input->points[indices[i]];

				int xi = std::floor((p.x - xmin) / cellsize);
				int yi = std::floor((p.y - ymin) / cellsize);
				int zi = std::floor((p.z - zmin) / cellsize);

				xi = clamp(xi, 0, xSize - 1);
				yi = clamp(yi, 0, ySize - 1);
				zi = clamp(zi, 0, zSize - 1);

				std::shared_ptr<GridCell>& cell = grid.getOrCreate(xi, yi, zi);

				if (!cell)
				{
					cell = std::make_shared<GridCell>();
					cell->SetInput(input);
					cell->label = label;

					cell->_min = osg::Vec3d(xmin + xi * cellsize,
											ymin + yi * cellsize,
											zmin + zi * cellsize);

					cell->_max = osg::Vec3d(xmin + (xi + 1) * cellsize,
											ymin + (yi + 1) * cellsize,
											zmin + (zi + 1) * cellsize);
				}

				cell->AddPoint(indices[i]);
			}
		}

		void GetCellNeighbors(const Grid2D& gird, int r, int c, std::vector<int>& indicesNeighbor)
		{
			int numRows = gird.getNumRows();
			int numColumns = gird.getNumColumns();
			indicesNeighbor.clear();

			for (int j = 0; j < sizeof(NEIGHBORS_8) / sizeof(osg::Vec2i); ++j)
			{
				int xi = std::max(0, std::min(c + NEIGHBORS_8[j].x(), numColumns - 1));
				int yi = std::max(0, std::min(r + NEIGHBORS_8[j].y(), numRows - 1));

				const GridCell& cell = gird.at(yi, xi);

				if (cell.GetSize() <= 0)
					continue;

				indicesNeighbor.insert(indicesNeighbor.end(),
									   cell.GetIndices().begin(),
									   cell.GetIndices().end());
			}
		}

		/*static osg::Vec3i NEIGHBORS_26[26] = { {}
		};*/

		/**
		 *  @brief    根据体素网格，获取连同区域
		 *
		 *  @param    const Grid3D & grid						体素网格
		 *  @param    std::vector<std::vector<Grid3D::Index>> & clusters	聚类
		 *
		 *  @return   void
		 */
		void GetVoxelGridConnectComponents(const Grid3D& grid,
										   std::vector<std::vector<Grid3D::Index>>& clusters)
		{
			clusters.clear();

			int xSize = grid.getXSize();
			int ySize = grid.getYSize();
			int zSize = grid.getZSize();
			size_t totalSize = xSize * ySize * zSize;

			// PCS_DEBUG("grid size memory: %lf MB", (double)totalSize / 1024.0 / 1024.0);

			CHECK(grid.size() > 0);
			std::set<size_t> processed;

			std::array<int, 3> d = { -1, 0, 1 };
			std::vector<std::array<int, 3>> dimOffset;

			for (int z = 0; z < 3; ++z)
			{
				for (int y = 0; y < 3; ++y)
				{
					for (int x = 0; x < 3; ++x)
					{
						if (x == y && y == z && z == 1)
							continue;

						dimOffset.push_back({ d[x], d[y], d[z] });
					}
				}
			}

			auto visitor = [&](Grid3D::Index index, const std::shared_ptr<GridCell>& cell) {
				auto currentCell = grid.get(index);

				if (!currentCell)
					return;

				if (currentCell->GetSize() <= 0)
					return;

				size_t i = index[2] * xSize * ySize + index[1] * xSize + index[0];

				if (processed.find(i) != processed.end())
					return;

				std::vector<Grid3D::Index> seed_queue;

				int sq_idx = 0;
				seed_queue.push_back(index);
				processed.insert(i);

				while (sq_idx < (int)seed_queue.size())
				{
					Grid3D::Index index = seed_queue[sq_idx];
					auto cell = grid.get(index[0], index[1], index[2]);

					if (!cell || cell->GetSize() <= 0)
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
						size_t j = zii * xSize * ySize + yii * xSize + xii;

						if (processed.find(j) != processed.end())
							continue;

						auto cellNbr = grid.get(xii, yii, zii);

						if (cellNbr && cellNbr->GetSize() > 0)
						{
							seed_queue.push_back({ xii, yii, zii });
							processed.insert(j);
						}
					}

					sq_idx++;

				} // end of while

				if (!seed_queue.empty())
					clusters.push_back(seed_queue);
			};

			grid.Visit(visitor);
		}

		void EuclideanVoxelCluster(PointCloudViewPtr input,
								   const std::vector<int>& indices,
								   double tolerance,
								   unsigned int minPts,
								   unsigned int maxPts,
								   std::vector<std::vector<int>>& clusters)
		{
			if (indices.empty())
			{
				PCS_WARN("[EuclideanVoxelCluster] 聚类输入点集为空.");
				return;
			}

			Grid3D grid;
			ComputeLocalGridCells(tolerance, input, indices, eUnclassified, grid);

			std::vector<std::vector<Grid3D::Index>> components;
			GetVoxelGridConnectComponents(grid, components);

			// 排除非电力线组件和点数不足的组件
			for (auto iter = components.begin(); iter != components.end(); ++iter)
			{
				const auto& component = *iter;
				std::vector<int> cluster;

				for (size_t i = 0; i < component.size(); ++i)
				{
					const Grid3D::Index& index = component[i];
					auto cell = grid.get(index[0], index[1], index[2]);

					if (!cell)
						continue;

					cluster.insert(cluster.end(),
								   cell->GetIndices().begin(),
								   cell->GetIndices().end());
				}

				//[问题]：单个噪点无法识别
				//[历史原因]：单个点的网格划分被剔除
				//[修改人]：李成 2023/09/11
				if (cluster.size() >= minPts && cluster.size() < maxPts)
					clusters.push_back(cluster);
			}
		}

	}
}