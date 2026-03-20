// 点云深度学习分割数据预处理
//////////////////////////////////////////////////////////////////////////
#include <src/core/deeplearning/data_process.h>
#include <src/core/private/filters.h>
#include <src/core/private/statistics.h>
#include <src/core/private/cloudProcess.h>
#include <src/plot/plotHandle.h>
#include <src/utils/logging.h>

#include <numeric>

#define MIN_POINTS_BLOCK 100
#define RENDER_BLOCK_DEBUG

namespace d3s {
	namespace pcs {
		//////////////////////////////////////////////////////////////////////////
		DataProcess::DataProcess(PointCloudViewPtr cloud) : _cloud(cloud)
		{
			_indices.resize(_cloud->size());
			std::iota(_indices.begin(), _indices.end(), 0);
		}

		DataProcess::DataProcess(PointCloudViewPtr cloud, const std::vector<int>& indices)
			: _cloud(cloud), _indices(indices)
		{
		}

		void DataProcess::GenerateBlocks(size_t numPoint,
										 double blockSize,
										 double stride,
										 std::vector<std::vector<int>>& blockIndices,
										 std::vector<PointCloudViewPtr>& blocks)
		{
			PCS_INFO("[DataProcess] 开始进行点云数据分块...");
			PCS_INFO("\n\t numPoint = %d\n\t blockSize = %lf\n\t stride = %lf\n\t",
					 numPoint,
					 blockSize,
					 stride);

			blocks.clear();

			if (numPoint <= 0 || blockSize <= 1.0 || stride <= 1.0)
				return;

			if (stride > blockSize)
			{
				PCS_WARN(
					"[DataProcess] parameter 'stride' must greater than parameter 'blockSize'");

				return;
			}

			// 计算点云边界框
			osg::BoundingBox bbox;
			computeMinMax3D(*_cloud, bbox);

			// ceil向上取整
			int numCols = int(std::ceil((bbox.xMax() - bbox.xMin() - blockSize) / stride)) + 1;
			int numRows = int(std::ceil((bbox.yMax() - bbox.yMin() - blockSize) / stride)) + 1;

			std::vector<double> xbegin_list, ybegin_list;

			for (int r = 0; r < numRows; ++r)
			{
				for (int c = 0; c < numCols; ++c)
				{
					xbegin_list.push_back(c * stride);
					ybegin_list.push_back(r * stride);
				}
			}

			std::vector<std::vector<int>> blocks_indices(numRows * numCols);
			std::vector<bool> mask(numRows * numCols, false);

			// 统计分块点云
#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int idx = 0; idx < (int)xbegin_list.size(); ++idx)
			{
				double xbegin = xbegin_list[idx];
				double ybegin = ybegin_list[idx];
				auto& block = blocks_indices[idx];

				for (size_t i = 0; i < _indices.size(); ++i)
				{
					const auto& p = _cloud->points[_indices[i]];

					if ((p.x >= xbegin && p.x <= xbegin + blockSize) &&
						(p.y >= ybegin && p.y <= ybegin + blockSize))
					{
						block.push_back(_indices[i]);

						if (p.label == 3 || p.label == 4)
							mask[idx] = true;
					}
				}
			}

#ifdef RENDER_BLOCK_DEBUG
			std::vector<double> xlist, ylist;

			for (size_t i = 0; i < xbegin_list.size(); ++i)
			{
				if (!mask[i])
					continue;

				xlist.push_back(xbegin_list[i]);
				ylist.push_back(ybegin_list[i]);
			}

			RenderBlocks(xlist, ylist, blockSize, _cloud);
#endif

			size_t max_indices_num = 0;
			size_t min_indices_num = _indices.size();

			for (size_t i = 0; i < blocks_indices.size(); ++i)
			{
				const auto& indices = blocks_indices[i];
				max_indices_num = std::max(max_indices_num, indices.size());
				min_indices_num = std::min(min_indices_num, indices.size());
			}

			size_t numPointsSampled = 0;

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)blocks_indices.size(); ++i)
			{
				if (!mask[i])
					continue;

				auto& indices = blocks_indices[i];
				// indices = randomSampling(indices, numPoint);
				indices = farthestPointSampling(*_cloud, indices, numPoint);
				numPointsSampled += indices.size();
			}

			blockIndices.clear();
			for (int i = 0; i < (int)blocks_indices.size(); ++i)
			{
				if (!mask[i])
					continue;

				blockIndices.push_back(blocks_indices[i]);
			}

			// 构造点云
			for (const auto& indices : blockIndices)
			{
				PointCloudViewPtr blockCloud = new PointCloudView<PointPCLH>();
				blockCloud->epsg = _cloud->epsg; // 坐标系
				blockCloud->reserve(indices.size());

				for (const auto& index : indices)
					blockCloud->push_back(_cloud->points[index]);

				computeMinMax3D(*blockCloud, blockCloud->bbox);
				blocks.push_back(blockCloud);
			}

			PCS_INFO("[DataProcess] 点云数据分块完成.");
			PCS_INFO(
				"\n\t 块数量 = %d\n\t 最大块点数 = %d\n\t 最小块点数 = %d\n\t 采样后总点数 = %d",
				blockIndices.size(),
				max_indices_num,
				min_indices_num,
				numPointsSampled);
		}

		void DataProcess::Normalized(std::vector<PointCloudViewPtr>& blocks)
		{
#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)blocks.size(); ++i)
			{
				PointCloudViewPtr blockCloud = blocks[i];
				
				double xmean = 0.0;
				double ymean = 0.0;
				double zmean = 0.0;
				double max_distance = 0.0;

				size_t size = blockCloud->size();

				for (size_t j = 0; j < size; ++j)
				{
					auto& p = blockCloud->points[j];

					xmean += (p.x / (double)size);
					ymean += (p.y / (double)size);
					zmean += (p.z / (double)size);
				}

				osg::Vec3 centroid(xmean, ymean, zmean);
				blockCloud->bbox._min -= centroid;
				blockCloud->bbox._max -= centroid;

				for (size_t j = 0; j < size; ++j)
				{
					auto& p = blockCloud->points[j];

					p.x -= centroid.x();
					p.y -= centroid.y();
					p.z -= centroid.z();

					double distance = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
					max_distance = std::max(max_distance, distance);
				}

				for (size_t j = 0; j < size; ++j)
				{
					auto& p = blockCloud->points[j];
					p.x /= max_distance;
					p.y /= max_distance;
					p.z /= max_distance;
				}
			}
		}
	}
}