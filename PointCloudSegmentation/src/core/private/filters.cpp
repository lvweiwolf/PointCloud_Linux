#include <src/core/private/filters.h>

#include <src/algorithm/approximate_pmf.h>
#include <src/algorithm/voxel_grid_filter.h>
#include <src/utils/logging.h>
#include <src/utils/timer.h>

#include <random>

namespace d3s {
	namespace pcs {

		bool groundSegmentation_ApproximatePMF(PointCloudView<PointPCLH>& pcv,
											   const std::vector<int>& indices,
											   std::vector<int>& ground,
											   int max_window_size,
											   float slope,
											   float max_dist,
											   float init_dist,
											   float cellsize,
											   float base,
											   bool exponential /*= true*/)
		{
			Timer timer;
			timer.Start();

			pcl::IndicesPtr indicesPtr(new std::vector<int>());
			*indicesPtr = indices;

			// 拷贝点
			pcl::PointCloud<PointPCLH>::Ptr cloud(new pcl::PointCloud<PointPCLH>);
			cloud->width = pcv.points.size();
			cloud->height = 1;

			std::swap(cloud->points, pcv.points);

			// PMF 算法提取地面点
			ApproximatePMF<PointPCLH> apmf;
			apmf.setInputCloud(cloud);
			apmf.setIndices(indicesPtr);
			apmf.setMaxWindowSize(max_window_size);
			apmf.setSlope(slope);
			apmf.setMaxDistance(max_dist);
			apmf.setInitialDistance(init_dist);
			apmf.setCellSize(cellsize);
			apmf.setBase(base);
			apmf.setExponential(exponential);
			apmf.extract(ground);

			// 还原内存
			std::swap(cloud->points, pcv.points);

			PCS_INFO("[groundSegmentation_ApproximatePMF] 地面点提取  %.3f s.",
					 timer.ElapsedSeconds());

			return true;
		}

		bool groundSegmentation_ApproximatePMF(PointCloudView<PointPCLH>& pcv,
											   std::vector<int>& ground,
											   int max_window_size,
											   float slope,
											   float max_dist,
											   float init_dist,
											   float cellsize,
											   float base,
											   bool exponential /*= true*/)
		{
			std::vector<int> all(pcv.size());
			std::iota(all.begin(), all.end(), 0);

			return groundSegmentation_ApproximatePMF(pcv,
													 all,
													 ground,
													 max_window_size,
													 slope,
													 max_dist,
													 init_dist,
													 cellsize,
													 base,
													 exponential);
		}


		void cloudRandomSampling(const PointCloudView<PointPCLH>& input,
								 size_t numberOfPoints,
								 PointCloudView<PointPCLH>& output)
		{
			size_t cloudSize = input.size();

			output = input;

			if (cloudSize <= numberOfPoints)
				return;

			size_t pointsToRemove = cloudSize - numberOfPoints;
			std::random_device rd;	// non-deterministic generator
			std::mt19937 gen(rd()); // to seed mersenne twister.

			size_t lastPointIndex = cloudSize - 1;

			for (size_t i = 0; i < pointsToRemove; ++i)
			{
				std::uniform_int_distribution<size_t> dist(0, lastPointIndex);
				size_t index = dist(gen);
				std::swap(output.points[index], output.points[lastPointIndex]);
				--lastPointIndex;
			}

			output.points.resize(numberOfPoints);
		}

		std::vector<int> randomSampling(const std::vector<int>& indices, size_t numSample)
		{
#if 0
			std::vector<int> result(numSample);

			if (indices.empty())
				return result;

			if (indices.size() > numSample)
			{
				std::vector<int> indicesCopy = indices;
				size_t arrLength = indicesCopy.size();

				std::random_device rd;	// non-deterministic generator
				std::mt19937 gen(rd()); // to seed mersenne twister.

				for (size_t i = 0; i < numSample; ++i)
				{
					std::uniform_int_distribution<size_t> dist(0, arrLength - 1);

					size_t index = dist(gen); // 生成随机索引
					result[i] = indicesCopy[index];
					indicesCopy[i] = indicesCopy[arrLength - 1];
					arrLength--;
				}
			}
			else
				result = indices;


			return result;
#else

			if (!numSample)
				return std::vector<int>();

			std::vector<int> result;

			size_t N = indices.size();

			// 采样数量超过原始数据总数，直接返回原始数据
			if (numSample >= N)
			{
				result = indices;
			}
			else
			{
				// 调整输出大小为采样大小
				result.resize(static_cast<size_t>(numSample));

				// 设置随机种子，以便每次运行筛选器时派生的标记示例都相同
				// std::srand(SEED);
				std::random_device rd;
				std::mt19937 gen(rd());
				std::uniform_real_distribution<float> dist(0, 1); // 随机分布

				// Algorithm A
				unsigned top = N - numSample;
				unsigned i = 0;
				unsigned index = 0;

				for (size_t n = numSample; n >= 2; n--)
				{
					float V = dist(gen);
					unsigned S = 0;
					float quot = float(top) / float(N);
					while (quot > V)
					{
						S++;
						top--;
						N--;
						quot = quot * float(top) / float(N);
					}
					index += S;
					result[i++] = indices[index++];
					N--;
				}

				index += N * static_cast<unsigned>(dist(gen));
				result[i++] = indices[index++];
			}

			return result;
#endif
		}

		std::vector<int> farthestPointSampling(const PointCloudView<PointPCLH>& input,
											   const std::vector<int>& indices,
											   size_t numSample)
		{
			if (indices.empty())
				return std::vector<int>();

			if (indices.size() < numSample)
				return indices;

			auto SquareDistance = [](const PointPCLH& p1, const PointPCLH& p2) {
				double dx = p1.x - p2.x;
				double dy = p1.y - p2.y;
				double dz = p1.z - p2.z;

				return dx * dx + dy * dy + dz * dz;
			};

			std::vector<int> result(numSample);

			// 随机选择一个点
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_int_distribution<size_t> dst(0, indices.size() - 1);

			size_t index = dst(gen);
			result[0] = indices[index];

			// 计算点云到第一个点的距离，并选择最远点
			std::vector<double> distances(indices.size());

			for (size_t i = 0; i < indices.size(); ++i)
				distances[i] = SquareDistance(input[indices[i]], input[indices[index]]);

			for (size_t i = 1; i < numSample; ++i)
			{
				int farthest =
					std::max_element(distances.begin(), distances.end()) - distances.begin();

				result[i] = indices[farthest];

				// 更新点云距离列表
				for (size_t j = 0; j < indices.size(); ++j)
				{
					distances[j] =
						std::min(distances[j],
								 SquareDistance(input[indices[farthest]], input[indices[j]]));
				}
			}

			return result;
		}

	}
}