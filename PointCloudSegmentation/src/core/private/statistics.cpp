#include <src/core/private/statistics.h>
#include <src/core/pointTypes.hpp>
#include <src/algorithm/math.h>
#include <src/utils/logging.h>

#include <pcl/PCLPointField.h>
#include <pcl/common/io.h>
#include <numeric>

namespace d3s {
	namespace pcs {

		void computeMinMax(const PointCloudView<PointPCLH>& pcv,
						   const std::string& fieldname,
						   int minpts,
						   double step,
						   double& min,
						   double& max)
		{
			std::vector<int> all(pcv.size());
			std::iota(all.begin(), all.end(), 0);

			computeMinMax(pcv, all, fieldname, minpts, step, min, max);
		}

		void computeMinMax(const PointCloudView<PointPCLH>& pcv,
						   const std::vector<int>& indices,
						   const std::string& fieldname,
						   int minpts,
						   double step,
						   double& min,
						   double& max)
		{
			std::vector<pcl::PCLPointField> fields;
			int distance_idx = pcl::getFieldIndex<PointPCLH>(fieldname, fields);

			if (distance_idx == -1)
			{
				PCS_WARN("[computeMinMax] 未知的字段定义 %s.", fieldname.c_str());
				return;
			}

			// 桶排序，处理超大量元素
			// 根据bbox的 z 值范围，按照0.5m的步长设置“桶”的数量
			std::atomic<double> field_min ( DBL_MAX);
			std::atomic<double> field_max ( - DBL_MAX);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)indices.size(); ++i)
			{
				const uint8_t* pt_data = reinterpret_cast<const uint8_t*>(&pcv.points[indices[i]]);

				float field_value = 0;
				memcpy(&field_value, pt_data + fields[distance_idx].offset, sizeof(float));

				if (field_value > field_max)
					field_max = field_value;

				if (field_value < field_min)
					field_min = field_value;
			}

			int numBucket = std::floor((field_max - field_min) / step) + 1;

			if (numBucket <= 0)
			{
				PCS_WARN("[computeMinMax] 分配桶失败.");
				return;
			}

			std::vector<std::vector<int>> buckets;
			buckets.resize(numBucket);

			for (int i = 0; i < (int)indices.size(); ++i)
			{
				const uint8_t* pt_data = reinterpret_cast<const uint8_t*>(&pcv.points[indices[i]]);

				float field_value = 0;
				memcpy(&field_value, pt_data + fields[distance_idx].offset, sizeof(float));

				int index = std::floor(((double)field_value - field_min) / step);
				index = clamp(index, 0, numBucket - 1);

				buckets[index].push_back(indices[i]);
			}

			int i, j;
			for (i = 0; i < numBucket; ++i)
				if (buckets[i].size() > (size_t)minpts)
					break;

			for (j = numBucket - 1; j >= 0; --j)
				if (buckets[j].size() > (size_t)minpts)
					break;

			if (i < j)
			{
				const auto& minBin = buckets[i];
				const auto& maxBin = buckets[j];

				for (size_t ib = 0; ib < minBin.size(); ++ib)
				{
					const uint8_t* pt_data =
						reinterpret_cast<const uint8_t*>(&pcv.points[minBin[ib]]);

					float field_value = 0;
					memcpy(&field_value, pt_data + fields[distance_idx].offset, sizeof(float));

					min = std::min(min, (double)field_value);
				}

				for (size_t ib = 0; ib < maxBin.size(); ++ib)
				{
					const uint8_t* pt_data =
						reinterpret_cast<const uint8_t*>(&pcv.points[maxBin[ib]]);

					float field_value = 0;
					memcpy(&field_value, pt_data + fields[distance_idx].offset, sizeof(float));

					max = std::max(max, (double)field_value);
				}
			}
			else
			{
				for (int k = 0; k < numBucket; ++k)
				{
					PCS_INFO("buccket(%lf - %lf), number of points: %d.",
							 field_min + k * step,
							 field_min + (k + 1) * step,
							 buckets[k].size());
				}
			}
		}

		void computeMinMax(const PointCloudView<PointPCLH>& pcv,
						   const std::vector<int>& indices,
						   const std::string& fieldname,
						   ClassificationType label,
						   int minpts,
						   double step,
						   double& min,
						   double& max)
		{
			std::vector<pcl::PCLPointField> fields;
			int distance_idx = pcl::getFieldIndex<PointPCLH>(fieldname, fields);

			if (distance_idx == -1)
			{
				PCS_WARN("[computeMinMax] 未知的字段定义 %s.", fieldname.c_str());
				return;
			}

			std::atomic<double> field_min ( DBL_MAX);
			std::atomic<double> field_max ( - DBL_MAX);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)indices.size(); ++i)
			{
				if (pcv.points[indices[i]].label != (const uint32_t)label)
					continue;
				const uint8_t* pt_data = reinterpret_cast<const uint8_t*>(&pcv.points[indices[i]]);

				float field_value = 0;
				memcpy(&field_value, pt_data + fields[distance_idx].offset, sizeof(float));

				if (field_value > field_max)
					field_max = field_value;

				if (field_value < field_min)
					field_min = field_value;
			}

			if (field_min > field_max)
				return;

			int numBucket = std::floor((field_max - field_min) / step) + 1;

			if (numBucket <= 0)
			{
				PCS_WARN("[computeMinMax] 分配桶失败.");
				return;
			}

			std::vector<std::vector<int>> buckets;
			buckets.resize(numBucket);

			for (int i = 0; i < (int)indices.size(); ++i)
			{
				if (pcv.points[indices[i]].label != (const uint32_t)label)
					continue;

				const uint8_t* pt_data = reinterpret_cast<const uint8_t*>(&pcv.points[indices[i]]);

				float field_value = 0;
				memcpy(&field_value, pt_data + fields[distance_idx].offset, sizeof(float));

				int index = std::floor(((double)field_value - field_min) / step);
				index = clamp(index, 0, numBucket - 1);

				buckets[index].push_back(indices[i]);
			}

			int i, j;
			for (i = 0; i < numBucket; ++i)
				if (buckets[i].size() > (size_t)minpts)
					break;

			for (j = numBucket - 1; j >= 0; --j)
				if (buckets[j].size() > (size_t)minpts)
					break;


			if (i < j)
			{
				const auto& minBin = buckets[i];
				const auto& maxBin = buckets[j];

				for (size_t ib = 0; ib < minBin.size(); ++ib)
				{
					const uint8_t* pt_data =
						reinterpret_cast<const uint8_t*>(&pcv.points[minBin[ib]]);

					float field_value = 0;
					memcpy(&field_value, pt_data + fields[distance_idx].offset, sizeof(float));

					min = std::min(min, (double)field_value);
				}

				for (size_t ib = 0; ib < maxBin.size(); ++ib)
				{
					const uint8_t* pt_data =
						reinterpret_cast<const uint8_t*>(&pcv.points[maxBin[ib]]);

					float field_value = 0;
					memcpy(&field_value, pt_data + fields[distance_idx].offset, sizeof(float));

					max = std::max(max, (double)field_value);
				}
			}
			else
			{
				for (int k = 0; k < numBucket; ++k)
				{
					PCS_INFO("buccket(%lf - %lf), number of points: %d.",
							 field_min + k * step,
							 field_min + (k + 1) * step,
							 buckets[k].size());
				}
			}
		}

		void computeMinMax3D(const PointCloudView<PointPCLH>& pcv,
							 const std::vector<int>& indices,
							 osg::BoundingBox& bbox)
		{
			std::atomic<double> xmin ( DBL_MAX);
			std::atomic<double> ymin ( DBL_MAX);
			std::atomic<double> zmin ( DBL_MAX);

			std::atomic<double> xmax ( - DBL_MAX);
			std::atomic<double> ymax ( - DBL_MAX);
			std::atomic<double> zmax(-DBL_MAX);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)indices.size(); ++i)
			{
				const auto& p = pcv[indices[i]];

				if (p.x < xmin)
					xmin = p.x;
				if (p.y < ymin)
					ymin = p.y;
				if (p.z < zmin)
					zmin = p.z;

				if (p.x > xmax)
					xmax = p.x;
				if (p.y > ymax)
					ymax = p.y;
				if (p.z > zmax)
					zmax = p.z;
			}

			bbox = osg::BoundingBox(xmin, ymin, zmin, xmax, ymax, zmax);
		}

		void computeMinMax3D(const PointCloudView<PointPCLH>& pcv, osg::BoundingBox& bbox) 
		{
			std::vector<int> indices(pcv.size());
			std::iota(indices.begin(), indices.end(), 0);

			computeMinMax3D(pcv, indices, bbox);
		}

	}
}