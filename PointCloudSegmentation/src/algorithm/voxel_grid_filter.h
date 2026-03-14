/////////////////////////////////////////////////////////////////////
// 文件名称：voxel_grid_filter.h
// 功能描述：体素滤波
// 创建标识：吕伟	2022/4/21
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef VOXLE_GRID_FILTER_H_
#define VOXLE_GRID_FILTER_H_

#include <src/algorithm/hash.h>

#include <Eigen/Dense>

namespace d3s {
	namespace pcs {

		class AccumulatedPoint
		{
		public:
			AccumulatedPoint() : _num_of_points(0), _point(0.0f, 0.0f, 0.0f) {}

		public:
			void AddPoint(const Eigen::Vector3f& point)
			{
				_point += point;
				_num_of_points++;
			}

			Eigen::Vector3f GetAveragePoint() const { return _point / double(_num_of_points); }



		public:
			int _num_of_points;
			Eigen::Vector3f _point;
		};

		template <typename PointT>
		void VoxelGridSample(std::vector<PointT, Eigen::aligned_allocator<PointT>>& points,
							 const Eigen::Vector3f& min,
							 const Eigen::Vector3f& max,
							 double voxelsize)
		{
			Eigen::Vector3f voxel_size3 = Eigen::Vector3f(voxelsize, voxelsize, voxelsize);
			std::unordered_map<Eigen::Vector3i, AccumulatedPoint, hash_eigen<Eigen::Vector3i>>
				voxelindex_to_accpoint;

			Eigen::Vector3f ref_coord;
			Eigen::Vector3i voxel_index;

			for (int i = 0; i < (int)points.size(); i++)
			{
				Eigen::Vector3f point3f = points[i].getVector3fMap();
				Eigen::Vector3f pointRef = point3f - min;
				// int label = points[i].label;

				// 体素索引
				ref_coord = pointRef / voxelsize;
				voxel_index << int(floor(ref_coord(0))), int(floor(ref_coord(1))),
					int(floor(ref_coord(2)));

				voxelindex_to_accpoint[voxel_index].AddPoint(pointRef);
			}

			std::vector<PointT, Eigen::aligned_allocator<PointT>> new_points;

			for (const auto& accpoint : voxelindex_to_accpoint)
			{
				PointT p;
				p.getVector3fMap() = min + accpoint.second.GetAveragePoint();
				// p.label = accpoint.first(3);
				new_points.emplace_back(p);
			}

			points = new_points;
		}
	}
}

#endif // VOXLE_GRID_FILTER_H_