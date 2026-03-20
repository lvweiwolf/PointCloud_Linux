#include <src/core/api.h>
#include <src/core/pointTypes.hpp>

#include <src/io/pointCloudWriter.h>
#include <src/io/pointCloudToLod.h>

#include <src/algorithm/raster.h>
#include <src/algorithm/gdal_raster.h>
#include <src/algorithm/voxel_grid_filter.h>
#include <unordered_map>
#include <src/utils/logging.h>
#include <src/utils/timer.h>
#include <src/utils/misc.h>

// PCL
#include <pcl/search/kdtree.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

// LASLib
#include <lasreader.hpp>
#include <bytestreamin.hpp>

// Boost
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>


namespace d3s {
	namespace pcs {

		// api.h 接口的实现
		//////////////////////////////////////////////////////////////////////////
		bool readPointCloud(const std::string& filename,
							PointCloudView<PointPCLH>& pcv,
							bool voxelSample,
							double voxelSize)
		{
			Timer timer;
			timer.Start();

			io::PointCloudReader reader(filename);

			if (!reader.IsValid())
				return false;

			double readSize = 0;
			size_t numProcess = 0;
			size_t numPoints = reader.GetNumOfPoints();
			pcv.bbox = reader.GetBoundingBox();
			pcv.offset_xyz = reader.GetOffset(); // 坐标偏移
			pcv.epsg = reader.GetEPSG();
			pcv.clear();

			std::unordered_map<Eigen::Vector3i, AccumulatedPoint, hash_eigen<Eigen::Vector3i>>
				voxelindex_to_accpoint;

			Eigen::Vector3f ref_coord;
			Eigen::Vector3i voxel_index;

			Eigen::Vector3f min(pcv.bbox.xMin(), pcv.bbox.yMin(), pcv.bbox.zMin());
			Eigen::Vector3f max(pcv.bbox.xMax(), pcv.bbox.yMax(), pcv.bbox.zMax());

			while (numProcess < numPoints)
			{
				PointPCLH p;

				if (reader.ReadNextPoint(p))
				{
					if (voxelSample)
					{
						Eigen::Vector3f point3f = p.getVector3fMap();
						Eigen::Vector3f pointRef = point3f - min;

						ref_coord = pointRef / voxelSize;
						voxel_index << int(floor(ref_coord(0))), int(floor(ref_coord(1))),
							int(floor(ref_coord(2)));

						voxelindex_to_accpoint[voxel_index].AddPoint(pointRef);
					}
					else
					{
						pcv.push_back(p);
					}

					readSize += sizeof(PointPCLH) / (1024.0 * 1024.0);
					++numProcess;
				}
				else
					break;
			}

			pcv.bbox._min -= pcv.offset_xyz;
			pcv.bbox._max -= pcv.offset_xyz;

			PCS_INFO("[readPointCloud] 读取点云结束 (%d / %d)，内存 %.3f MB，耗时 %.3f s.",
					 numProcess,
					 numPoints,
					 readSize,
					 timer.ElapsedSeconds());

			if (voxelSample)
			{
				timer.Restart();

				for (const auto& accpoint : voxelindex_to_accpoint)
				{
					PointPCLH p;
					p.getVector3fMap() = min + accpoint.second.GetAveragePoint();
					pcv.push_back(p);
				}

				PCS_INFO("[readPointCloud] 点云抽稀(leafSize=%lf)后点云数(%d/%d)，耗时 %.3f s.",
						 voxelSize,
						 pcv.size(),
						 numPoints,
						 timer.ElapsedSeconds());
			}

			return true;
		}

		bool readPointCloud2(const std::string& filename, PointCloudView<PointPCLH>& pcv)
		{
			LASreadOpener lasFile;
			lasFile.set_file_name(filename.c_str());
			LASreader* lasReader = lasFile.open();
			/*
			size_t numPoints = lasReader->header.number_of_point_records;

			for (size_t i = 0; i < numPoints; ++i)
			{
				if (i % 50 == 0)
				{
					lasReader->seek(i);
					CHECK(lasReader->read_point());

					PointPCLH p;
					p.x = lasReader->point.get_x();
					p.y = lasReader->point.get_y();
					p.z = lasReader->point.get_z();

					pcv.push_back(p);
				}
			}*/


			size_t pos = 300000000;
			I64 cur = 0;
			for (size_t i = pos; i < pos + 10; ++i)
			{
				lasReader->seek(i);
				I64 cur = lasReader->get_stream()->tell();
				PCS_INFO("%d", cur);

				CHECK(lasReader->read_point());
				cur = lasReader->get_stream()->tell();

				double x = lasReader->point.get_x();
				double y = lasReader->point.get_y();
				double z = lasReader->point.get_z();

				PCS_INFO("point XYZ: %lf, %lf, %lf, %d", x, y, z, cur);
			}

			/*std::ifstream fs(filename);

			if (fs.is_open())
			{
				size_t pos = 10200000227;
				fs.seekg(pos, std::ios::beg);

				char buffer[100] = { 0 };
				fs.read(buffer, 100);
				auto cur = fs.tellg();

				PCS_INFO(StringPrintf("%ld"), cur);
			}*/

			return true;
		}

		bool writePointCloud(const std::string& filename, const PointCloudView<PointPCLH>& pcv)
		{
			io::PointCloudWriter writer(filename);

			return writer.Write(pcv);
		}

		bool writePointCloud(const std::string& filename,
							 const PointCloudView<PointPCLH>& pcv,
							 const std::vector<int>& indices)
		{
			io::PointCloudWriter writer(filename);

			return writer.Write(pcv, indices);
		}


		void denoiseStatistical(PointCloudView<PointPCLH>& pcv, int meanK, double stdMul)
		{
			Timer timer;
			timer.Start();

			std::vector<int> indices(pcv.points.size());
			std::iota(indices.begin(), indices.end(), 0);

			std::vector<int> inliers, outliers;
			denoiseStatistical(pcv, indices, meanK, stdMul, inliers, outliers);
			CHECK(indices.size() <= pcv.points.size());

			PointCloudView<PointPCLH>::PointCloud points;
			points.resize(inliers.size());

			std::atomic<double> xmin(DBL_MAX);
			std::atomic<double> ymin(DBL_MAX);
			std::atomic<double> zmin(DBL_MAX);
			std::atomic<double> xmax(-DBL_MAX);
			std::atomic<double> ymax(-DBL_MAX);
			std::atomic<double> zmax(-DBL_MAX);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)inliers.size(); ++i)
			{
				points[i] = pcv.points[inliers[i]];

				const auto& p = points[i];

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
			} // cloud -> newPcv

			pcv.bbox = osg::BoundingBox(xmin, ymin, zmax, xmax, ymax, zmax);
			std::swap(pcv.points, points);

			PCS_INFO("[denoiseStatistical] 点云降噪(meanK=%d, "
					 "stdMul=%lf)后点云数(%d/%d)，耗时 %.3f s.",
					 meanK,
					 stdMul,
					 inliers.size(),
					 indices.size(),
					 timer.ElapsedSeconds());
		}


		void denoiseStatistical(PointCloudView<PointPCLH>& pcv,
								const std::vector<int>& indices,
								int meanK,
								double stdMul,
								std::vector<int>& inliners,
								std::vector<int>& outliers)
		{
			inliners.clear();

			// 拷贝点
			pcl::PointCloud<PointPCLH>::Ptr cloud(new pcl::PointCloud<PointPCLH>);
			std::swap(cloud->points, pcv.points); // pcv -> cloud
			cloud->width = cloud->points.size();
			cloud->height = 1;

			{
				pcl::search::Search<PointPCLH>::Ptr kdtree(new pcl::search::KdTree<PointPCLH>());

				{
					pcl::IndicesPtr indicesPtr(new std::vector<int>());
					*indicesPtr = indices;
					kdtree->setInputCloud(cloud, indicesPtr);
				}


				std::vector<float> distances(indices.size());

				// 第一遍：计算所有点相对于k个最近邻居的平均距离
				std::atomic<int> valid_distances(0);

#ifdef _OPENMP
#pragma omp parallel for
#endif
				for (int i = 0; i < (int)indices.size(); ++i)
				{
					if (!pcl_isfinite(cloud->points[indices[i]].x) ||
						!pcl_isfinite(cloud->points[indices[i]].y) ||
						!pcl_isfinite(cloud->points[indices[i]].z))
					{
						distances[i] = 0.0;
						continue;
					}

					std::vector<int> nn_indices(meanK);
					std::vector<float> nn_dists(meanK);

					// 执行最近K邻域搜索
					if (kdtree->nearestKSearch(indices[i], meanK + 1, nn_indices, nn_dists) == 0)
					{
						distances[i] = 0.0;
						PCS_WARN("[denoiseStatistical] Searching for the closest %d neighbors "
								 "failed.\n",
								 meanK);
						continue;
					}

					// 计算与邻居的平均距离
					double dist_sum = 0.0;
					for (int k = 1; k < meanK + 1; ++k) // k = 0 是查询点
						dist_sum += sqrt(nn_dists[k]);

					distances[i] = static_cast<float>(dist_sum / meanK);
					valid_distances++;
				}

				// 估计距离向量的平均值和标准差
				double sum = 0, sq_sum = 0;
				for (size_t i = 0; i < distances.size(); ++i)
				{
					sum += distances[i];
					sq_sum += distances[i] * distances[i];
				}

				double mean = sum / static_cast<double>(valid_distances);
				double variance = (sq_sum - sum * sum / static_cast<double>(valid_distances)) /
								  (static_cast<double>(valid_distances) - 1);
				double stddev = sqrt(variance);
				double distance_threshold = mean + stdMul * stddev;

				// 第二遍：根据计算的距离阈值对点进行分类
				inliners.resize(indices.size());
				outliers.resize(indices.size());
				int oii = 0, rii = 0;


				for (int i = 0; i < (int)indices.size(); ++i)
				{

					if (distances[i] > distance_threshold)
					{
						outliers[rii++] = indices[i];
						continue;
					}

					inliners[oii++] = indices[i];
				}

				inliners.resize(oii);
				outliers.resize(rii);
				CHECK(inliners.size() > 0);
			}

			// 还原点云数据
			std::swap(cloud->points, pcv.points);
		}


		void denoiseRadius(PointCloudView<PointPCLH>& pcv, double radius, int minpts)
		{
			typedef pcl::octree::OctreePointCloudSearch<PointPCLH> Octree;

			Timer timer;
			timer.Start();

#if 0
			std::vector<int> inliners;
			// 拷贝点
			pcl::PointCloud<PointPCLH>::Ptr cloud(new pcl::PointCloud<PointPCLH>);
			std::swap(cloud->points, pcv.points); // pcv -> cloud
			cloud->width = cloud->points.size();
			cloud->height = 1;

			pcl::RadiusOutlierRemoval<PointPCLH> filter;

			filter.setInputCloud(cloud);
			filter.setRadiusSearch(radius);
			filter.setMinNeighborsInRadius(3);
			filter.filter(inliners);
#else
			std::vector<int> inliners;

			pcl::PointCloud<PointPCLH>::Ptr cloud(new pcl::PointCloud<PointPCLH>);
			std::swap(cloud->points, pcv.points); // pcv -> cloud
			cloud->width = cloud->points.size();
			cloud->height = 1;


			// 使用八叉树近似计算
			Octree octree(radius * 2);
			octree.setInputCloud(cloud);	  // 设置点云数据
			octree.defineBoundingBox();		  // 自动更新边界框
			octree.addPointsFromInputCloud(); // 从点云数据添加点

			auto tree_it_end = octree.leaf_end();

			for (auto tree_it = octree.leaf_begin(); tree_it != tree_it_end; ++tree_it)
			{
				const auto& leafContainer = tree_it.getLeafContainer();

				if (leafContainer.getSize() <= (size_t)minpts)
					continue;

				leafContainer.getPointIndices(inliners);
			}
#endif
			CHECK(inliners.size() > 0);

			// 创建新点云数据
			PointCloudView<PointPCLH> newPcv;
			newPcv.resize(inliners.size());

			std::atomic<double> xmin(DBL_MAX);
			std::atomic<double> ymin(DBL_MAX);
			std::atomic<double> zmin(DBL_MAX);
			std::atomic<double> xmax(-DBL_MAX);
			std::atomic<double> ymax(-DBL_MAX);
			std::atomic<double> zmax(-DBL_MAX);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)inliners.size(); ++i)
			{
				newPcv.points[i] = cloud->points[inliners[i]];

				auto& p = newPcv.points[i];

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
			} // cloud -> newPcv

			newPcv.bbox = osg::BoundingBox(xmin, ymin, zmin, xmax, ymax, zmax);
			newPcv.offset_xyz = pcv.offset_xyz;
			newPcv.epsg = pcv.epsg;

			std::swap(pcv, newPcv); // newPcv -> pcv

			PCS_INFO("[denoiseRadius] 点云降噪(radius=%lf, "
					 "min_pts=%d)后点云数(%d/%d)，耗时 %.3f s.",
					 radius,
					 minpts,
					 inliners.size(),
					 cloud->size(),
					 timer.ElapsedSeconds());
		}

		void denoiseRange(PointCloudView<PointPCLH>& pcv,
						  const std::string& fieldname,
						  double low,
						  double high)
		{
			Timer timer;
			timer.Start();

			std::vector<int> inliners;
			PointCloudView<PointPCLH> newPcv;
			int numPointsOld = pcv.size();

			{
				// 拷贝点
				pcl::PointCloud<PointPCLH>::Ptr cloud(new pcl::PointCloud<PointPCLH>);
				std::swap(cloud->points, pcv.points); // pcv -> cloud
				cloud->width = cloud->points.size();
				cloud->height = 1;

				pcl::PassThrough<PointPCLH> filter;

				filter.setInputCloud(cloud);
				filter.setFilterFieldName(fieldname);
				filter.setFilterLimits(low, high);
				filter.filter(inliners);

				CHECK(inliners.size() > 0);
				newPcv.resize(inliners.size());

				std::atomic<double> xmin(DBL_MAX);
				std::atomic<double> ymin(DBL_MAX);
				std::atomic<double> zmin(DBL_MAX);
				std::atomic<double> xmax(-DBL_MAX);
				std::atomic<double> ymax(-DBL_MAX);
				std::atomic<double> zmax(-DBL_MAX);

#ifdef _OPENMP
#pragma omp parallel for
#endif
				for (int i = 0; i < (int)inliners.size(); ++i)
				{
					newPcv.points[i] = cloud->points[inliners[i]];

					auto& p = newPcv.points[i];

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
				} // cloud -> newPcv

				newPcv.bbox = osg::BoundingBox(xmin, ymin, zmin, xmax, ymax, zmax);
				newPcv.offset_xyz = pcv.offset_xyz;
				newPcv.epsg = pcv.epsg;
			}

			try
			{
				std::swap(pcv, newPcv); // newPcv -> pcv
			}
			catch (const std::bad_alloc& e)
			{
				PCS_ERROR(e.what());
			}

			PCS_INFO("[denoiseRange] 点云降噪(zLow=%lf, zHigh=%lf)后点云数(%d/%d)，耗时 %.3f s.",
					 low,
					 high,
					 inliners.size(),
					 numPointsOld,
					 timer.ElapsedSeconds());
		}

		void setClassification(ClassificationType label, PointCloudView<PointPCLH>& pcv)
		{
#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)pcv.size(); ++i)
				pcv.points[i].label = label;
		}

		void setClassification(ClassificationType label,
							   const std::vector<int>& indices,
							   PointCloudView<PointPCLH>& pcv)
		{
#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)indices.size(); ++i)
				pcv.points[indices[i]].label = label;
		}

		void getClassification(const PointCloudView<PointPCLH>& pcv,
							   ClassificationType label,
							   std::vector<int>& indices)
		{
			indices.clear();

			for (size_t i = 0; i < pcv.size(); ++i)
			{
				if (pcv[i].label == (const uint32_t)label)
					indices.push_back((int)i);
			}
		}

		void getClassification(const PointCloudView<PointPCLH>& pcv,
							   double xmin,
							   double ymin,
							   double xmax,
							   double ymax,
							   ClassificationType label,
							   std::vector<int>& indices)
		{
			indices.clear();

			for (int i = 0; i < (int)pcv.size(); ++i)
			{
				const auto& p = pcv[i];

				if (xmin <= p.x && ymin <= p.y && p.x < xmax && p.y < ymax &&
					p.label == (const uint32_t)label)
				{
					indices.push_back(i);
				}
			}
		}

		bool generatePointCloudLOD_KDTree(const std::string& input, const std::string& outdir)
		{
			Timer timer;
			timer.Start();

			float lodRatio = 1.0f;
			int tileSize = 1000000;
			int nodeSize = 5000;
			int depth = 99;
			int pointSize = 3.0f;

			io::PointCloudToLOD lod;

			if (lod.Export(input,
						   outdir,
						   "osgb",
						   tileSize,
						   nodeSize,
						   depth,
						   lodRatio,
						   pointSize,
						   "iHeightBlend"))
			{
				return true;
			}
			else
				return false;
		}

		bool normalizeByGround(const std::string& dtmfile, PointCloudView<PointPCLH>& pcv)
		{
			if (!ExistsFile(dtmfile))
				return false;

			gdal::Raster raster(dtmfile);
			gdal::GDALError error = raster.open();

			if (error != gdal::GDALError::None)
			{
				PCS_ERROR("读取地形文件 %s 失败.", dtmfile.c_str());
				return false;
			}

			int numBand = 1;

			for (int i = 0; i < (int)pcv.size(); ++i)
			{
				std::vector<double> data;

				double x = pcv[i].x;
				double y = pcv[i].y;
				double z = pcv[i].z;
				pcv[i].hag = 0;

				if (raster.read(x, y, data) == gdal::GDALError::None)
				{
					double zGround = data[numBand - 1];
					if (zGround <= -9990.0)
						continue;

					double heigtAboveGround = std::max(0.0, z - zGround);
					heigtAboveGround = std::min(heigtAboveGround, (double)pcv.bbox.zMax());

					pcv[i].hag = heigtAboveGround;

					if (pcv[i].hag > pcv.hag_max)
						pcv.hag_max = pcv[i].hag;

					if (pcv[i].hag < pcv.hag_min)
						pcv.hag_min = pcv[i].hag;
				}
			}

			pcv.normalized = true;
			return true;
		}

		bool normalizeByGround(const Rasterd& raster, PointCloudView<PointPCLH>& pcv)
		{
			if (raster.size() <= 0)
				return false;

			double halfEdge = raster.edgeLength() / 2.0;
			double edgeBit = raster.edgeLength() * .000001;

			std::atomic<double> hag_min(DBL_MAX);
			std::atomic<double> hag_max(-DBL_MAX);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)pcv.size(); ++i)
			{
				std::vector<double> data;

				double x = pcv[i].x;
				double y = pcv[i].y;
				double z = pcv[i].z;
				pcv[i].hag = 0;

				int xi = raster.xCell(x + halfEdge - edgeBit);
				int yi = raster.yCell(y + halfEdge - edgeBit);

				xi = clamp(xi, 0, raster.width() - 1);
				yi = clamp(yi, 0, raster.height() - 1);

				double zGround = raster.at(xi, yi);

				if (std::isnan(zGround) || zGround <= -9990.0)
				{
					int cmin = clamp(xi - 2, 0, raster.width() - 1);
					int cmax = clamp(xi + 2, 0, raster.width() - 1);
					int rmin = clamp(yi - 2, 0, raster.height() - 1);
					int rmax = clamp(yi + 2, 0, raster.height() - 1);

					std::vector<double> values;

					for (int r = rmin; r <= rmax; ++r)
					{
						for (int c = cmin; c <= cmax; ++c)
						{
							double zValue = raster.at(c, r);

							if (std::isnan(zValue) || zValue <= -9990.0)
								continue;

							values.push_back(zValue);
						}
					}

					if (values.empty())
						continue;
					else
						zGround = Mean(values);
				}

				double heigtAboveGround = std::max(0.0, z - zGround);
				heigtAboveGround = std::min(heigtAboveGround, (double)pcv.bbox.zMax());

				pcv[i].hag = heigtAboveGround;
				CHECK_MSG(!std::isnan(pcv[i].hag),
						  StringPrintf("z=%lf zGround=%lf", z, zGround).c_str());

				if (pcv[i].hag > hag_max)
					hag_max = pcv[i].hag;

				if (pcv[i].hag < hag_min)
					hag_min = pcv[i].hag;
			}

			pcv.hag_min = hag_min;
			pcv.hag_max = hag_max;
			pcv.normalized = true;
			return true;
		}

		bool generatePointCloudLOD_Octree(const std::string& input, const std::string& outdir)
		{
			throw std::runtime_error("Not Implement Yet!");

			return false;
		}

	}
}
