#include <src/service/PointCloud.h>
#include <src/core/api.h>
#include <src/core/pointTypes.hpp>
#include <src/segmentation/gridCell.h>
#include <src/utils/logging.h>
#include <src/utils/timer.h>

#include <include/ClassificationDef.h>

#include <pcl/PCLPointField.h>
#include <pcl/common/io.h>

#include <numeric>

namespace d3s {
	namespace pcs {

		CPointCloud::CPointCloud() {}

		CPointCloud::CPointCloud(PointCloudView<PointPCLH>& pcv) : _pcv(pcv) {}

		CPointCloud::~CPointCloud() {}

		void CPointCloud::Write(const std::string& filename) { writePointCloud(filename, _pcv); }

		void CPointCloud::SetFieldInternel(const char* pszField, PointId idx, const void* val)
		{
			CHECK(pszField);

			if (!pszField)
				return;

			std::string fieldname = pszField;
			std::vector<pcl::PCLPointField> fields;
			int distance_idx = pcl::getFieldIndex<PointPCLH>(fieldname, fields);

			if (distance_idx == -1)
				return;

			uint8_t* pt_data = reinterpret_cast<uint8_t*>(&_pcv.points[idx]);

			memcpy(pt_data + fields[distance_idx].offset,
				   val,
				   pcl::getFieldSize(fields[distance_idx].datatype));
		}

		void CPointCloud::GetFieldInternel(const char* pszField, PointId idx, void* val)
		{
			CHECK(pszField);

			if (!pszField)
				return;

			std::string fieldname = pszField;
			std::vector<pcl::PCLPointField> fields;
			int distance_idx = pcl::getFieldIndex<PointPCLH>(fieldname, fields);

			if (distance_idx == -1)
				return;

			const uint8_t* pt_data = reinterpret_cast<const uint8_t*>(&_pcv.points[idx]);

			memcpy(val,
				   pt_data + fields[distance_idx].offset,
				   pcl::getFieldSize(fields[distance_idx].datatype));
		}

		void CPointCloud::SetTreeId(PointId idx, uint32_t treeId)
		{
			if (idx < 0 || idx > _pcv.size())
				return;

			_pcv.points[idx].data[3] = (float)treeId;
		}

		uint32_t CPointCloud::GetTreeId(PointId idx) const
		{
			if (idx < 0 || idx > _pcv.size())
				return 0;

			return (uint32_t)_pcv.points[idx].data[3];
		}

		//[问题]：优化自动分类、单木分割性能
		//[历史原因]：通过GetFieldInternel、SetFieldInternel方式获取设置存在不必要的字符串匹配性能消耗
		//[修改策略]：通过索引直接获取
		//[修改人]：李成 2023/02/23
		void CPointCloud::SetClassification(PointId idx, uint32_t label)
		{
			_pcv.points[idx].label = label;
			// SetFieldInternel("label", idx, &label);
		}

		uint32_t CPointCloud::GetClassification(PointId idx)
		{
			return _pcv.points[idx].label;
			// uint32_t label = 0;
			// GetFieldInternel("label", idx, &label);
			// return label;
		}

		osg::Vec3 CPointCloud::GetXYZ(PointId idx)
		{
			return osg::Vec3(_pcv.points[idx].x, _pcv.points[idx].y, _pcv.points[idx].z);
		}

		void CPointCloud::SetSize(size_t size)
		{
			PointCloudViewPtr pcv = Data();
			pcv->points.resize(size);
		}

		size_t CPointCloud::GetSize() const { return _pcv.size(); }

		void CPointCloud::SetGeoInfo(const osg::Vec3d& offset, int epsg)
		{
			PointCloudViewPtr pcv = Data();
			pcv->offset_xyz = offset;
			pcv->epsg = epsg;

			PCS_INFO("[CPointCloud] 获得局部点云数量 %d, 大小 %lf MB.",
					 _pcv.size(),
					 (double)_pcv.size() * sizeof(PointPCLH) / (double)(1024 * 1024));

			PCS_INFO("[CPointCloud] Bound: (%lf, %lf, %lf, %lf, %lf, %lf)",
					 pcv->bbox.xMin(),
					 pcv->bbox.yMin(),
					 pcv->bbox.zMin(),
					 pcv->bbox.xMax(),
					 pcv->bbox.yMax(),
					 pcv->bbox.zMax());

			PCS_INFO("[CPointCloud] Offset: (%lf, %lf, %lf)", offset.x(), offset.y(), offset.z());
			PCS_INFO("[CPointCloud] EPSG: %d", epsg);
		}

		PointCloudViewPtr CPointCloud::Data() { return &_pcv; }

		PointCloudViewConstPtr CPointCloud::Data() const { return &_pcv; }


		void CPointCloud::ConvertFrom(const std::vector<POINT>& data,
									  const osg::Vec3d& offset,
									  int epsg)
		{
			if (data.empty())
				return;

			PointCloudViewPtr pcv = Data();
			pcv->clear();
			pcv->points.resize(data.size());

			std::atomic<double> xmin(DBL_MAX);
			std::atomic<double> ymin(DBL_MAX);
			std::atomic<double> zmin(DBL_MAX);

			std::atomic<double> xmax(-DBL_MAX);
			std::atomic<double> ymax(-DBL_MAX);
			std::atomic<double> zmax(-DBL_MAX);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)data.size(); ++i)
			{
				pcv->points[i].x = data[i].x;
				pcv->points[i].y = data[i].y;
				pcv->points[i].z = data[i].z;

				ClassificationType label;
				pcv->points[i].label = eUnclassified;

				// 更新边界
				const auto& p = pcv->points[i];

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

			pcv->bbox = osg::BoundingBox(xmin, ymin, zmin, xmax, ymax, zmax);
			pcv->offset_xyz = offset;
			pcv->epsg = epsg;


			PCS_INFO("[CPointCloud] Bound: (%lf, %lf, %lf, %lf, %lf, %lf)",
					 xmin.load(),
					 ymin.load(),
					 zmin.load(),
					 xmax.load(),
					 ymax.load(),
					 zmax.load());
			PCS_INFO("[CPointCloud] Offset: (%lf, %lf, %lf)", offset.x(), offset.y(), offset.z());
			PCS_INFO("[CPointCloud] EPSG: %d", epsg);
		}

		void CPointCloud::ConvertTo(std::vector<POINT>& data, osg::Vec3d& offset, int& epsg)
		{
			PointCloudViewConstPtr pcv = Data();
			CHECK(pcv);

			data.resize(pcv->size());

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)pcv->size(); ++i)
			{
				data[i].x = pcv->points[i].x;
				data[i].y = pcv->points[i].y;
				data[i].z = pcv->points[i].z;
				data[i].treeId = pcv->points[i].data[3];
				data[i].label = (char)(pcv->points[i].label);
			}

			epsg = pcv->epsg;
			offset = pcv->offset_xyz;
		}

		void CPointCloud::Denoise(double radius,
								  unsigned int minPts,
								  std::vector<int>& inliers,
								  std::vector<int>& outliers)
		{
			Timer timer;
			timer.Start();

			PointCloudViewPtr pcv = Data();
			CHECK(pcv);

			std::vector<int> indices(pcv->points.size());
			std::iota(indices.begin(), indices.end(), 0);

			// 欧式聚类
			std::vector<std::vector<int>> clusters;
			EuclideanVoxelCluster(pcv,
								  indices,
								  radius,
								  1,
								  std::numeric_limits<int>::max(),
								  clusters);

			for (size_t i = 0; i < clusters.size(); ++i)
			{
				const auto& cluster = clusters.at(i);

				if (cluster.size() < minPts)
				{
					outliers.insert(outliers.end(), cluster.begin(), cluster.end());
				}
				else
				{
					inliers.insert(inliers.end(), cluster.begin(), cluster.end());
				}
			}

			PCS_INFO("[CPointCloud] 点云降噪(radius=%lf, "
				"minPts=%d)后点云数(%d/%d)，耗时 %.3f s.",
					 radius,
					 minPts,
					 inliers.size(),
					 indices.size(),
					 timer.ElapsedSeconds());
		}

		void CPointCloud::DenoiseSOR(int meanK,
									 double stdMul,
									 std::vector<int>& inliers,
									 std::vector<int>& outliers)
		{
			Timer timer;
			timer.Start();

			PointCloudViewPtr pcv = Data();
			CHECK(pcv);

			std::vector<int> indices(pcv->points.size());
			std::iota(indices.begin(), indices.end(), 0);

			denoiseStatistical(*pcv, indices, meanK, stdMul, inliers, outliers);


			PCS_INFO("[CPointCloud] 点云降噪(meanK=%d, "
				"stdMul=%lf)后点云数(%d/%d)，耗时 %.3f s.",
					 meanK,
					 stdMul,
					 inliers.size(),
					 indices.size(),
					 timer.ElapsedSeconds());
		}

		void CPointCloud::FillOsgPoint(osg::ref_ptr<osg::Vec3Array> vertArray,
									   osg::Matrix& matrix,
									   int nIndex)
		{
			PointCloudViewPtr pcv = Data();
			double xmin = DBL_MAX;
			double ymin = DBL_MAX;
			double zmin = DBL_MAX;
			double xmax = -DBL_MAX;
			double ymax = -DBL_MAX;
			double zmax = -DBL_MAX;

			int nSize = vertArray->size();
			for (int i = 0; i < nSize; ++i)
			{
				int nTempIndex = i + nIndex;
				auto point = vertArray->at(i) * matrix;
				pcv->points[nTempIndex].x = point.x();
				pcv->points[nTempIndex].y = point.y();
				pcv->points[nTempIndex].z = point.z();

				pcv->points[nTempIndex].label = eUnclassified;

				// 更新边界
				const auto& p = pcv->points[nTempIndex];

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

			std::unique_lock<std::mutex> lock(_mutex);
			pcv->bbox.expandBy(osg::BoundingBox(xmin, ymin, zmin, xmax, ymax, zmax));
		}

		void CPointCloud::FillOsgPoint(const osg::Vec3& vert, int nIndex)
		{
			PointCloudViewPtr pcv = Data();
			pcv->points[nIndex].x = vert.x();
			pcv->points[nIndex].y = vert.y();
			pcv->points[nIndex].z = vert.z();
			pcv->points[nIndex].label = eUnclassified;

			pcv->bbox.expandBy(vert);
		}

		void CPointCloud::FillOsgPoint(const osg::Vec3& vert, uint32_t label, int nIndex)
		{
			PointCloudViewPtr pcv = Data();
			pcv->points[nIndex].x = vert.x();
			pcv->points[nIndex].y = vert.y();
			pcv->points[nIndex].z = vert.z();
			pcv->points[nIndex].label = label;

			pcv->bbox.expandBy(vert);
		}

	}
}