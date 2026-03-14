//stdafx.h
#include "cloudProcess.h"
//#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
//#include <pcl/io/impl/pcd_io.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/segmentation/impl/extract_clusters.hpp>
//#include <pcl/search/impl/kdtree.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>
#include <osg/Vec3d>
#include "statistics.h"

#include "../../algorithm/registration.h"
#include "../../algorithm/math.h"

#include "../../core/pointTypes.h"
#include "../../utils/logging.h"
// #define _normalPlanarSegment_PCD_DEBUG

extern std::string DebugDirectory;

namespace d3s
{
	namespace pcs
	{

		void euclideanCluster(PointCloudView<PointPCLH>& pcv,
							  const std::vector<int>& indices,
							  double tolerance,
							  unsigned int minPts,
							  unsigned int maxPts,
							  std::vector<std::vector<int>>& clusters)
		{
			if (indices.empty())
			{
				PCS_WARN("[euclideanCluster] 聚类输入点集为空.");
				return;
			}

			pcl::IndicesPtr indicesPtr(new std::vector<int>());
			*indicesPtr = indices;

			pcl::PointCloud<PointPCLH>::Ptr cloud(new pcl::PointCloud<PointPCLH>());
			std::swap(cloud->points, pcv.points); // pcv -> cloud
			cloud->width = cloud->points.size();
			cloud->height = 1;


			pcl::search::Search<PointPCLH>::Ptr kdtree(new pcl::search::KdTree<PointPCLH>);
			kdtree->setInputCloud(cloud, indicesPtr);

			std::vector<pcl::PointIndices> cluster_indices;
			pcl::extractEuclideanClusters(*cloud,
										  *indicesPtr,
										  kdtree,
										  static_cast<float>(tolerance),
										  cluster_indices,
										  minPts,
										  maxPts);

			std::swap(cloud->points, pcv.points); // cloud -> pcv

			clusters.clear();
			for (size_t i = 0; i < cluster_indices.size(); ++i)
				clusters.push_back(cluster_indices[i].indices);
		}

		void euclideanClusterSafe(const PointCloudView<PointPCLH>& pcv,
								  const std::vector<int>& indices,
								  double tolerance,
								  unsigned int minPts,
								  unsigned int maxPts,
								  std::vector<std::vector<int>>& clusters)
		{
			if (indices.empty())
			{
				PCS_WARN("[euclideanClusterSafe] 聚类输入点集为空.");
				return;
			}

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
			cloud->points.reserve(indices.size()); // 预先分配内存

			// 初始化点云
			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& p = pcv.points[indices[i]];
				cloud->points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
			}

			cloud->width = cloud->points.size();
			cloud->height = 1;

			// KDTree
			pcl::search::Search<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
			kdtree->setInputCloud(cloud);

			std::vector<pcl::PointIndices> cluster_result;
			pcl::extractEuclideanClusters(*cloud,
										  kdtree,
										  static_cast<float>(tolerance),
										  cluster_result,
										  minPts,
										  maxPts);

			// 转换点云索引
			for (size_t i = 0; i < cluster_result.size(); ++i)
			{
				std::vector<int> cluster;

				const std::vector<int>& cluster_indices = cluster_result.at(i).indices;

				for (size_t j = 0; j < cluster_indices.size(); ++j)
					cluster.push_back(indices[cluster_indices[j]]);

				if (!cluster.empty())
					clusters.push_back(cluster);
			}
		}

		void computeCloudFeatures(const PointCloudView<PointPCLH>& pcv,
								  const std::vector<int>& indices,
								  double radius,
								  std::string featureType,
								  std::vector<float>& features)
		{
			if (indices.empty())
			{
				PCS_WARN("[computeCloudFeatures] 聚类输入点集为空.");
				return;
			}

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
			cloud->points.reserve(indices.size()); // 预先分配内存
			
			// 初始化点云
			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& p = pcv.points[indices[i]];
				cloud->points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
			}

			cloud->width = cloud->points.size();
			cloud->height = 1;

			// KDTree
			pcl::search::Search<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
			kdtree->setInputCloud(cloud);

			features.resize(indices.size(), 0.0f);	// 初始化特征值

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)cloud->size(); ++i)
			{
				std::vector<int> nn_indices;
				std::vector<float> nn_distances;

				if (kdtree->radiusSearch(cloud->points[i], radius, nn_indices, nn_distances) < 1)
					continue;

				std::vector<osg::Vec3d> neighbors;
				neighbors.reserve(nn_indices.size());

				for (size_t j = 0; j < nn_indices.size(); ++j)
				{
					const auto& p = cloud->points[nn_indices[j]];
					neighbors.push_back(osg::Vec3d(p.x, p.y, p.z));
				}

				float& feature_value = features[i];

				if (featureType == "linearity")
				{
					double lambda1 = 0.0;
					double lambda2 = 0.0;
					double lambda3 = 0.0;
					
					getEigenValues(neighbors, lambda1, lambda2, lambda3); // λ1 > λ2 > λ3

					if (lambda1 != 0.0)
						feature_value = (lambda1 - lambda2) / lambda1;
				}
			}
		}

		void radiusSearch(const std::vector<osg::Vec3d>& points,
						  double radius,
						  std::vector<std::vector<int>>& neigborsList,
						  std::vector<std::vector<float>>& distancesList)
		{
			if (points.empty())
			{
				PCS_WARN("[computeFeaturesSafe] 半径邻域搜索输入点集为空.");
				return;
			}

			neigborsList.clear();
			distancesList.clear();
			neigborsList.resize(points.size());
			distancesList.resize(points.size());

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
			cloud->points.reserve(points.size()); // 预先分配内存

			// 初始化点云
			for (size_t i = 0; i < points.size(); ++i)
			{
				const auto& p = points[i];
				cloud->points.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
			}

			cloud->width = cloud->points.size();
			cloud->height = 1;

			// KDTree
			pcl::search::Search<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
			kdtree->setInputCloud(cloud);

			// 邻近点索引
#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)cloud->size(); ++i)
			{
				std::vector<int> nn_indices;
				std::vector<float> nn_distances;

				if (kdtree->radiusSearch(cloud->points[i], radius, nn_indices, nn_distances) < 1)
				{
					continue;
				}

				neigborsList[i] = nn_indices;
				distancesList[i] = nn_distances;
			}
		}

		void sliceVerticalLayers(const PointCloudView<PointPCLH>& pcv,
								 const std::vector<int>& indices,
								 double step,
								 std::vector<std::vector<int>>& layers)
		{
			osg::BoundingBox bbox;
			computeMinMax3D(pcv, indices, bbox);

			int numLayers = std::floor((bbox.zMax() - bbox.zMin()) / step) + 1;
			layers.resize(numLayers);

			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& p = pcv.points[indices[i]];

				int zi = std::floor((p.z - bbox.zMin()) / step);
				zi = clamp(zi, 0, numLayers - 1);
				layers[zi].push_back(indices[i]);
			}
		}

		void planarSegment(PointCloudView<PointPCLH>& pcv,
						   const std::vector<int>& indices,
						   int minPts,
						   double distanceThreshold,
						   std::vector<std::vector<int>>& inliers,
						   std::vector<std::vector<float>>& coefficens)
		{
			typedef pcl::PointXYZ PointT;

			PCS_INFO("[normalPlanarSegment] 提取平面, (%d/%d) 平面最少点数/总点数.",
					 minPts,
					 indices.size());

			inliers.clear();
			coefficens.clear();

			// 约束参数
			int kMinClusterSize = minPts;
			double kDistanceThreshold = distanceThreshold;

			// 数据
			pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
			pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);

			// 初始化
			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& p = pcv[indices[i]];
				cloud->push_back(PointT(p.x, p.y, p.z));
			}

			cloud->width = cloud->points.size();
			cloud->height = 1;

			// 结果
			pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);


			// 开始提取平面
			static int nc = 0;
			int i = 0, nr_points = (int)cloud->points.size();
			int plane_found = 0;
			int reg_id = 0;
			int csize = 0;
			int k = 0;
			std::vector<int> size_of_clusters;

			pcl::PCDWriter writer;
			pcl::SACSegmentation<PointT> seg;
			pcl::ExtractIndices<PointT> extract;

			while (cloud->points.size() > kMinClusterSize)
			{
				seg.setOptimizeCoefficients(false);
				seg.setModelType(pcl::SACMODEL_PLANE);
				seg.setMethodType(pcl::SAC_RANSAC);
				seg.setMaxIterations(1000);
				seg.setDistanceThreshold(kDistanceThreshold);
				seg.setInputCloud(cloud);

				// 平面包含的点索引和平面系数
				seg.segment(*inliers_plane, *coefficients_plane);

				if (inliers_plane->indices.size() == 0)
				{
					PCS_WARN("无法估计给定数据集的平面模型.");
					break;
				}

				// 从原始点云中提取平面点
				extract.setInputCloud(cloud);
				extract.setIndices(inliers_plane);
				extract.setNegative(false);

				pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
				extract.filter(*cloud_plane);

				bool haveMinSize = true;
				csize = (int)(inliers_plane->indices.size());

				if (csize < kMinClusterSize)
					haveMinSize = false;

				if (haveMinSize)
				{
#ifdef _normalPlanarSegment_PCD_DEBUG
					std::string cluster_filename = StringPrintf("plane_cluster%d.pcd", nc++);
					writer.write<pcl::PointXYZ>(DebugDirectory + cluster_filename,
												*cloud_plane,
												false);

#endif
					plane_found++;
					i++;
					size_of_clusters.push_back((int)cloud_plane->size());


					// 点云添加到聚类列表
					std::vector<int> seg;

					for (size_t i = 0; i < inliers_plane->indices.size(); ++i)
					{
						const auto& plane = inliers_plane->indices;
						seg.push_back(indices[plane[i]]);
					}

					if (!seg.empty())
					{
						inliers.push_back(seg);
						coefficens.push_back(coefficients_plane->values);
					}
				}

				// 从原始点云中移除平面点云，提取剩余部分
				extract.setNegative(true);
				extract.filter(*cloud2);

				cloud.swap(cloud2);
			}
		}

		void getEigenVectors(const std::vector<osg::Vec3d>& pts,
							 osg::Vec3d& major,
							 osg::Vec3d& middle,
							 osg::Vec3d& minor)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

			for (size_t i = 0; i < pts.size(); ++i)
			{
				cloud->points.push_back(pcl::PointXYZ(pts[i].x(), pts[i].y(), pts[i].z()));
			}

			cloud->width = cloud->points.size();
			cloud->height = 1;

			Eigen::Vector3f major_vector, middle_vector, minor_vector;

			/*pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
			feature_extractor.setInputCloud(cloud);
			feature_extractor.compute();
			feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);*/

			{
				Eigen::Vector4f centroid;
				Eigen::Matrix3f covariance;

				pcl::compute3DCentroid(*cloud, centroid);
				pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);

				Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance,
																	  Eigen::ComputeEigenvectors);

				Eigen::Matrix3f eigen_vectors = solver.eigenvectors();
				Eigen::Vector3f eigen_values = solver.eigenvalues();

				Eigen::Vector3f major_axis;
				Eigen::Vector3f middle_axis;
				Eigen::Vector3f minor_axis;
				float major_value;
				float middle_value;
				float minor_value;

				{
					unsigned int temp = 0;
					unsigned int major_index = 0;
					unsigned int middle_index = 1;
					unsigned int minor_index = 2;

					if (eigen_values.real()(major_index) < eigen_values.real()(middle_index))
					{
						temp = major_index;
						major_index = middle_index;
						middle_index = temp;
					}

					if (eigen_values.real()(major_index) < eigen_values.real()(minor_index))
					{
						temp = major_index;
						major_index = minor_index;
						minor_index = temp;
					}

					if (eigen_values.real()(middle_index) < eigen_values.real()(minor_index))
					{
						temp = minor_index;
						minor_index = middle_index;
						middle_index = temp;
					}

					major_value = eigen_values.real()(major_index);
					middle_value = eigen_values.real()(middle_index);
					minor_value = eigen_values.real()(minor_index);

					major_axis = eigen_vectors.col(major_index).real();
					middle_axis = eigen_vectors.col(middle_index).real();
					minor_axis = eigen_vectors.col(minor_index).real();

					major_axis.normalize();
					middle_axis.normalize();
					minor_axis.normalize();

					float det = major_axis.dot(middle_axis.cross(minor_axis));
					if (det <= 0.0f)
					{
						major_axis(0) = -major_axis(0);
						major_axis(1) = -major_axis(1);
						major_axis(2) = -major_axis(2);
					}
				}

				major_vector = major_axis;
				middle_vector = middle_axis;
				minor_vector = minor_axis;
			}

			major = osg::Vec3d(major_vector[0], major_vector[1], major_vector[2]);
			middle = osg::Vec3d(middle_vector[0], middle_vector[1], middle_vector[2]);
			minor = osg::Vec3d(minor_vector[0], minor_vector[1], minor_vector[2]);
		}

		void getEigenValues(const std::vector<osg::Vec3d>& pts,
							double& majorValue,
							double& middleValue,
							double& minorValue)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

			for (size_t i = 0; i < pts.size(); ++i)
			{
				cloud->points.push_back(pcl::PointXYZ(pts[i].x(), pts[i].y(), pts[i].z()));
			}

			cloud->width = cloud->points.size();
			cloud->height = 1;

			Eigen::Vector4f centroid;
			Eigen::Matrix3f covariance;

			pcl::compute3DCentroid(*cloud, centroid);
			pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);

			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance,
																  Eigen::ComputeEigenvectors);

			Eigen::Vector3f eigen_values = solver.eigenvalues();


			{
				unsigned int temp = 0;
				unsigned int major_index = 0;
				unsigned int middle_index = 1;
				unsigned int minor_index = 2;

				if (eigen_values.real()(major_index) < eigen_values.real()(middle_index))
				{
					temp = major_index;
					major_index = middle_index;
					middle_index = temp;
				}

				if (eigen_values.real()(major_index) < eigen_values.real()(minor_index))
				{
					temp = major_index;
					major_index = minor_index;
					minor_index = temp;
				}

				if (eigen_values.real()(middle_index) < eigen_values.real()(minor_index))
				{
					temp = minor_index;
					minor_index = middle_index;
					middle_index = temp;
				}

				majorValue = eigen_values.real()(major_index);
				middleValue = eigen_values.real()(middle_index);
				minorValue = eigen_values.real()(minor_index);
			}
		}

		bool RegistrationPCL(const PointCloudView<PointPCLH>& input,
							 const PointCloudView<PointPCLH>& target,
							 osg::Matrix& matrix,
							 double& rms)
		{
			typedef pcl::PointXYZ PointT;
			
			bool covergence = false;
			double fitscore = DBL_MAX;
			
			pcl::PointCloud<PointT>::Ptr cloudRegistered(new pcl::PointCloud<PointT>());

			{
				pcl::PointCloud<PointT>::Ptr cloudInput(new pcl::PointCloud<PointT>());
				{
					auto offset = input.offset_xyz;
					cloudInput->width = input.points.size();
					cloudInput->height = 1;
					cloudInput->points.reserve(input.points.size());


					for (size_t i = 0; i < input.points.size(); ++i)
					{
						const auto& p = input.points[i];
						cloudInput->push_back(
							PointT(p.x + offset.x(), p.y + offset.y(), p.z + offset.z()));
					}
				}

				pcl::PointCloud<PointT>::Ptr cloudTarget(new pcl::PointCloud<PointT>());
				{
					auto offset = target.offset_xyz;
					cloudTarget->width = target.points.size();
					cloudTarget->height = 1;
					cloudTarget->points.reserve(target.points.size());

					for (size_t i = 0; i < target.points.size(); ++i)
					{
						const auto& p = target.points[i];
						cloudTarget->push_back(
							PointT(p.x + offset.x(), p.y + offset.y(), p.z + offset.z()));
					}
				}

				pcl::IterativeClosestPoint<PointT, PointT> icp;

				icp.setMaximumIterations(100);
				icp.setMaxCorrespondenceDistance(3.0);
				icp.setRANSACOutlierRejectionThreshold(0.6);
				icp.setInputSource(cloudInput);
				icp.setInputTarget(cloudTarget);

				icp.align(*cloudRegistered);

				covergence = icp.hasConverged();
				fitscore = icp.getFitnessScore();
				auto transformation_matrix = icp.getFinalTransformation();

				osg::Matrixf matrixF;
				Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::ColMajor>>(matrixF.ptr()) =
					transformation_matrix;


				matrix = matrixF;
			}

			
			double* data = matrix.ptr();
			rms = ComputeRMS(input, target, matrix); // 计算Hausetoff距离均方误差

			/*PCS_INFO("[RegistrationICP] fitness_score: %lf", fitscore);
			PCS_INFO("[RegistrationICP] RMS: %lf", rms);
			PCS_INFO("[RegistrationICP] Transform matrix:"
					 "\n				%lf, %lf, %lf, %lf"
					 "\n				%lf, %lf, %lf, %lf"
					 "\n				%lf, %lf, %lf, %lf"
					 "\n				%lf, %lf, %lf, %lf",
					 data[0],
					 data[1],
					 data[2],
					 data[3],
					 data[4],
					 data[5],
					 data[6],
					 data[7],
					 data[8],
					 data[9],
					 data[10],
					 data[11],
					 data[12],
					 data[13],
					 data[14],
					 data[15]);*/

			if (!covergence)
			{
				PCS_WARN("[RegistrationICP] ICP配准未收敛!");
			}
			
			return covergence;
		}

		bool Registration(const PointCloudView<PointPCLH>& input,
						  const PointCloudView<PointPCLH>& target,
						  osg::Matrix& matrix,
						  double& rms)
		{
			bool covergence = false;
			double covergence_mse = DBL_MAX;
			Transform transformation_matrix;

			{
				Vertices vertices_input, normal_input;
				Vertices vertices_target, normal_target;

				vertices_input.resize(3, input.size());
				vertices_target.resize(3, target.size());

				for (size_t i = 0; i < input.points.size(); ++i)
				{
					const auto& offset = input.offset_xyz;
					const auto& p = input.points[i];

					vertices_input(0, i) = p.x + offset.x();
					vertices_input(1, i) = p.y + offset.y();
					vertices_input(2, i) = p.z + offset.z();
				}

				for (size_t i = 0; i < target.points.size(); ++i)
				{
					const auto& offset = target.offset_xyz;
					const auto& p = target.points[i];

					vertices_target(0, i) = p.x + offset.x();
					vertices_target(1, i) = p.y + offset.y();
					vertices_target(2, i) = p.z + offset.z();
				}


				covergence = registration(vertices_input,
										  vertices_target,
										  normal_input,
										  normal_target,
										  ICPMethod::RICP,
										  transformation_matrix,
										  covergence_mse);
			}
			

			Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(matrix.ptr()) =
				transformation_matrix;

			const double* data = matrix.ptr();
			rms = ComputeRMS(input, target, matrix);	// 计算Hausetoff距离均方误差

			/*PCS_INFO("[Registration] covergence_mse: %lf", covergence_mse);
			PCS_INFO("[Registration] RMS: %lf", rms);
			PCS_INFO("[Registration] Transform matrix:"
					 "\n			 %lf, %lf, %lf, %lf"
					 "\n			 %lf, %lf, %lf, %lf"
					 "\n			 %lf, %lf, %lf, %lf"
					 "\n			 %lf, %lf, %lf, %lf",
					 data[0],
					 data[1],
					 data[2],
					 data[3],
					 data[4],
					 data[5],
					 data[6],
					 data[7],
					 data[8],
					 data[9],
					 data[10],
					 data[11],
					 data[12],
					 data[13],
					 data[14],
					 data[15]);*/

			return covergence;
		}

		double ComputeRMS(const PointCloudView<PointPCLH>& input,
						  const PointCloudView<PointPCLH>& target,
						  const osg::Matrix& matrix)
		{
			typedef pcl::PointXYZ PointT;

			double max_range = std::numeric_limits<double>::max();

			pcl::PointCloud<PointT>::Ptr cloudInput(new pcl::PointCloud<PointT>());
			{
				auto offset = input.offset_xyz;
				cloudInput->width = input.points.size();
				cloudInput->height = 1;
				cloudInput->points.reserve(input.points.size());

				for (size_t i = 0; i < input.points.size(); ++i)
				{
					const auto& p = input.points[i];
					cloudInput->push_back(
						PointT(p.x + offset.x(), p.y + offset.y(), p.z + offset.z()));
				}
			}

			pcl::PointCloud<PointT>::Ptr cloudTarget(new pcl::PointCloud<PointT>());
			{
				auto offset = target.offset_xyz;
				cloudTarget->width = target.points.size();
				cloudTarget->height = 1;
				cloudTarget->points.reserve(target.points.size());

				for (size_t i = 0; i < target.points.size(); ++i)
				{
					const auto& p = target.points[i];
					cloudTarget->push_back(
						PointT(p.x + offset.x(), p.y + offset.y(), p.z + offset.z()));
				}
			}

			Transform transform =
				Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(matrix.ptr());

			// 转换输入点坐标
			pcl::PointCloud<PointT>::Ptr cloudTransformed(new pcl::PointCloud<PointT>());
			pcl::transformPointCloud(*cloudInput, *cloudTransformed, transform);

			// KDTree查找
			pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
			tree->setInputCloud(cloudTarget);


			std::atomic<int> nr(0);
			std::atomic<double> fitness_score (0.0);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)cloudTransformed->points.size(); ++i)
			{
				std::vector<int> nn_indices;
				std::vector<float> nn_dists;

				// 查找 Input 矩阵变换后在 Target 中的邻近点
				tree->nearestKSearch(cloudTransformed->points[i], 1, nn_indices, nn_dists);

				if (nn_dists[0] <= max_range)
				{
					fitness_score = fitness_score + (double)nn_dists[0];
					nr++;
				}
			}

			if (nr > 0)
				return (fitness_score / nr);
			else
				return (std::numeric_limits<double>::max());
		}

	}
}
