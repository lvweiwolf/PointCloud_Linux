//#include "stdafx.h"
#include "buildingExtraction.h"

#include <mutex>
#include <set>

#include "gridCell.h"

#include "../core/private/cloudProcess.h"
#include "../core/private/statistics.h"

#include "../algorithm/math.h"
#include "../io/rasterWriter.h"
#include "../../include/ClassificationDef.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

// #define _createHoleMask_OPENCV_DEBUG

extern std::string DebugDirectory;

namespace d3s {
	namespace pcs {

		namespace {

			void createHoleMask(const PointCloudView<PointPCLH>& pcv,
								const std::vector<int>& ground,
								const std::vector<int>& indices,
								const osg::BoundingBox& bbox,
								double resolution,
								double minArea,
								double maxArea,
								int kSize,
								Rasterd& result)
			{
				CHECK(!ground.empty());
				CHECK(resolution > 0.0);
				CHECK(bbox.valid());

				// 创建光栅数据
				double half = resolution / 2.0;
				double eps = resolution * .000001;

				double x = bbox.xMin() - half;
				double y = bbox.yMin() - half;
				int width = ((bbox.xMax() - x) / resolution) + 1;
				int height = ((bbox.yMax() - y) / resolution) + 1;

				cv::Mat gnd(height, width, CV_8UC1, cv::Scalar(0));
				cv::Mat vegetation(height, width, CV_8UC1, cv::Scalar(0));

				// ground
				for (size_t i = 0; i < ground.size(); ++i)
				{
					const auto& p = pcv[ground[i]];

					int xi = std::floor((p.x + half - eps - x) / resolution);
					int yi = std::floor((p.y + half - eps - y) / resolution);

					if (xi < 0 || xi >= width || yi < 0 || yi >= height)
						continue;

					gnd.at<uchar>(height - yi - 1, xi) = 255;
				}

				// indices;
				for (size_t i = 0; i < indices.size(); ++i)
				{
					const auto& p = pcv[indices[i]];

					int xi = std::floor((p.x + half - eps - x) / resolution);
					int yi = std::floor((p.y + half - eps - y) / resolution);

					if (xi < 0 || xi >= width || yi < 0 || yi >= height)
						continue;

					vegetation.at<uchar>(height - yi - 1, xi) = 255;
				}

				cv::Mat diff(vegetation - gnd);

#ifdef _createHoleMask_OPENCV_DEBUG
				cv::imwrite(DebugDirectory + StringPrintf("diff.jpg"), diff);
#endif

				cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kSize, kSize));
				cv::morphologyEx(diff, diff, cv::MORPH_OPEN, kernel);
				cv::morphologyEx(diff, diff, cv::MORPH_CLOSE, kernel);

#ifdef _createHoleMask_OPENCV_DEBUG
				cv::imwrite(DebugDirectory + StringPrintf("morph.jpg"), diff);
#endif

				cv::Mat binary(height, width, CV_8UC1, cv::Scalar(0));

				// 查找边界
				int numHoles = 0;
				double areaPerPixel = resolution * resolution;
				std::vector<std::vector<cv::Point>> contours;
				std::vector<cv::Vec4i> hierarchy;
				cv::findContours(diff, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

				//2024.1.13，wjf，去掉青苗特定优化
				//// 轮廓膨胀的核
				//cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

				//// 对每个轮廓进行膨胀
				//cv::Mat dst = cv::Mat::zeros(diff.size(), CV_8UC3);
				//cv::dilate(diff, dst, element);
				//for (size_t i = 0; i < contours.size(); i++)
				//{
				//	cv::drawContours(dst,
				//					 contours,
				//					 static_cast<int>(i),
				//					 cv::Scalar(255, 255, 255),
				//					 1);
				//}
				//// 获取膨胀后的轮廓
				//std::vector<std::vector<cv::Point>> contours_new;
				//std::vector<cv::Vec4i> hierarchy_new;
				//cv::findContours(dst,
				//				 contours_new,
				//				 hierarchy_new,
				//				 cv::RETR_LIST,
				//				 cv::CHAIN_APPROX_NONE);
				//contours = contours_new;
				////cv::imwrite(DebugDirectory + StringPrintf("dst.jpg"), dst);

				for (size_t i = 0; i < contours.size(); ++i)
				{
					std::vector<cv::Point> hull;
					cv::convexHull(contours[i], hull);
					// 计算轮廓大小
					double area = cv::contourArea(contours[i]) * areaPerPixel;
					double hullArea = cv::contourArea(hull) * areaPerPixel;
						
					// 2024.1.13，wjf，去掉青苗特定优化
					//if (area > minArea && area <= maxArea && area > 0.4 * hullArea)
					if (area > minArea && area <= maxArea && area > 0.7 * hullArea)
					{
						cv::fillPoly(binary, contours[i], cv::Scalar(255));
						++numHoles;
					}
				}

#ifdef _createHoleMask_OPENCV_DEBUG
				cv::imwrite(DebugDirectory + StringPrintf("binary.jpg"), binary);
#endif

				PCS_INFO("检测到 %d 个地面空洞区域.", numHoles);


				RasterExtents extents(x, y, width, height, resolution);
				result = Rasterd(extents, "vegetation", 0.0);

				// opencv -> Rasterd
				for (int r = 0; r < height; ++r)
				{
					for (int c = 0; c < width; ++c)
					{
						if (binary.at<unsigned char>(r, c) == 255)
							result.at(c, height - r - 1) = 1.0;
					}
				}
			}

			void findPlaneComponent(const PointCloudView<PointPCLH>& pcv,
									const std::vector<int>& indices,
									const Rasterd& mask,
									int kSize,
									cv::Mat& component)
			{
				static std::atomic<int> count (0);

				int width = mask.width();
				int height = mask.height();

				cv::Mat1b binary(cv::Size(width, height), 0);

				// Rasterd -> opencv
				for (int r = 0; r < height; ++r)
				{
					for (int c = 0; c < width; ++c)
					{
						double value0 = mask.at(c, height - r - 1) * 255.0;
						binary(r, c) = std::min(255, (int)value0);
					}
				}

				cv::Mat labels;
				cv::connectedComponents(binary, labels); // 连同区域

				std::set<int> labelSet;

				// indices;
				double half = mask.edgeLength() / 2.0;
				double eps = mask.edgeLength() * .000001;

				for (size_t i = 0; i < indices.size(); ++i)
				{
					const auto& p = pcv[indices[i]];

					int xi = mask.xCell(p.x + half - eps);
					int yi = mask.yCell(p.y + half - eps);

					if (xi < 0 || xi >= width || yi < 0 || yi >= height)
						continue;

					int label = labels.at<int>(height - yi - 1, xi);

					if (label != 0)
						labelSet.insert(label);
				}

				// 提取连同区域
				cv::Mat1b dst(cv::Size(width, height), 0);

				for (int r = 0; r < height; ++r)
				{
					for (int c = 0; c < width; ++c)
					{
						int label = labels.at<int>(r, c);

						if (labelSet.find(label) != labelSet.end())
							dst(r, c) = 255;
					}
				}

				cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kSize, kSize));
				cv::morphologyEx(dst, dst, cv::MORPH_DILATE, kernel);

				component = dst;
			}
		}

		// BuildingExtraction 建筑物提取类
		//////////////////////////////////////////////////////////////////////////
		BuildingExtraction::BuildingExtraction(const BuildingExtractionOptions& options,
											   PointCloudViewPtr input)
			: _options(options), _input(input)
		{
		}

		BuildingExtraction::~BuildingExtraction() {}

		void BuildingExtraction::Extract()
		{
			std::vector<int> all, ground, indices, candidate;

			double resolution = 0.8;
			double surface_resolution = 0.2;

			for (size_t i = 0; i < _input->size(); ++i)
			{
				const auto& p = _input->points[i];

				if (p.hag > _options.min_building_height)
				{
					if (p.label == eLowVegetation || p.label == eHighVegetation)
						indices.push_back(i);
				}
				else
				{
					if (p.label == eGround || p.label == eLowVegetation ||
						p.label == eHighVegetation)
					{
						ground.push_back(i);
					}
				}

				if (p.label == eLowVegetation || p.label == eHighVegetation)
					all.push_back(i);
			}

			// 计算边界范围
			osg::BoundingBox bbox;
			computeMinMax3D(*_input, ground, bbox);

			Rasterd mask;
			createHoleMask(*_input,
						   ground,
						   all,
						   bbox,
						   resolution,
						   _options.min_building_area,
						   _options.max_building_area,
						   _options.morph_kernel_size,
						   mask);


			// 在掩码内点云表面映射(每个单元记录z值最大的点云索引)
			double half2 = surface_resolution / 2.0;
			double eps2 = surface_resolution * .000001;

			RasterExtents surfExtents;
			surfExtents.resolution = surface_resolution; // 更高的分辨率
			surfExtents.x = bbox.xMin() - half2;
			surfExtents.y = bbox.yMin() - half2;
			surfExtents.width = ((bbox.xMax() - surfExtents.x) / surfExtents.resolution) + 1;
			surfExtents.height = ((bbox.yMax() - surfExtents.y) / surfExtents.resolution) + 1;

			Raster<int> surface(surfExtents, "surface raster", -1); // -1 表示空像素

			// 获取在地面孔洞范围内的点云
			double half = mask.edgeLength() / 2.0;
			double eps = mask.edgeLength() * .000001;

			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& p = _input->points[indices[i]];

				int xi = mask.xCell(p.x + half - eps);
				int yi = mask.yCell(p.y + half - eps);

				if ((xi < 0 || xi >= mask.width()) || (yi < 0 || yi >= mask.height()))
					continue;

				// 当前点云在某个掩码像素命中，获取之前在该像素的最大Z值点云索引
				if (mask.at(xi, yi) == 1.0)
				{
					// 计算点云在 surface 上的像素坐标
					int xj = surface.xCell(p.x + half2 - eps2);
					int yj = surface.yCell(p.y + half2 - eps2);
					xj = clamp(xj, 0, surface.width() - 1);
					yj = clamp(yj, 0, surface.height() - 1);

					int& pos = surface.at(xj, yj);

					if (pos == -1)
					{
						pos = indices[i];
					}
					else
					{
						// 计较z值，保留最高的那个
						const auto& q = _input->points[pos];

						if (p.z > q.z)
							pos = indices[i];
					}
				}
			}

			// 获取候选表面点云缩影
			for (int i = 0; i < surface.width(); ++i)
			{
				for (int j = 0; j < surface.height(); ++j)
				{
					if (surface.at(i, j) != -1)
						candidate.push_back(surface.at(i, j));
				}
			}

			PCS_INFO("[BuildingExtraction::Extract] 发现候选建筑点云数量 %d.", candidate.size());


			// 候选点云数量过少
			if (candidate.size() < _options.euclidean_cluster_min_pts)
			{

				PCS_INFO("[BuildingExtraction::Extract] 平面分割被跳过.");
				return;
			}

			
			// 欧式聚类
			std::vector<std::vector<int>> clusters;
			
			euclideanCluster(*_input,
							 candidate,
							 _options.euclidean_cluster_distance,
							 _options.euclidean_cluster_min_pts,
							 std::numeric_limits<int>::max(),
							 clusters);

			PCS_INFO("[BuildingExtraction::Extract] 候选建筑点云欧式聚类数量 %d.", clusters.size());
			

			double distanceThr = _options.planar_ransac_distance; // 0.1m
			double distanceThr2 = distanceThr * 2.0;

			std::mutex mut;
			std::vector<std::vector<int>> inliers;
			std::vector<std::vector<float>> coefficents;

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)clusters.size(); ++i)
			{
				const auto& cluster = clusters[i];
				int minPts = _options.planar_inlier_pts_ratio * cluster.size();
				// int minPts = 50;

				std::vector<std::vector<int>> cluster_inliners;
				std::vector<std::vector<float>> cluster_coeffs;

				// 平面分割
				planarSegment(*_input,
							  cluster,
							  minPts,
							  distanceThr,
							  cluster_inliners,
							  cluster_coeffs);

				if (!cluster_inliners.empty())
				{
					std::unique_lock<std::mutex> lock(mut);

					inliers.insert(inliers.end(), cluster_inliners.begin(), cluster_inliners.end());
					coefficents.insert(coefficents.end(),
									   cluster_coeffs.begin(),
									   cluster_coeffs.end());
				}
			}

			// 提过提取的平面点，在mask中找到关联的连通区域，并放大
			CHECK(inliers.size() == coefficents.size());
			int width = mask.width();
			int height = mask.height();

			// cv::Mat buildings(height, width, CV_8UC1, cv::Scalar(0));

			for (int i = 0; i < (int)inliers.size(); ++i)
			{
				const auto& inlierIndices = inliers[i];
				const auto& plane = coefficents[i];

				cv::Mat mat;
				findPlaneComponent(*_input, inlierIndices, mask, 6, mat);

#ifdef _OPENMP
#pragma omp parallel for
#endif
				for (int j = 0; j < (int)indices.size(); ++j)
				{
					auto& p = _input->points[indices[j]];

					int xi = mask.xCell(p.x + half - eps);
					int yi = mask.yCell(p.y + half - eps);

					if ((xi < 0 || xi >= mask.width()) || (yi < 0 || yi >= mask.height()))
						continue;

					if (mat.at<uchar>(height - yi - 1, xi) == 255)
					{
						Eigen::VectorXf coeff;
						coeff.resize(4);
						coeff[0] = plane[0];
						coeff[1] = plane[1];
						coeff[2] = plane[2];
						coeff[3] = plane[3];

						// 计算点到拟合平面内距离
						Eigen::Vector4f pt(p.x, p.y, p.z, 1.f);
						double distance = fabs(coeff.dot(pt));

						if (distance <= distanceThr2)
							p.label = eBuilding;
					}
				} // omp for
			}	// end for
		}
	}
}