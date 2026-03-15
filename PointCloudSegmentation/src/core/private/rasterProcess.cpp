//stdafx.h
#include "rasterProcess.h"

#include <numeric>
#include <set>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "../../io/rasterWriter.h"
#include "../../algorithm/delanuator.h"
#include "../../algorithm/mesh.h"
#include "../../algorithm/lsd.h"
#include "../pointTypes.h"

extern std::string DebugDirectory;

#define NODATA std::numeric_limits<double>::quiet_NaN()


namespace d3s {
	namespace pcs {

		void pointCloudMorph(const PointCloudView<PointPCLH>& pcv,
							 const std::vector<int>& indices,
							 double resolution,
							 int morphSize,
							 std::vector<int>& out)
		{
			Rasterd raster;
			createBinaryMask(pcv, indices, pcv.bbox, resolution, raster);

			int width = raster.width();
			int height = raster.height();

			cv::Mat binary(height, width, CV_8UC1, cv::Scalar(0));

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (raster.at(c, height - r - 1) == 1.0)
						binary.at<uchar>(r, c) = 255;
				}
			}

#ifdef _pointCloudMorph_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "cloud.jpg", binary);
#endif
			cv::Mat kernel =
				cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphSize, morphSize));

			cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
			cv::morphologyEx(binary, binary, cv::MORPH_DILATE, kernel);

#ifdef _pointCloudMorph_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "cloud_morph.jpg", binary);
#endif

			Rasterd mask(raster.extents(), "cloudMask", 0.0);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (binary.at<uchar>(r, c) == 255)
						mask.at(c, height - r - 1) = 1.0;
				}
			}

			out.reserve(indices.size());
			double half = resolution / 2.0;
			double eps = resolution * .000001;

			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& p = pcv.at(indices[i]);

				int xi = mask.xCell(p.x + half - eps);
				int yi = mask.yCell(p.y + half - eps);
				xi = clamp(xi, 0, (int)width - 1);
				yi = clamp(yi, 0, (int)height - 1);

				if (mask.at(xi, yi) == 1.0)
					out.push_back(indices[i]);
			}

			out.shrink_to_fit();
		}

		void createCloudRaster(const PointCloudView<PointPCLH>& pcv,
							   const std::vector<int>& ground,
							   double resolution,
							   Rasterd& result)
		{
			PointCloudView<PointPCLH>::PointCloud points;

			for (size_t i = 0; i < ground.size(); ++i)
			{
				const auto& p = pcv.points[ground[i]];
				points.push_back(p);
			}

			// 1px = 1m
			createCloudRaster(points, pcv.bbox, resolution, result);

		} // createCloudRaster

		void createCloudRaster(const PointCloudView<PointPCLH>::PointCloud& points,
							   const osg::BoundingBox& bbox,
							   double resolution,
							   Rasterd& result)
		{
			CHECK(!points.empty());

			// 三角剖分顶点
			std::vector<double> delaunayPoints(points.size() * 2);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)points.size(); ++i)
			{
				const auto& point = points[i];

				delaunayPoints[i * 2] = point.x;
				delaunayPoints[i * 2 + 1] = point.y;
			}

			std::vector<VerticesIndexes> mesh;

			try
			{
				// 对顶点进行2D Delaunay三角剖分
				Delaunator triangulation(delaunayPoints);

				int numTriangles = (int)triangulation.triangles.size() / 3;
				CHECK_MSG(numTriangles != 0, "numTriangles is 0.");
				mesh.resize(numTriangles);

#ifdef _OPENMP
#pragma omp parallel for
#endif
				for (int i = 0; i < numTriangles; ++i)
				{
					mesh[i] = VerticesIndexes(triangulation.triangles[i * 3 + 2],
											  triangulation.triangles[i * 3 + 1],
											  triangulation.triangles[i * 3]);
				}
			}
			catch (const delaunator_error& e)
			{
				PCS_ERROR(StringPrintf("创建 DTM 发生错误: %s", e.what()));
				return;
			}

			// 创建光栅数据
			double halfEdge = resolution / 2.0;
			double edgeBit = resolution * .000001;

			// 初始化数据范围
			RasterExtents extents;
			extents.resolution = resolution;
			extents.x = bbox.xMin() - halfEdge;
			extents.y = bbox.yMin() - halfEdge;
			extents.width = ((bbox.xMax() - extents.x) / extents.resolution) + 1;
			extents.height = ((bbox.yMax() - extents.y) / extents.resolution) + 1;

			PCS_INFO("[createCloudRaster] extents(%lf, %lf, %d, %d)",
					 extents.x,
					 extents.y,
					 extents.width,
					 extents.height);

			Rasterd raster(extents, "faceraster", NODATA);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)mesh.size(); ++i)
			{
				const VerticesIndexes& t = mesh[i];

				double x1 = points[t.i1].x;
				double y1 = points[t.i1].y;
				double z1 = points[t.i1].z;

				double x2 = points[t.i2].x;
				double y2 = points[t.i2].y;
				double z2 = points[t.i2].z;

				double x3 = points[t.i3].x;
				double y3 = points[t.i3].y;
				double z3 = points[t.i3].z;

				double xmax = (std::max)((std::max)(x1, x2), x3);
				double xmin = (std::min)((std::min)(x1, x2), x3);
				double ymax = (std::max)((std::max)(y1, y2), y3);
				double ymin = (std::min)((std::min)(y1, y2), y3);

				/*
				 * 因为我们在检查三角形所占单元中心，将其增加了1/2的边缘长度，
				 * 以避免在我们知道极限位置不能与单元中心相交的情况下测试细胞。
				 * 下界的 edgeBit 减法是为了考虑到最小位置与单元格中心精确对齐
				 * 的情况（我们可以简单地从左下方开始一个单元格，但在大多数情况
				 * 下，这种小调整消除了额外的行/列）。
				 */

				int ax = raster.xCell(xmin + halfEdge - edgeBit);
				int ay = raster.yCell(ymin + halfEdge - edgeBit);

				// 这里不需要调整edgeBit，因为我们要四舍五入获取精确值。
				int bx = raster.xCell(xmax + halfEdge);
				int by = raster.yCell(ymax + halfEdge);

				ax = clamp(ax, 0, (int)extents.width);
				bx = clamp(bx, 0, (int)extents.width);
				ay = clamp(ay, 0, (int)extents.height);
				by = clamp(by, 0, (int)extents.height);

				for (int xi = ax; xi < bx; ++xi)
				{
					for (int yi = ay; yi < by; ++yi)
					{
						double x = raster.xCellPos(xi);
						double y = raster.yCellPos(yi);

						double val =
							barycentricInterpolation(x1, y1, z1, x2, y2, z2, x3, y3, z3, x, y);
						if (val != std::numeric_limits<double>::infinity())
							raster.at(xi, yi) = val;
					}
				}
			}

			result = raster;

		} // createCloudRaster

		void createRoadRaster(const PointCloudView<PointPCLH>& pcv,
							  const std::vector<std::vector<osg::Vec3d>>& roads,
							  Rasterd& result)
		{
			const auto& bbox = pcv.bbox;

			// 创建光栅数据
			double resolution = 2.0;
			double halfEdge = resolution / 2.0;
			double edgeBit = resolution * .000001;

			// 初始化数据范围
			RasterExtents extents;
			extents.resolution = resolution;
			extents.x = bbox.xMin() - halfEdge;
			extents.y = bbox.yMin() - halfEdge;
			extents.width = ((bbox.xMax() - extents.x) / extents.resolution) + 1;
			extents.height = ((bbox.yMax() - extents.y) / extents.resolution) + 1;

			PCS_INFO("[createRoadRaster] extents(%lf, %lf, %d, %d)",
					 extents.x,
					 extents.y,
					 extents.width,
					 extents.height);

			Rasterd raster(extents, "roadraster", 0.0);
			int width = raster.width();
			int height = raster.height();
			int thickness = 15;

			cv::Mat1b binary(cv::Size(width, height), 0);

			for (int i = 0; i < roads.size(); ++i)
			{
				const auto& road = roads.at(i);

				for (size_t j = 0; j < road.size() - 1; ++j)
				{
					// 线段两点
					const auto& p = road.at(j);
					const auto& q = road.at(j + 1);

					int px = raster.xCell(p.x() + halfEdge - edgeBit);
					int py = raster.yCell(p.y() + halfEdge - edgeBit);
					int qx = raster.xCell(q.x() + halfEdge - edgeBit);
					int qy = raster.yCell(q.y() + halfEdge - edgeBit);

					px = clamp(px, 0, (int)width - 1);
					py = clamp(py, 0, (int)height - 1);
					qx = clamp(qx, 0, (int)width - 1);
					qy = clamp(qy, 0, (int)height - 1);

					cv::line(binary,
							 cv::Point(px, height - py - 1),
							 cv::Point(qx, height - qy - 1),
							 cv::Scalar(255),
							 thickness);
				}
			}

			// int kSize = 11;
			// cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kSize, kSize));
			// cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

#ifdef _road_vector_OPECV_DEBUG
			cv::imwrite(DebugDirectory + "road.jpg", binary);
#endif
			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (binary(r, c) == 255)
						raster.at(c, height - r - 1) = 1.0;
				}
			}

			result = raster;
		}

		void createBinaryMask(const PointCloudView<PointPCLH>& pcv,
							  const std::vector<int>& indices,
							  const osg::BoundingBox& bbox,
							  double resolution,
							  Rasterd& result)
		{
			CHECK(!indices.empty());
			CHECK(resolution > 0.0);
			CHECK(bbox.valid());

			// 创建光栅数据，分辨率 0.5m
			double halfEdge = resolution / 2.0;
			double edgeBit = resolution * .000001;

			// 初始化数据范围
			RasterExtents extents;
			extents.resolution = resolution;
			extents.x = bbox.xMin() - halfEdge;
			extents.y = bbox.yMin() - halfEdge;
			extents.width = ((bbox.xMax() - extents.x) / extents.resolution) + 1;
			extents.height = ((bbox.yMax() - extents.y) / extents.resolution) + 1;

			/*PCS_INFO("extents(%lf, %lf, %d, %d",
									  extents.x,
									  extents.y,
									  extents.width,
									  extents.height);*/

			Rasterd raster(extents, "binarymask", 0.0);


#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)indices.size(); ++i)
			{
				const auto& p = pcv[indices[i]];

				int xi = raster.xCell(p.x + halfEdge - edgeBit);
				int yi = raster.yCell(p.y + halfEdge - edgeBit);

				if (xi < 0 || xi >= raster.width() || yi < 0 || yi >= raster.height())
					continue;

				raster.at(xi, yi) = 1.0;
			}


			result = raster;
		}

		void createHagRaster(const PointCloudView<PointPCLH>& pcv,
							 const std::vector<int>& indices,
							 const osg::BoundingBox& bbox,
							 double resolution,
							 Rasterd& result)
		{
			CHECK(!indices.empty());
			CHECK(resolution > 0.0);
			CHECK(bbox.valid());

			// 创建光栅数据，分辨率 0.5m
			double halfEdge = resolution / 2.0;
			double edgeBit = resolution * .000001;

			// 初始化数据范围
			RasterExtents extents;
			extents.resolution = resolution;
			extents.x = bbox.xMin() - halfEdge;
			extents.y = bbox.yMin() - halfEdge;
			extents.width = ((bbox.xMax() - extents.x) / extents.resolution) + 1;
			extents.height = ((bbox.yMax() - extents.y) / extents.resolution) + 1;

			Rasterd raster(extents, "hagmask", NODATA);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)indices.size(); ++i)
			{
				const auto& p = pcv[indices[i]];

				int xi = raster.xCell(p.x + halfEdge - edgeBit);
				int yi = raster.yCell(p.y + halfEdge - edgeBit);

				if (xi < 0 || xi >= raster.width() || yi < 0 || yi >= raster.height())
					continue;

				raster.at(xi, yi) = std::max(raster.at(xi, yi), (double)p.hag);
			}

			result = raster;
		}

		void createHeightRaster(const PointCloudView<PointPCLH>& pcv,
								const std::vector<int>& indices,
								const osg::BoundingBox& bbox,
								double resolution,
								Rasterd& result)
		{
			CHECK(!indices.empty());
			CHECK(resolution > 0.0);
			CHECK(bbox.valid());

			// 创建光栅数据，分辨率 0.5m
			double halfEdge = resolution / 2.0;
			double edgeBit = resolution * .000001;

			// 初始化数据范围
			RasterExtents extents;
			extents.resolution = resolution;
			extents.x = bbox.xMin() - halfEdge;
			extents.y = bbox.yMin() - halfEdge;
			extents.width = ((bbox.xMax() - extents.x) / extents.resolution) + 1;
			extents.height = ((bbox.yMax() - extents.y) / extents.resolution) + 1;

			Rasterd raster(extents, "heightmask", NODATA);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)indices.size(); ++i)
			{
				const auto& p = pcv[indices[i]];

				int xi = raster.xCell(p.x + halfEdge - edgeBit);
				int yi = raster.yCell(p.y + halfEdge - edgeBit);

				if (xi < 0 || xi >= raster.width() || yi < 0 || yi >= raster.height())
					continue;

				raster.at(xi, yi) = std::max(raster.at(xi, yi), (double)p.z);
			}

			result = raster;
		}

		void writeTiff(const Rasterd& binary, const std::string& filepath, std::string srs)
		{
			RasterWriter writer(filepath);
			writer.setDriver("GTiff");
			writer.write(&binary, srs);
		}

		void writeImage(const Rasterd& src, const std::string& filepath, bool binary)
		{
			int width = src.width();
			int height = src.height();

			cv::Mat1f gray(cv::Size(width, height), 0.0f);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (binary)
					{
						if (src.at(c, height - r - 1) == 1.0)
							gray(r, c) = 255.0f;
					}
					else
						gray(r, c) = src.at(c, height - r - 1);
				}
			}

			cv::imwrite(filepath.c_str(), gray);
		}

		void writeImage(const Rasterd& src,
						const std::string& filepath,
						const std::vector<osg::Vec4>& colormap)
		{
			int width = src.width();
			int height = src.height();

			cv::Mat3b image(cv::Size(width, height), cv::Vec3b(0, 0, 0));

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					int treeId = (int)src.at(c, height - r - 1);

					if (treeId == 0)
						continue;

					const auto& color = colormap[treeId - 1];
					image(r, c) = cv::Vec3b(color.b() * 255, color.g() * 255, color.r() * 255);
				}
			}

			cv::imwrite(filepath.c_str(), image);
		}

		bool createPylonMask(const Rasterd& src,
							 const ImageProcessParameter& p,
							 Rasterd& result,
							 int& x,
							 int& y)
		{
			int width = src.width();
			int height = src.height();
			result = src;

			cv::Mat1b binary(cv::Size(width, height), 0);
			cv::Mat1b valid(cv::Size(width, height), 0);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (src.at(c, height - r - 1) == 1.0)
						binary(r, c) = 255;

					result.at(c, height - r - 1) = 0.0;
				}
			}


			int kfiil = p.morph_fill_size;
			int kopen = p.morph_open_size;
			cv::Mat kernel_fill = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kfiil, kfiil));
			cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kopen, kopen));

			// 闭运算
			cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel_fill);

#ifdef _createPylonMask_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "origin.jpg", binary);
#endif

			cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel_open);
			cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel_open);
#ifdef _createPylonMask_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "open_dilate.jpg", binary);
#endif
			// 寻找轮廓
			double areaMax = -DBL_MAX;
			int contourIndex = 0;
			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;
			cv::findContours(binary, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

			// 轮廓分析，找到组件
			for (size_t i = 0; i < contours.size(); ++i)
			{
				// 计算轮廓大小
				double area = cv::contourArea(contours[i]);

				if (area > areaMax)
				{
					areaMax = area;
					contourIndex = i;
				}
			}

			// 绘制轮廓
			cv::Mat dst(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
			cv::drawContours(dst,
							 contours,
							 contourIndex,
							 cv::Scalar(0, 0, 255),
							 2,
							 cv::LINE_8,
							 hierarchy,
							 0);


			bool bSuccess = false;

			// PCA 计算组件方向
			if (!contours.empty())
			{
				auto& pts = contours[contourIndex];

				std::vector<cv::Point> hull;
				cv::convexHull(pts, hull);

				// 填充，并膨胀
				cv::fillPoly(valid, hull, cv::Scalar(255));
				cv::morphologyEx(valid, valid, cv::MORPH_DILATE, kernel_open, cv::Point(-1, -1), 2);

				// 使用mat来保存轮廓坐标数据，也是为了后面pca处理需要
				cv::Mat data_pts = cv::Mat(pts.size(), 2, CV_64FC1);
				for (int i = 0; i < data_pts.rows; ++i)
				{
					data_pts.at<double>(i, 0) = pts[i].x;
					data_pts.at<double>(i, 1) = pts[i].y;
				}

				// 执行PCA分析
				cv::PCA pca_analysis(data_pts, cv::Mat(), 0);
				// 获得最主要分量（均值），对应的就是轮廓中点，也是图像中点
				cv::Point pos = cv::Point(pca_analysis.mean.at<double>(0, 0),
										  pca_analysis.mean.at<double>(0, 1));


				// 存储特征向量和特征值
				std::vector<cv::Point2d> eigen_vecs(2);
				std::vector<double> eigen_val(2);

				for (int i = 0; i < 2; ++i)
				{
					eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
												pca_analysis.eigenvectors.at<double>(i, 1));
					eigen_val[i] = pca_analysis.eigenvalues.at<double>(i, 0);
				}

				// 绘制出中心点
				circle(dst, pos, 3, CV_RGB(255, 0, 255), 2);
				x = pos.x;
				y = dst.rows - pos.y - 1;
				bSuccess = true;

				// 计算出直线，在主要方向上绘制直线（每个特征向量乘以其特征值并转换为平均位置。有一个
				// 0.02 的缩放系数，它只是为了确保矢量适合图像并且没有 10000 像素的长度）
				line(dst,
					 pos,
					 pos + 0.02 * cv::Point(eigen_vecs[0].x * eigen_val[0],
											eigen_vecs[0].y * eigen_val[0]),
					 CV_RGB(255, 255, 0));

				line(dst,
					 pos,
					 pos + 0.02 * cv::Point(eigen_vecs[1].x * eigen_val[1],
											eigen_vecs[1].y * eigen_val[1]),
					 CV_RGB(0, 255, 255));

#ifdef _createPylonMask_OPENCV_DEBUG
				cv::imwrite(DebugDirectory + "fill.jpg", valid);
				cv::imwrite(DebugDirectory + "contour.jpg", dst);
#endif
			}

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (valid(r, c) == 255)
						result.at(c, height - r - 1) = 1.0;
				}
			}

			return bSuccess;
		}

		void createPowerlineMask(const Rasterd& src,
								 const ImageProcessParameter& p,
								 Rasterd& result)
		{
			int width = src.width();
			int height = src.height();
			result = src;

			cv::Mat1b binary(cv::Size(width, height), 0);
			cv::Mat1b dst(cv::Size(width, height), 0);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (src.at(c, height - r - 1) == 1.0)
						binary(r, c) = 255;

					result.at(c, height - r - 1) = 0.0;
				}
			}

#ifdef _createPowerlineMask_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "pl_origin.jpg", binary);
#endif
			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;
			cv::findContours(binary, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

			// 轮廓分析，找到组件
			for (size_t i = 0; i < contours.size(); ++i)
			{
				// 计算轮廓大小
				cv::Rect bbox = cv::boundingRect(contours[i]);

				if (bbox.area() < 100)
					continue;

				cv::fillPoly(dst, contours[i], cv::Scalar(255));
			}

			/*int kfiil = p.morph_fill_size;
			cv::Mat kernel_fill = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kfiil, kfiil));
			cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, kernel_fill);*/

#ifdef _createPowerlineMask_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "pl_dst.jpg", dst);
#endif

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (dst(r, c) == 255)
						result.at(c, height - r - 1) = 1.0;
				}
			}
		}

		void createVegetationMask(const Rasterd& mask,
								  const ImageProcessParameter& p,
								  Rasterd& result,
								  double threshold /*= 100*/)
		{
			int width = mask.width();
			int height = mask.height();
			result = mask;

			cv::Mat1b binary(cv::Size(width, height), 0);
			cv::Mat1b valid(cv::Size(width, height), 0);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (mask.at(c, height - r - 1) == 0)
						binary(r, c) = 255;

					result.at(c, height - r - 1) = NODATA;
				}
			}

			int kfiil = p.morph_fill_size;
			int kopen = p.morph_open_size;
			cv::Mat kernel_fill = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kfiil, kfiil));
			cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kopen, kopen));

#ifdef _detectLines_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "dl_origin.jpg", binary);
#endif

			cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel_fill);

#ifdef _detectLines_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "dl_close.jpg", binary);
#endif

			cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel_open);

#ifdef _detectLines_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "dl_open.jpg", binary);
#endif

			// 寻找轮廓
			double areaMax = -DBL_MAX;
			std::vector<std::vector<cv::Point>> contours, filtered;
			std::vector<cv::Vec4i> hierarchy;
			cv::findContours(binary, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

			// 轮廓分析，找到组件
			for (size_t i = 0; i < contours.size(); ++i)
			{
				// 计算轮廓大小
				/*std::vector<cv::Point> hull;
				cv::convexHull(contours[i], hull);
				double area = cv::contourArea(hull);*/
				double area = cv::contourArea(contours[i]);

				if (area <= threshold)
					continue;

				filtered.push_back(contours[i]);
			}

			for (size_t i = 0; i < filtered.size(); ++i)
			{
				cv::fillPoly(valid, filtered[i], cv::Scalar(255));
			}

#ifdef _detectLines_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "dl_valid.jpg", valid);
#endif

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (valid(r, c) == 255)
						result.at(c, height - r - 1) = 0;
				}
			}
		}

		void createBodyMask(const PointCloudView<PointPCLH>& pcv,
							const std::vector<int>& mask,
							const osg::BoundingBox& bbox,
							double resolution,
							double morphRatio,
							int post,
							Rasterd& result)

		{
			CHECK(!mask.empty());
			CHECK(resolution > 0.0);
			CHECK(bbox.valid());

			// 创建光栅数据，分辨率 0.5m
			double halfEdge = resolution / 2.0;
			double edgeBit = resolution * .000001;

			// 初始化数据范围
			RasterExtents extents;
			extents.resolution = resolution;
			extents.x = bbox.xMin() - halfEdge;
			extents.y = bbox.yMin() - halfEdge;
			extents.width = ((bbox.xMax() - extents.x) / extents.resolution) + 1;
			extents.height = ((bbox.yMax() - extents.y) / extents.resolution) + 1;

			Rasterd raster(extents, "graymask", 0.0);

			for (int i = 0; i < (int)mask.size(); ++i)
			{
				const auto& p = pcv[mask[i]];

				int xi = raster.xCell(p.x + halfEdge - edgeBit);
				int yi = raster.yCell(p.y + halfEdge - edgeBit);

				if (xi < 0 || xi >= raster.width() || yi < 0 || yi >= raster.height())
					continue;

				raster.at(xi, yi) = 1.0;
			}

			int width = raster.width();
			int height = raster.height();

			cv::Mat1b gray(cv::Size(width, height), 0);
			cv::Mat1b dst(cv::Size(width, height), 0);

			// Rasterd -> opencv
			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					double value = raster.at(c, height - r - 1) * 255.0;
					gray(r, c) = std::min(255, (int)value);
				}
			}

			int kSize = std::max(3, (int)(std::max(width, height) * morphRatio));

			cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kSize, kSize));
			cv::morphologyEx(gray, dst, cv::MORPH_CLOSE, kernel);

#ifdef _createBodyMask_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + StringPrintf("close_%d.jpg", post), dst);
#endif

			result = Rasterd(extents, "binarymask", 0.0);

			// opencv -> Rasterd
			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (dst(r, c) == 255)
						result.at(c, height - r - 1) = 1.0;
				}
			}
		}

		void detectOutliers(const Rasterd& raster,
							double multi,
							Rasterd& result,
							std::vector<osg::Vec2i>& pixels)
		{
			int width = raster.width();
			int height = raster.height();

			std::vector<double> data;

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					double p = raster.at(c, r);

					if (p > 0.0)
						data.push_back(p);
				}
			}

			double sum = std::accumulate(data.begin(), data.end(), 0.0);
			double mean = sum / data.size(); // 均值
			double acc = 0.0;
			std::for_each(data.begin(), data.end(), [&](const double v) {
				acc += (v - mean) * (v - mean);
			});

			double stdev = sqrt(acc / (data.size() - 1)); // 标准差

			cv::Mat density(height, width, CV_32FC1, cv::Scalar(0.0));

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					double p = raster.at(c, height - r - 1);
					density.at<float>(r, c) = p;
				}
			}

#ifdef _detectOutliers_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "gray.jpg", density);
#endif

			cv::Mat binary(height, width, CV_8UC1, cv::Scalar(0));

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					float p = density.at<float>(r, c);

					if ((p - mean) > multi * stdev)
						binary.at<uchar>(r, c) = 255;
				}
			}


#ifdef _detectOutliers_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "binary.jpg", binary);
#endif

			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;
			cv::findContours(binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);


			for (size_t i = 0; i < contours.size(); ++i)
			{
				auto& pts = contours[i];

				// 使用mat来保存轮廓坐标数据，也是为了后面pca处理需要
				cv::Mat data_pts = cv::Mat(pts.size(), 2, CV_64FC1);
				for (int i = 0; i < data_pts.rows; ++i)
				{
					data_pts.at<double>(i, 0) = pts[i].x;
					data_pts.at<double>(i, 1) = pts[i].y;
				}

				// 执行PCA分析
				cv::PCA pca_analysis(data_pts, cv::Mat(), 0);
				// 获得最主要分量（均值），对应的就是轮廓中点，也是图像中点
				cv::Point pos = cv::Point(pca_analysis.mean.at<double>(0, 0),
										  pca_analysis.mean.at<double>(0, 1));

				pixels.push_back(osg::Vec2i(pos.x, height - pos.y - 1));
			}

			// opencv -> Rasterd
			Rasterd outliers(raster.extents(), "outliers", 0.0);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (binary.at<uchar>(r, c) == 255)
						outliers.at(c, height - r - 1) = 1.0;
				}
			}

			result = outliers;
		}


		void detectLines(const Rasterd& mask,
						 const ImageProcessParameter& p,
						 std::vector<std::array<osg::Vec2d, 2>>& lines)
		{
			int width = mask.width();
			int height = mask.height();

			cv::Mat1b binary(cv::Size(width, height), 0);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (mask.at(c, height - r - 1) == 1.0)
						binary(r, c) = 255;
				}
			}

#ifdef _detectLines_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "dl_origin.jpg", binary);
#endif

			cv::Mat dst(height, width, CV_8UC1, cv::Scalar(0));
			std::vector<cv::Vec4i> pxiel_lines; // 定义一个矢量结构lines用于存放得到的线段矢量集合

			cv::HoughLinesP(binary,
							pxiel_lines,
							p.hough_rho,
							p.hough_theta,
							p.hough_threshold,
							p.hough_min_length,
							p.hough_max_gap);

#ifdef _detectLines_OPENCV_DEBUG
			for (size_t i = 0; i < pxiel_lines.size(); i++)
			{
				const cv::Vec4i& l = pxiel_lines[i];
				cv::line(dst,
						 cv::Point(l[0], l[1]),
						 cv::Point(l[2], l[3]),
						 cv::Scalar(255),
						 p.line_thickness,
						 cv::LINE_8);
			}

			cv::imwrite(DebugDirectory + "dl_dst.jpg", dst);
#endif

			for (size_t i = 0; i < pxiel_lines.size(); ++i)
			{
				const cv::Vec4i& l = pxiel_lines[i];

				int x1 = l[0];
				int y1 = height - l[1] - 1;
				int x2 = l[2];
				int y2 = height - l[3] - 1;

				double dX1 = mask.xCellPos(x1);
				double dY1 = mask.yCellPos(y1);
				double dX2 = mask.xCellPos(x2);
				double dY2 = mask.yCellPos(y2);

				std::array<osg::Vec2d, 2> line = { osg::Vec2d(dX1, dY1), osg::Vec2d(dX2, dY2) };
				lines.push_back(line);
			}
		}

		void detectLines(const Rasterd& mask, const ImageProcessParameter& p, Rasterd& result)
		{
			int width = mask.width();
			int height = mask.height();

			cv::Mat1b binary(cv::Size(width, height), 0);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (mask.at(c, height - r - 1) == 1.0)
						binary(r, c) = 255;
				}
			}

#ifdef _detectLines_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "dl_origin.jpg", binary);
#endif

			cv::Mat dst(height, width, CV_8UC1, cv::Scalar(0));
			std::vector<cv::Vec4i> pxiel_lines; // 定义一个矢量结构lines用于存放得到的线段矢量集合

			cv::HoughLinesP(binary,
							pxiel_lines,
							p.hough_rho,
							p.hough_theta,
							p.hough_threshold,
							p.hough_min_length,
							p.hough_max_gap);


			for (size_t i = 0; i < pxiel_lines.size(); i++)
			{
				const cv::Vec4i& l = pxiel_lines[i];
				cv::line(dst,
						 cv::Point(l[0], l[1]),
						 cv::Point(l[2], l[3]),
						 cv::Scalar(255),
						 p.line_thickness,
						 cv::LINE_8);
			}

#ifdef _detectLines_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "dl_dst.jpg", dst);
#endif

			Rasterd lineMask(mask.extents(), "lineMask", 0.0);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (dst.at<uchar>(r, c) == 255)
						lineMask.at(c, height - r - 1) = 1.0;
				}
			}

			result = lineMask;
		}

		void detectLinesLSD(const Rasterd& mask,
							const ImageProcessParameter& p,
							std::vector<std::array<osg::Vec2d, 2>>& lines)
		{
			int width = mask.width();
			int height = mask.height();

			cv::Mat1b binary(cv::Size(width, height), 0);
			// 原始数据
			std::vector<double> raw_image(width * height, 0.0);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (mask.at(c, height - r - 1) == 1.0)
					{
						binary(r, c) = 255;
						raw_image[r * width + c] = 255.0;
					}
				}
			}

#ifdef _detectLines_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "dl_origin.jpg", binary);
#endif

			cv::Mat dst(height, width, CV_8UC1, cv::Scalar(0));
			std::vector<cv::Vec4f> pxiel_lines; // 定义一个矢量结构lines用于存放得到的线段矢量集合

			int num_segments = 0;
			double scale = p.line_lsd_scale;

			double* segments_data = LineSegmentDetection(&num_segments,
														 raw_image.data(),
														 width,
														 height,
														 scale, // Scale
														 0.6,	// Sigma scale
														 2,		// Quant
														 22.5,	// Angle tolerance
														 0,		// Log eps
														 0.7,	// Density threshold
														 1024,	// Num of bins
														 nullptr,
														 nullptr,
														 nullptr);

			for (int i = 0; i < num_segments; ++i)
			{
				cv::Vec4f line(segments_data[i * 7 + 0],
							   segments_data[i * 7 + 1],
							   segments_data[i * 7 + 2],
							   segments_data[i * 7 + 3]);

				pxiel_lines.push_back(line);
			}

			// 释放内存
			free((void*)segments_data);

#ifdef _detectLines_OPENCV_DEBUG
			for (size_t i = 0; i < pxiel_lines.size(); i++)
			{
				const cv::Vec4i& l = pxiel_lines[i];
				cv::line(dst,
						 cv::Point(l[0], l[1]),
						 cv::Point(l[2], l[3]),
						 cv::Scalar(255),
						 p.line_thickness,
						 cv::LINE_8);
			}

			cv::imwrite(DebugDirectory + "dl_dst.jpg", dst);
#endif

			for (size_t i = 0; i < pxiel_lines.size(); ++i)
			{
				const cv::Vec4i& l = pxiel_lines[i];

				int x1 = l[0];
				int y1 = height - l[1] - 1;
				int x2 = l[2];
				int y2 = height - l[3] - 1;

				double dX1 = mask.xCellPos(x1);
				double dY1 = mask.yCellPos(y1);
				double dX2 = mask.xCellPos(x2);
				double dY2 = mask.yCellPos(y2);

				std::array<osg::Vec2d, 2> line = { osg::Vec2d(dX1, dY1), osg::Vec2d(dX2, dY2) };
				lines.push_back(line);
			}
		}

		void removeSmallLengths(LineSegmnets& lines, double threshold)
		{
			for (auto iter = lines.begin(); iter != lines.end();)
			{
				auto& line = *iter;
				double len = (line[0] - line[1]).length();

				if (len < threshold)
					iter = lines.erase(iter);
				else
				{
					// 延长线段
					auto dir = line[1] - line[0];
					dir.normalize();

					line[0] = line[0] - dir * threshold * 0.5;
					line[1] = line[1] + dir * threshold * 0.5;

					++iter;
				}
			}
		}

		void createLineMask(const Rasterd& mask,
							const ImageProcessParameter& p,
							const std::vector<std::array<osg::Vec2d, 2>>& lines,
							Rasterd& result)
		{
			int width = mask.width();
			int height = mask.height();

			cv::Mat dst(height, width, CV_8UC1, cv::Scalar(0));

			for (size_t i = 0; i < lines.size(); ++i)
			{
				const std::array<osg::Vec2d, 2>& seg = lines[i];

				int x1 = clamp(mask.xCell(seg[0].x()), 0, width - 1);
				int y1 = clamp(mask.yCell(seg[0].y()), 0, height - 1);
				int x2 = clamp(mask.xCell(seg[1].x()), 0, width - 1);
				int y2 = clamp(mask.yCell(seg[1].y()), 0, height - 1);

				cv::line(dst,
						 cv::Point(x1, height - y1 - 1),
						 cv::Point(x2, height - y2 - 1),
						 cv::Scalar(255),
						 p.line_thickness,
						 cv::LINE_8);
			}

#ifdef _detectLines_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "dl_dst.jpg", dst);
#endif

			Rasterd lineMask(mask.extents(), "lineMask", 0.0);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (dst.at<uchar>(r, c) == 255)
						lineMask.at(c, height - r - 1) = 1.0;
				}
			}

			result = lineMask;
		}


		void findConnectComponents(const Rasterd& mask,
								   std::vector<std::vector<osg::Vec2i>>& components)
		{
			int width = mask.width();
			int height = mask.height();

			cv::Mat1b binary(cv::Size(width, height), 0);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (mask.at(c, height - r - 1) == 1.0)
						binary(r, c) = 255;
				}
			}

			cv::Mat label;
			int nLabels = cv::connectedComponents(binary, label);

#ifdef _findConnectComponents_OPENCV_DEBUG
			std::vector<cv::Vec3b> colors(nLabels);
			colors[0] = cv::Vec3b(0, 0, 0); // background

			for (int label = 1; label < nLabels; ++label)
			{
				colors[label] = cv::Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
			}

			cv::Mat dst(binary.size(), CV_8UC3);

			for (int r = 0; r < dst.rows; ++r)
			{
				for (int c = 0; c < dst.cols; ++c)
				{
					int l = label.at<int>(r, c);
					cv::Vec3b& pixel = dst.at<cv::Vec3b>(r, c);
					pixel = colors[l];
				}
			}

			cv::imwrite(DebugDirectory + "components.jpg", dst);
#endif
			components.clear();
			components.resize(nLabels - 1);

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					int l = label.at<int>(r, c);

					if (l == 0)
						continue;

					components[l - 1].push_back(osg::Vec2i(c, height - r - 1));
				}
			}
		}

		void intersectConnectComponents(const Rasterd& mask0, Rasterd& mask1, int post)
		{
			CHECK(mask0.size() == mask1.size());

			int width = mask0.width();
			int height = mask0.height();

			cv::Mat1b binary0(cv::Size(width, height), 0);
			cv::Mat1b binary1(cv::Size(width, height), 0);

			// Rasterd -> opencv
			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					double value0 = mask0.at(c, height - r - 1) * 255.0;
					binary0(r, c) = std::min(255, (int)value0);

					double value1 = mask1.at(c, height - r - 1) * 255.0;
					binary1(r, c) = std::min(255, (int)value1);
				}
			}

			// 查找连通区域
			cv::Mat label0, label1;
			cv::connectedComponents(binary0, label0);
			cv::connectedComponents(binary1, label1);

			std::set<int> labelSet;

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					int l0 = label0.at<int>(r, c);
					int l1 = label1.at<int>(r, c);

					if (l1 == 0)
						continue;

					if (labelSet.find(l1) != labelSet.end())
						continue;

					if (l0 != 0)
						labelSet.insert(l1);
				}
			}

			// 清除label1中无效的连通区域
			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					int l1 = label1.at<int>(r, c);

					if (labelSet.find(l1) == labelSet.end())
						binary1(r, c) = 0;
				}
			}

#ifdef _createBodyMask_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + StringPrintf("filter_%d.jpg", post), binary1);
#endif
			// opencv -> Rasterd
			mask1 = Rasterd(mask1.extents(), "binarymask", 0.0);

			// opencv -> Rasterd
			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (binary1(r, c) == 255)
						mask1.at(c, height - r - 1) = 1.0;
				}
			}
		}


		void dilate(Rasterd& mask, double morphRatio, int post)
		{
			int width = mask.width();
			int height = mask.height();

			cv::Mat1b binary(cv::Size(width, height), 0);

			// Rasterd -> opencv
			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					double value = mask.at(c, height - r - 1) * 255.0;
					binary(r, c) = std::min(255, (int)value);
				}
			}

			int kSize = std::max(3, (int)(std::max(width, height) * morphRatio));

			// 膨胀处理
			cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kSize, kSize));
			cv::morphologyEx(binary, binary, cv::MORPH_DILATE, kernel);

#ifdef _createBodyMask_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + StringPrintf("dilate_%d.jpg", post), binary);
#endif

			mask = Rasterd(mask.extents(), "binarymask", 0.0);

			// opencv -> Rasterd
			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (binary(r, c) == 255)
						mask.at(c, height - r - 1) = 1.0;
				}
			}
		}


		void convexHull(Rasterd& mask, double morphRatio, int post)
		{
			int width = mask.width();
			int height = mask.height();

			cv::Mat1b binary(cv::Size(width, height), 0);

			// Rasterd -> opencv
			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					double value = mask.at(c, height - r - 1) * 255.0;
					binary(r, c) = std::min(255, (int)value);
				}
			}

			std::vector<cv::Point> pts;

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (binary(r, c) == 255)
						pts.push_back(cv::Point(c, r));
				}
			}

			if (pts.empty())
				return;

			std::vector<cv::Point> hull;
			cv::convexHull(pts, hull);

			// 填充，并膨胀
			cv::fillPoly(binary, hull, cv::Scalar(255));

			int kSize = std::max(3, (int)(std::max(width, height) * morphRatio));

			// 膨胀处理
			cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kSize, kSize));
			cv::morphologyEx(binary, binary, cv::MORPH_DILATE, kernel);

#ifdef _createBodyMask_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + StringPrintf("convexHull_%d.jpg", post), binary);
#endif

			mask = Rasterd(mask.extents(), "binarymask", 0.0);

			// opencv -> Rasterd
			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (binary(r, c) == 255)
						mask.at(c, height - r - 1) = 1.0;
				}
			}
		}

		void scale(Rasterd& mask, double scale, int post)
		{
			int width = mask.width();
			int height = mask.height();

			cv::Mat1b binary(cv::Size(width, height), 0);
			cv::Mat1b binary_trans(cv::Size(width, height), 0);

			// Rasterd -> opencv
			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					double value = mask.at(c, height - r - 1) * 255.0;
					binary(r, c) = std::min(255, (int)value);
				}
			}

			// 放大
			cv::Point center = cv::Point(binary.cols / 2, binary.rows / 2);
			double angle = 0.0;
			cv::Mat transform(2, 3, CV_32FC1);
			transform = getRotationMatrix2D(center, angle, scale);

			cv::warpAffine(binary, binary_trans, transform, binary.size());


#ifdef _createBodyMask_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + StringPrintf("scale_%d.jpg", post), binary_trans);
#endif

			mask = Rasterd(mask.extents(), "binarymask", 0.0);

			// opencv -> Rasterd
			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (binary_trans(r, c) == 255)
						mask.at(c, height - r - 1) = 1.0;
				}
			}
		}

		/**
		 * @brief 对输入图像进行细化,骨骼化
		 * @param
		 * src为输入图像,用cvThreshold函数处理过的8位灰度图像格式，元素中只有0与1,1代表有元素，0代表为空白
		 * @param
		 * maxIterations限制迭代次数，如果不进行限制，默认为-1，代表不限制迭代次数，直到获得最终结果
		 * @return
		 * 为对src细化后的输出图像,格式与src格式相同，元素中只有0与1,1代表有元素，0代表为空白
		 */
		cv::Mat thinImage(const cv::Mat& src, const int maxIterations = -1)
		{
			assert(src.type() == CV_8UC1);

			int width = src.cols;
			int height = src.rows;

			cv::Mat dst;
			src.copyTo(dst);

			int count = 0; // 记录迭代次数

			while (true)
			{
				count++;

				if (maxIterations != -1 && count > maxIterations) // 限制次数并且迭代次数到达
					break;

				std::vector<uchar*> mFlag; // 用于标记需要删除的点

				// 对点标记
				for (int i = 0; i < height; ++i)
				{
					uchar* p = dst.ptr<uchar>(i);
					for (int j = 0; j < width; ++j)
					{
						// 如果满足四个条件，进行标记
						//   p9 p2 p3
						//   p8 p1 p4
						//   p7 p6 p5
						uchar p1 = p[j];
						if (p1 != 1)
							continue;

						uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
						uchar p8 = (j == 0) ? 0 : *(p + j - 1);
						uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
						uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
						uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
						uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
						uchar p5 =
							(i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
						uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);

						if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 &&
							(p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
						{
							int ap = 0;
							if (p2 == 0 && p3 == 1)
								++ap;
							if (p3 == 0 && p4 == 1)
								++ap;
							if (p4 == 0 && p5 == 1)
								++ap;
							if (p5 == 0 && p6 == 1)
								++ap;
							if (p6 == 0 && p7 == 1)
								++ap;
							if (p7 == 0 && p8 == 1)
								++ap;
							if (p8 == 0 && p9 == 1)
								++ap;
							if (p9 == 0 && p2 == 1)
								++ap;

							if (ap == 1 && p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0)
							{
								// 标记
								mFlag.push_back(p + j);
							}
						}
					}
				}

				// 将标记的点删除
				for (std::vector<uchar*>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
				{
					**i = 0;
				}

				// 直到没有点满足，算法结束
				if (mFlag.empty())
				{
					break;
				}
				else
				{
					mFlag.clear(); // 将mFlag清空
				}

				// 对点标记
				for (int i = 0; i < height; ++i)
				{
					uchar* p = dst.ptr<uchar>(i);
					for (int j = 0; j < width; ++j)
					{
						// 如果满足四个条件，进行标记
						//   p9 p2 p3
						//   p8 p1 p4
						//   p7 p6 p5
						uchar p1 = p[j];
						if (p1 != 1)
							continue;
						uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
						uchar p8 = (j == 0) ? 0 : *(p + j - 1);
						uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
						uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
						uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
						uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
						uchar p5 =
							(i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
						uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);

						if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 &&
							(p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
						{
							int ap = 0;
							if (p2 == 0 && p3 == 1)
								++ap;
							if (p3 == 0 && p4 == 1)
								++ap;
							if (p4 == 0 && p5 == 1)
								++ap;
							if (p5 == 0 && p6 == 1)
								++ap;
							if (p6 == 0 && p7 == 1)
								++ap;
							if (p7 == 0 && p8 == 1)
								++ap;
							if (p8 == 0 && p9 == 1)
								++ap;
							if (p9 == 0 && p2 == 1)
								++ap;

							if (ap == 1 && p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0)
							{
								// 标记
								mFlag.push_back(p + j);
							}
						}
					}
				}

				// 将标记的点删除
				for (std::vector<uchar*>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
				{
					**i = 0;
				}

				// 直到没有点满足，算法结束
				if (mFlag.empty())
				{
					break;
				}
				else
				{
					mFlag.clear(); // 将mFlag清空
				}
			}

			return dst;
		}

		/**
		 * @brief 对骨骼化图数据进行过滤，实现两个点之间至少隔一个空白像素
		 * @param thinSrc为输入的骨骼化图像,8位灰度图像格式，元素中只有0与1,1代表有元素，0代表为空白
		 */
		void filterOver(cv::Mat thinSrc)
		{
			assert(thinSrc.type() == CV_8UC1);

			int width = thinSrc.cols;
			int height = thinSrc.rows;

			for (int i = 0; i < height; ++i)
			{
				uchar* p = thinSrc.ptr<uchar>(i);
				for (int j = 0; j < width; ++j)
				{
					// 实现两个点之间至少隔一个像素
					//  p9 p2 p3
					//  p8 p1 p4
					//  p7 p6 p5
					uchar p1 = p[j];

					if (p1 != 1)
						continue;

					uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
					uchar p8 = (j == 0) ? 0 : *(p + j - 1);
					uchar p2 = (i == 0) ? 0 : *(p - thinSrc.step + j);
					uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - thinSrc.step + j + 1);
					uchar p9 = (i == 0 || j == 0) ? 0 : *(p - thinSrc.step + j - 1);
					uchar p6 = (i == height - 1) ? 0 : *(p + thinSrc.step + j);
					uchar p5 =
						(i == height - 1 || j == width - 1) ? 0 : *(p + thinSrc.step + j + 1);
					uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + thinSrc.step + j - 1);

					if (p2 + p3 + p8 + p9 >= 1)
					{
						p[j] = 0;
					}
				}
			}
		}

		/**
		 * @brief 从过滤后的骨骼化图像中寻找端点和交叉点
		 * @param
		 * thinSrc为输入的过滤后骨骼化图像,8位灰度图像格式，元素中只有0与1,1代表有元素，0代表为空白
		 * @param raudis卷积半径，以当前像素点位圆心，在圆范围内判断点是否为端点或交叉点
		 * @param thresholdMax交叉点阈值，大于这个值为交叉点
		 * @param thresholdMin端点阈值，小于这个值为端点
		 * @return
		 * 为对src细化后的输出图像,格式与src格式相同，元素中只有0与1,1代表有元素，0代表为空白
		 */
		std::vector<cv::Point> getPoints(const cv::Mat& thinSrc,
										 unsigned int raudis = 4,
										 unsigned int thresholdMax = 6,
										 unsigned int thresholdMin = 4)
		{
			assert(thinSrc.type() == CV_8UC1);

			int width = thinSrc.cols;
			int height = thinSrc.rows;

			cv::Mat tmp;
			thinSrc.copyTo(tmp);
			std::vector<cv::Point> points;

			for (int i = 0; i < height; ++i)
			{
				for (int j = 0; j < width; ++j)
				{
					if (*(tmp.data + tmp.step * i + j) == 0)
					{
						continue;
					}
					int count = 0;
					for (int k = i - raudis; k < i + raudis + 1; k++)
					{
						for (int l = j - raudis; l < j + raudis + 1; l++)
						{
							if (k < 0 || l < 0 || k > height - 1 || l > width - 1)
							{
								continue;
							}
							else if (*(tmp.data + tmp.step * k + l) == 1)
							{
								count++;
							}
						}
					}

					if (count > thresholdMax || count < thresholdMin)
					{
						cv::Point point(j, i);
						points.push_back(point);
					}
				}
			}

			return points;
		}

		void thinning(Rasterd& mask) 
		{
			int width = mask.width();
			int height = mask.height();

			cv::Mat1b binary(cv::Size(width, height), 0);

			// Rasterd -> opencv
			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					double value = mask.at(c, height - r - 1) * 255.0;
					binary(r, c) = std::min(255, (int)value);
				}
			}

			cv::imwrite(DebugDirectory + "binary.jpg", binary);

			// 细化提取骨骼
			cv::threshold(binary, binary, 127, 1, cv::THRESH_BINARY);
			binary = thinImage(binary);

			// 过滤细化图像
			filterOver(binary);
			/*
			std::vector<cv::Point> points = getPoints(binary, 6, 9, 6);

			binary *= 255;

			vector<cv::Point>::iterator it = points.begin();
			for (; it != points.end(); it++)
			{
				circle(binary, *it, 4, 255, 1);
			}*/

			binary *= 255;
			cv::imwrite(DebugDirectory + "thin.jpg", binary);


			// opencv -> Rasterd
			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (binary(r, c) == 255)
						mask.at(c, height - r - 1) = 1.0;
				}
			}
		}

		double compuateShapeness(const Rasterd& mask)
		{
			int width = mask.width();
			int height = mask.height();

			cv::Mat1b binary(cv::Size(width, height), 0);

			// Rasterd -> opencv
			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					if (mask.at(c, height - r - 1) == 1.0)
						binary(r, c) = 255;
				}
			}

			cv::Mat kernel_closed = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
			cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel_closed);

#ifdef _shapeness_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + "cluster_mask.jpg", binary);
#endif

			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;
			cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

			CHECK(contours.size() == 1);

#ifdef _shapeness_OPENCV_DEBUG
			cv::Mat dst(cv::Size(width, height), CV_8UC3, cv::Scalar(0, 0, 0));
			cv::drawContours(dst, contours, 0, cv::Scalar(0, 0, 255), 1, cv::LINE_8);
			cv::imwrite(DebugDirectory + "cluster_contour.jpg", dst);
#endif

			double shapeness = DBL_MAX;

			for (size_t i = 0; i < contours.size(); ++i)
			{
				// 计算轮廓大小
				double area = cv::contourArea(contours[i]);
				double length = cv::arcLength(contours[i], true);

				shapeness = std::min(shapeness, sqrt(area) / length);
			}

			return shapeness;
		}

		std::vector<osg::Vec4> createColors(int colorSize)
		{
			std::vector<osg::Vec4> colorMap(colorSize);
			
			cv::RNG rng(time(0));
			for (int i = 0; i < colorSize; ++i)
			{
				colorMap[i].r() = rng.uniform(0.0f, 1.0f);
				colorMap[i].g() = rng.uniform(0.0f, 1.0f);
				colorMap[i].r() = rng.uniform(0.0f, 1.0f);
			}

			return colorMap;
		}

		std::vector<osg::Vec4> createPseudoColors(int numClasses) 
		{
			std::vector<osg::Vec4> colorMap(numClasses);
			int j = 0;
			int lab = 0;

			for (int i = 0; i < numClasses; ++i)
			{
				j = 0;
				lab = i;

				while (lab)
				{
					colorMap[i][0] = (double)(((lab >> 0) & 1) << (7 - j)) / 255.f;
					colorMap[i][1] = (double)(((lab >> 1) & 1) << (7 - j)) / 255.f;
					colorMap[i][2] = (double)(((lab >> 2) & 1) << (7 - j)) / 255.f;
					colorMap[i][3] = 1.0f;
					j++;
					lab >>= 3;
				}
			}

			return colorMap;
		}

		void gaussianBlur(Rasterd& raster, int kx, int ky, int sigmaX)
		{
			int width = raster.width();
			int height = raster.height();
			double resolution = raster.edgeLength();

			// Rasterd -> opencv
			cv::Mat1d mat(cv::Size(width, height), 0.0);

			for (int y = 0; y < height; ++y)
			{
				for (int x = 0; x < width; ++x)
				{
					auto value = raster.at(x, height - y - 1);
					mat(y, x) = value;
				}
			}

			cv::GaussianBlur(mat, mat, cv::Size(kx, ky), sigmaX);

			// opencv -> Rasterd
			for (int y = 0; y < height; ++y)
			{
				for (int x = 0; x < width; ++x)
				{	
					raster.at(x, height - y - 1) = mat(y, x);
				}
			}
		}

		Rasterd computeSlope(const Rasterd& raster)
		{
			int width = raster.width();
			int height = raster.height();
			double resolution = raster.edgeLength();

			// 转化成opencv格式
			cv::Mat1d dem(cv::Size(width, height), 0.0);

			for (int y = 0; y < height; ++y)
				for (int x = 0; x < width; ++x)
					dem(y, x) = raster.at(x, height - y - 1);

			cv::Mat1d slope(cv::Size(width, height), 0.0);

			for (int y = 0; y < height; ++y)
			{
				for (int x = 0; x < width; ++x)
				{
					int l = x - 1;
					int r = x + 1;
					int t = y - 1;
					int b = y + 1;

					double e1 = 0.0;
					double e2 = 0.0;
					double e3 = 0.0;
					double e4 = 0.0;
					double e5 = 0.0;
					double e6 = 0.0;
					double e7 = 0.0;
					double e8 = 0.0;

					double e = dem(y, x);

					if (std::isnan(e))
						continue;

					if (l >= 0 && l < width)
						e1 = dem(y, l);

					if (r >= 0 && r < width)
						e3 = dem(y, r);

					if (t >= 0 && t < height)
						e2 = dem(t, x);

					if (b >= 0 && b < height)
						e4 = dem(b, x);

					if (l >= 0 && l < width && t >= 0 && t < height)
						e5 = dem(t, l);

					if (r >= 0 && r < width && t >= 0 && t < height)
						e6 = dem(t, r);

					if (r >= 0 && r < width && b >= 0 && b < height)
						e7 = dem(b, r);

					if (l >= 0 && l < width && b >= 0 && b < height)
						e8 = dem(b, l);

					/*double dx = ((e8 + 2.0 * e1 + e5) - (e7 + 2.0 * e3 + e6)) / (8.0 *
					resolution); double dy = ((e7 + 2.0 * e4 + e8) - (e6 + 2.0 * e2 + e5)) / (8.0 *
					resolution); double key = sqrt(dx * dx + dy * dy); double slopeDegree =
					osg::RadiansToDegrees(atan(key));*/


					double dx = (e1 - e3) / (2.0 * resolution);
					double dy = (e4 - e2) / (2.0 * resolution);
					double key = sqrt(dx * dx + dy * dy);
					double slopeDegree = osg::RadiansToDegrees(atan(key));
					slope(y, x) = slopeDegree;
				}
			}


			Rasterd out(raster.extents(), NODATA);

			// opencv -> Rasterd
			for (int y = 0; y < height; ++y)
				for (int x = 0; x < width; ++x)
					out.at(x, height - y - 1) = slope(y, x);

			return out;
		}

		Rasterd computeGrad(const Rasterd& raster)
		{
			int width = raster.width();
			int height = raster.height();
			double resolution = raster.edgeLength();

			// 转化成opencv格式
			cv::Mat1d src(cv::Size(width, height), 0.0);

			for (int y = 0; y < height; ++y)
				for (int x = 0; x < width; ++x)
					src(y, x) = raster.at(x, height - y - 1);
			

			cv::Mat grad_x, grad_y, dst;
			cv::Sobel(src, grad_x, CV_64FC1, 1, 0);
			cv::Sobel(src, grad_y, CV_64FC1, 0, 1);
			cv::Mat1d gradXY(cv::Size(width, height), 0.0);

			double grad_min = DBL_MAX;
			double grad_max = -DBL_MAX;

			for (int y = 0; y < height; ++y)
			{
				for (int x = 0; x < width; ++x)
				{
					double dx = grad_x.at<double>(y, x);
					double dy = grad_y.at<double>(y, x);
					double grad = std::sqrt(dx * dx + dy * dy);

					grad_min = std::min(grad_min, grad);
					grad_max = std::max(grad_max, grad);

					gradXY(y, x) = (grad - grad_min) / (grad_max - grad_min);
				}
			}

			// cv::imwrite(DebugDirectory + "gradXY.jpg", gradXY * 255);

			Rasterd out(raster.extents(), NODATA);

			// opencv -> Rasterd
			for (int y = 0; y < height; ++y)
				for (int x = 0; x < width; ++x)
					out.at(x, height - y - 1) = gradXY(y, x);

			return out;
		}

		Rasterd computeLaplacian(const Rasterd& raster) 
		{
			int width = raster.width();
			int height = raster.height();
			double resolution = raster.edgeLength();

			// Rasterd -> opencv
			cv::Mat1d src(cv::Size(width, height), 0.0);

			for (int y = 0; y < height; ++y)
				for (int x = 0; x < width; ++x)
					src(y, x) = raster.at(x, height - y - 1);
	

			cv::Mat1d laplacian;
			cv::Laplacian(src, laplacian, CV_64F);
			// cv::imwrite(DebugDirectory + "laplacian.jpg", laplacian * 255);

			Rasterd out(raster.extents(), NODATA);

			// opencv -> Rasterd
			for (int y = 0; y < height; ++y)
				for (int x = 0; x < width; ++x)
					out.at(x, height - y - 1) = laplacian(y, x);

			return out;
		}

		void morphology(Rasterd& raster, int morphType, int morphShape, int xsize, int ysize) 
		{
			int width = raster.width();
			int height = raster.height();
			double resolution = raster.edgeLength();

			// 转化成opencv格式
			cv::Mat1b binary(cv::Size(width, height), 0);

			for (int y = 0; y < height; ++y)
			{
				for (int x = 0; x < width; ++x)
				{
					auto value = raster.at(x, height - y - 1);
					if (value == 1.0)
						binary(y, x) = 255;
				}
			}

			cv::Mat kernel = cv::getStructuringElement(morphShape, cv::Size(xsize, ysize));
			cv::morphologyEx(binary, binary, morphType, kernel);

			Rasterd out(raster.extents(), 0.0);

			// opencv -> Rasterd
			for (int y = 0; y < height; ++y)
			{
				for (int x = 0; x < width; ++x)
				{
					if (binary(y, x) > 127)
						out.at(x, height - y - 1) = 1.0;
				}
			}

			raster = out;
		}


		Rasterd filterWithArea(const Rasterd& raster, double areaThr) 
		{
			int width = raster.width();
			int height = raster.height();
			double resolution = raster.edgeLength();

			// 转化成opencv格式
			cv::Mat1b binary(cv::Size(width, height), 0);

			for (int y = 0; y < height; ++y)
			{
				for (int x = 0; x < width; ++x)
				{
					auto value = raster.at(x, height - y - 1);
					if (value == 1.0)
						binary(y, x) = 255;
				}
			}

			/*cv::Mat label;
			int nLabels = cv::connectedComponents(binary, label);
			std::vector<int> pixelCounts(nLabels, 0);*/

			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;
			cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

			for (size_t i = 0; i < contours.size(); ++i)
			{
				// 计算轮廓大小
				double area = cv::contourArea(contours[i]) ;
				// double length = cv::arcLength(contours[i], true);

				if ((area* resolution * resolution) < areaThr)
					cv::fillPoly(binary, contours[i], cv::Scalar(0));
				else
				{
					std::vector<cv::Point> hull;
					cv::convexHull(contours[i], hull);
					cv::fillPoly(binary, hull, cv::Scalar(255));
				}
			}

			Rasterd out(raster.extents(), 0);

			// opencv -> Rasterd
			for (int y = 0; y < height; ++y)
			{
				for (int x = 0; x < width; ++x)
				{
					if (binary(y, x) > 127)
						out.at(x, height - y - 1) = 1.0;
				}
			}

			return out;
		}

		

	}
}