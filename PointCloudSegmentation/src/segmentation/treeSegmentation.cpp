#include <src/segmentation/treeSegmentation.h>
#include <src/segmentation/gridCell.h>

#include <src/core/private/statistics.h>
#include <src/core/api.h>
#include <src/core/private/rasterProcess.h>
#include <src/plot/plotHandle.h>

#include <pcl/search/kdtree.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

extern std::string DebugDirectory;

#define NA_INTEGER (-INT_MAX)
#define EPSILON 2e-8
#define XYINF 99999999999
#define ZINF 2147483640

namespace d3s {
	namespace pcs {
		namespace {

			template <typename T1, typename T2, typename T3>
			struct Point2D
			{
				T1 x;
				T2 y;
				T3 id;

				Point2D() {}
				Point2D(const T1 _x, const T2 _y) : x(_x), y(_y), id(0) {}
				Point2D(const T1 _x, const T2 _y, const T3 _id) : x(_x), y(_y), id(_id) {}
			};

			template <typename T1, typename T2, typename T3, typename T4>
			struct Point3D
			{
				T1 x;
				T2 y;
				T3 z;
				T4 id;

				Point3D() {}
				Point3D(const T1 _x, const T2 _y) : x(_x), y(_y), z(0), id(0) {}
				Point3D(const T1 _x, const T2 _y, const T3 _z) : x(_x), y(_y), z(_z), id(0) {}
				Point3D(const T1 _x, const T2 _y, const T3 _z, const T4 _id)
					: x(_x), y(_y), z(_z), id(_id)
				{
				}
			};

			typedef Point2D<double, double, unsigned int> Point;
			typedef Point3D<int, int, int, int> Pixeli;
			typedef Point3D<int, int, double, int> Pixeld;

			
			struct Shape
			{
				double xmin;
				double xmax;
				double ymin;
				double ymax;
				double zmin;
				double zmax;

				Shape();
				Shape(double xmin_, double xmax_, double ymin_, double ymax_);
				Shape(double xmin_,
					  double xmax_,
					  double ymin_,
					  double ymax_,
					  double zmin_,
					  double zmax_);

				template <typename T>
				bool contains(const T&);
			};

			Shape::Shape()
				: xmin(-XYINF), xmax(XYINF), ymin(-XYINF), ymax(XYINF), zmin(-ZINF), zmax(ZINF)
			{
			}

			Shape::Shape(double xmin_, double xmax_, double ymin_, double ymax_)
				: xmin(xmin_), xmax(xmax_), ymin(ymin_), ymax(ymax_), zmin(-ZINF), zmax(ZINF)
			{
			}

			Shape::Shape(double xmin_,
						 double xmax_,
						 double ymin_,
						 double ymax_,
						 double zmin_,
						 double zmax_)
				: xmin(xmin_), xmax(xmax_), ymin(ymin_), ymax(ymax_), zmin(zmin_), zmax(zmax_)
			{
			}
			
			template <typename T>
			bool Shape::contains(const T& p)
			{
				return (p.x >= xmin - EPSILON && p.x <= xmax + EPSILON && p.y >= ymin - EPSILON &&
						p.y <= ymax + EPSILON && p.z >= zmin - EPSILON && p.z <= zmax + EPSILON);
			}

			struct Rectangle : public Shape
			{
				Rectangle(double xmin_, double xmax_, double ymin_, double ymax_);

				template <typename T>
				bool contains(const T&);
			};

			Rectangle::Rectangle(double xmin_, double xmax_, double ymin_, double ymax_) : Shape(xmin_, xmax_, ymin_, ymax_) {}


			template <typename T>
			bool Rectangle::contains(const T& p)
			{
				return (p.x >= xmin - EPSILON && p.x <= xmax + EPSILON && p.y >= ymin - EPSILON &&
						p.y <= ymax + EPSILON);
			}

			struct Circle : public Shape
			{
				Circle(double xcenter, double ycenter, double radius);

				template <typename T>
				bool contains(const T&);

				Point center;
				double radius;
			};

			Circle::Circle(double xcenter, double ycenter, double radius_)
				: Shape(xcenter - radius_, xcenter + radius_, ycenter - radius_, ycenter + radius_),
				  radius(radius_)
			{
				center.x = xcenter;
				center.y = ycenter;
			}

			template <typename T>
			bool Circle::contains(const T& p)
			{
				double A = center.x - p.x;
				double B = center.y - p.y;
				double d = A * A + B * B;
				return d <= radius * radius + EPSILON;
			}
			
			template<typename T>
			void GridLookup(const Grid2D& grid, T& shape, std::vector<int>& res)
			{
				double xmin = shape.xmin;
				double xmax = shape.xmax;
				double ymin = shape.ymin;
				double ymax = shape.ymax;
				double zmin = shape.zmin;
				double zmax = shape.zmax;

				osg::BoundingBox shape_bbox(xmin, ymin, zmin, xmax, ymax, zmax);

				res.clear();

				for (size_t i = 0; i < grid.size(); ++i)
				{
					const auto& cell = grid.at(i);

					if (cell.GetSize() <= 0)
						continue;

					if (cell.intersects(shape_bbox))
					{
						const auto& cloud = cell.GetInput();
						const auto& cellIndices = cell.GetIndices();

						for (size_t j = 0; j < cellIndices.size(); ++j)
						{
							const auto& p = cloud->points[cellIndices[j]];

							if (shape.contains(p))
								res.push_back(cellIndices[j]);
						}
					}
				}
			}
		}

		// TreeSegmentationOptions
		//////////////////////////////////////////////////////////////////////////
		void TreeSegmentationOptions::Print()
		{
			PCS_INFO("green_color_style: %s", green_color_style ? "true" : "false");
			PCS_INFO("min_pts: %d", min_pts);
			PCS_INFO("min_height: %lf", min_height);
			PCS_INFO("max_height: %lf", max_height);
			PCS_INFO("min_radius: %lf", min_radius);
			PCS_INFO("max_radius: %lf", max_radius);
			PCS_INFO("th_tree: %lf", th_tree);
			PCS_INFO("th_seed: %lf", th_seed);
			PCS_INFO("th_crown: %lf", th_crown);
			PCS_INFO("max_crown: %lf", max_crown);
			PCS_INFO("resolution: %lf", resolution);

			PCS_INFO("dt1: %lf", dt1);
			PCS_INFO("dt2: %lf", dt2);
			PCS_INFO("Zu: %lf", Zu);
			PCS_INFO("R: %lf", R);
			PCS_INFO("radius: %lf", radius);
		}

		// TreeSegmentation 单木分割
		//////////////////////////////////////////////////////////////////////////
		Dalponte2016::Dalponte2016(const TreeSegmentationOptions& options,
								   PointCloudViewPtr input,
								   const std::vector<int>& indices)
			: _options(options), _input(input), _indices(indices)
		{
		}

		Dalponte2016::~Dalponte2016() {}

		std::vector<TreeMetrics> Dalponte2016::GetTreeMetrics() const { return _treeMetrics; }

		void Dalponte2016::Run()
		{
			if (!_input->normalized)
				return;

			if (_indices.empty())
				return;

			Timer timer;
			timer.Start();

			double resolution = _options.resolution;
			double r1 = _options.min_radius;
			double r2 = _options.max_radius;
			double h1 = _options.min_height;
			double h2 = _options.max_height;

			Rasterd mask;
			createBinaryMask(*_input, _indices, _input->bbox, resolution, mask);

			// 获得表面点
			double half = resolution / 2.0;
			double eps = resolution * .000001;
			int width = mask.width();
			int height = mask.height();

			cv::Mat mat(height, width, CV_32SC1, cv::Scalar(-1));
			cv::Mat chm(height, width, CV_32FC1, cv::Scalar(0.0f));

			for (size_t i = 0; i < _indices.size(); ++i)
			{
				const auto& p = _input->points[_indices[i]];

				int xi = mask.xCell(p.x + half - eps);
				int yi = mask.yCell(p.y + half - eps);
				int r = clamp(height - yi - 1, 0, height);
				int c = clamp(xi, 0, width);


				if (mat.at<int>(r, c) == -1)
				{
					mat.at<int>(r, c) = _indices[i];
					chm.at<float>(r, c) = p.z;
				}
				else
				{
					const auto& q = _input->points[mat.at<int>(r, c)];

					if (p.z > q.z)
					{
						mat.at<int>(r, c) = _indices[i];
						chm.at<float>(r, c) = p.z;
					}
				}
			}

			PCS_INFO("[TreeSegmentation] 单木分割，植被掩码尺寸 %d x %d.", width, height);

			// 优化: 动态局部最大值
			auto dynamic_radius = [&](double h) {

				if (h < h1)
					return r1;

				if (h > h2)
					return r2;

				double a = (r2 - r1) / (-(exp(-0.08 * (h2 - h1)) - 1));
					
				// return (2.6 * (-(exp(-0.08 * (h - 2)) - 1)) + 1);
				return (a * (-(exp(-0.08 * (h - h1)) - 1)) + r1);

			};

			/*auto dynamic_radius = [](double h) { return (2.51503 + 0.00901 * h * h); };*/

			// 计算局部最大值
			cv::Mat treetops, labels;
			ComputeLocalMaxima(mat, treetops, dynamic_radius);

			// 分割树木
			SegmentTree(chm,
						treetops,
						labels,
						_options.th_seed,
						_options.th_crown,
						_options.th_tree,
						_options.max_crown);

			cv::RNG rng(time(0));

#ifdef _treeSegment_OPENCV_DEBUG
			std::map<int, cv::Vec3b> colorMap;
			cv::Mat dst(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

			for (int r = 0; r < height; ++r)
			{
				for (int c = 0; c < width; ++c)
				{
					int treeId = labels.at<int>(r, c);

					if (treeId == 0)
						continue;

					auto it = colorMap.find(treeId);

					if (it == colorMap.end())
					{
						uchar b = rng.uniform(0, 255);
						uchar g = rng.uniform(0, 255);
						uchar r = rng.uniform(0, 255);

						colorMap[treeId] = cv::Vec3b(b, g, r);
					}

					dst.at<cv::Vec3b>(r, c) = colorMap[treeId];
				}
			}

			cv::imwrite(DebugDirectory + StringPrintf("segment.jpg"), dst);
#endif
			std::map<int, std::vector<int>> treesCandidate, trees;

			std::vector<int> vegetation;
			getClassification(*_input, eLowVegetation, vegetation);
			vegetation.insert(vegetation.end(), _indices.begin(), _indices.end());

			ColorManager::treeSegments.resize(_input->size(), -1);
			ColorManager::treeColorMap.clear();

			for (size_t i = 0; i < vegetation.size(); ++i)
			{
				const auto& p = _input->points[vegetation[i]];

				int xi = mask.xCell(p.x + half - eps);
				int yi = mask.yCell(p.y + half - eps);
				xi = clamp(xi, 0, mask.width() - 1);
				yi = clamp(yi, 0, mask.height() - 1);
				int r = height - yi - 1;
				int c = xi;

				// 检索所属数据分类
				int id = labels.at<int>(r, c);

				if (id == 0)
					continue;

				// 统计单颗树木的点云
				treesCandidate[id].push_back(vegetation[i]);
			}

			int treeId = 1;
			std::vector<TreeMetrics> newTreeMetrics;

			for (auto iter = treesCandidate.begin(); iter != treesCandidate.end(); ++iter)
			{
				int oldId = iter->first;
				const auto& indices = iter->second;
				auto& metric = _treeMetrics[oldId - 1];

				if (indices.size() < _options.min_pts)
					continue;

				// 过滤最小高度
				double dTreeHeight = 0.0;
				for (size_t i = 0; i < indices.size(); ++i)
				{
					auto& p = _input->points[indices[i]];
					dTreeHeight = p.hag > dTreeHeight ? p.hag : dTreeHeight;
				}
				if (dTreeHeight < _options.min_height)
					continue;

				trees[treeId] = indices;
				metric.id = treeId;
				newTreeMetrics.push_back(metric);

				// 为有效树木分配颜色
				auto it = ColorManager::treeColorMap.find(treeId);

				if (it == ColorManager::treeColorMap.end())
				{
					float r, g, b;

					if (_options.green_color_style)
					{
						b = rng.uniform(0.1f, 0.3f);
						g = rng.uniform(0.3f, 0.8f);
						r = rng.uniform(0.1f, 0.3f);
					}
					else
					{
						b = rng.uniform(0.0f, 1.0f);
						g = rng.uniform(0.0f, 1.0f);
						r = rng.uniform(0.0f, 1.0f);
					}

					ColorManager::treeColorMap[treeId] = osg::Vec4(r, g, b, 1.0f);
				}

				// 设置点云颜色
				for (size_t i = 0; i < indices.size(); ++i)
				{
					auto& p = _input->points[indices[i]];
					p.data[3] = treeId; // 设置点所属树木 Id

					ColorManager::treeSegments[indices[i]] = treeId;
				}

				++treeId;
			}

			treesCandidate.clear();

			_treeMetrics = newTreeMetrics;
			_treeMetrics.shrink_to_fit();

			PCS_INFO("[TreeSegmentation] 单木分割完成，有效树木 %d 颗，用时 %.lf s.",
					 treeId - 1,
					 timer.ElapsedSeconds());


			// 冠幅统计
			ComputeTreeMetrics(trees);
		}

		void Dalponte2016::ComputeLocalMaxima(const cv::Mat& mat,
											  cv::Mat& treetops,
											  std::function<double(double)> dynamic_radius)
		{
			double resolution = _options.resolution;
			double half = resolution / 2.0;
			double eps = resolution * .000001;
			int width = mat.cols;
			int height = mat.rows;

			std::vector<int> surface;
			std::map<int, std::array<int, 2>> indexMap;

			cv::Mat binary(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

			for (int r = 0; r < mat.rows; ++r)
			{
				for (int c = 0; c < mat.cols; ++c)
				{
					int index = mat.at<int>(r, c);
					const auto& p = _input->points[index];

					int colorIndex = std::max(
						0,
						std::min(255,
								 (int)((p.z - _input->bbox.zMin()) /
									   (_input->bbox.zMax() - _input->bbox.zMin()) * 255)));

					auto color = ColorManager::colorMap[colorIndex];

					if (index != -1)
					{
						binary.at<cv::Vec3b>(r, c) =
							cv::Vec3b(color.b() * 255, color.g() * 255, color.r() * 255);

						surface.push_back(index);

						indexMap[index] = { r, c };
					}
				}
			}

#ifdef _treeSegment_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + StringPrintf("binary.jpg"), binary);
#endif
			std::vector<bool> localmax;
			localmax.resize(surface.size());

			// 使用单元最高点创建用于查找局部最高点的点云
			pcl::PointCloud<pcl::PointXY>::Ptr cloud = CreateCloud2D(_input, surface);
			pcl::search::Search<pcl::PointXY>::Ptr tree(new pcl::search::KdTree<pcl::PointXY>);
			tree->setInputCloud(cloud);

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)surface.size(); ++i)
			{
				const auto& p = _input->points[surface[i]];

				pcl::PointXY p2d = { p.x, p.y };
				localmax[i] = true;

				double radius = dynamic_radius(p.hag);

				// 考虑到半径2m内的邻居
				std::vector<int> neighbors;
				std::vector<float> distances;
				tree->radiusSearch(p2d, radius, neighbors, distances);

				for (size_t n = 0; n < neighbors.size(); ++n)
				{
					const auto& pn = _input->points[surface[neighbors[n]]];

					if (pn.z > p.z)
					{
						localmax[i] = false;
						break;
					}
				}
			}

			_treeMetrics.clear();

			// 生成种子点
			int treeId = 1;
			cv::Mat seeds(height, width, CV_32SC1, cv::Scalar(0));

			for (size_t i = 0; i < localmax.size(); ++i)
			{
				if (localmax[i])
				{
					auto it = indexMap.find(surface[i]);
					CHECK(it != indexMap.end());
					int r = (it->second)[0];
					int c = (it->second)[1];

#ifdef _treeSegment_OPENCV_DEBUG
					cv::circle(binary, cv::Point(c, r), 1, cv::Scalar(255, 255, 255));
#endif
					seeds.at<int>(r, c) = treeId;

					const auto& p = _input->points[surface[i]];

					TreeMetrics metric = { treeId, p.x, p.y, p.z, p.hag, 0, 0, 0 };
					_treeMetrics.push_back(metric);
					treeId++;
				}
			}

			CHECK(_treeMetrics.size() == treeId - 1);
			PCS_INFO("[TreeSegmentation] 检测到树冠种子点: %d.", treeId - 1);

#ifdef _treeSegment_OPENCV_DEBUG
			cv::imwrite(DebugDirectory + StringPrintf("seeds.jpg"), binary);
#endif

			treetops = seeds;
		}


		void Dalponte2016::SegmentTree(const cv::Mat& chm,
									   const cv::Mat& treetops,
									   cv::Mat& result,
									   double th_seed /*= 0.45*/,
									   double th_crown /*= 0.55*/,
									   double th_tree /*= 2*/,
									   double distance /*= 10*/)
		{
			double resolution = _options.resolution;

			bool grown = true; // 标识是否据需生长
			bool expend = false;
			int nrow = chm.rows;
			int ncol = chm.cols;

			CHECK_MSG(treetops.rows == nrow && treetops.cols == ncol,
					  "ERROR: 意外的内部错误，矩阵大小不同.");

			std::vector<Pixeld> neighbours(4);

			cv::Mat region = treetops.clone();
			cv::Mat region_temp = treetops.clone();

			std::map<int, Pixeli> seeds; // 将所有种子存储为像素对象
			std::map<int, double> sum_height; // 存储树的每个像素的高程之和（以计算平均高度）
			std::map<int, int> npixel;

			for (int i = 0; i < nrow; i++)
			{
				for (int j = 0; j < ncol; j++)
				{
					int treeid = treetops.at<int>(i, j);

					if (treeid != 0) // 0 为背景
					{
						seeds[treeid] = Pixeli(i, j, treeid);
						sum_height[treeid] = chm.at<float>(i, j);
						npixel[treeid] = 1;
					}
				}
			}

			while (grown)
			{
				grown = false;

				// 处理CHM图像上的像素
				for (int r = 1; r < nrow - 1; r++)
				{
					for (int k = 1; k < ncol - 1; k++)
					{
						if (region.at<int>(r, k) != 0) // 如果像素已经被标记
						{
							int id = region.at<int>(r, k); // 树冠ID

							Pixeli seed = seeds[id];
							double hSeed = chm.at<float>(seed.x, seed.y);
							// 树冠平均高度
							double mhCrown = sum_height[id] / npixel[id];

							// 4个相邻像素的高度
							neighbours[0] = Pixeld(r - 1, k, chm.at<float>(r - 1, k));
							neighbours[1] = Pixeld(r, k - 1, chm.at<float>(r, k - 1));
							neighbours[2] = Pixeld(r, k + 1, chm.at<float>(r, k + 1));
							neighbours[3] = Pixeld(r + 1, k, chm.at<float>(r + 1, k));

							// 处理相邻像素
							for (size_t i = 0; i < neighbours.size(); i++)
							{
								Pixeld px = neighbours[i];

								if (px.z > th_tree) // 仅处理高于树高阈值的部分
								{
									double dist = std::sqrt((seed.x - px.x) * (seed.x - px.x) +
															(seed.y - px.y) * (seed.y - px.y)) *
												  resolution;

									expend = px.z > hSeed * th_seed && px.z > mhCrown * th_crown &&
											 px.z <= hSeed + hSeed * 0.05 &&
											 /*abs(seed.x - px.x) < distance &&
											 abs(seed.y - px.y) < distance &&*/
											 dist < distance && 
											 region.at<int>(px.x, px.y) == 0;

									if (expend) // 相邻像素属于当前树冠的区域
									{
										region_temp.at<int>(px.x, px.y) = region.at<int>(r, k);
										npixel[id]++;
										// 更新区域高度的总和
										sum_height[id] += chm.at<float>(px.x, px.y);
										grown = true;
									}
								}
							}
						}
					}
				}

				region = region_temp.clone();
			}

			result = region;
		}

		void Dalponte2016::ComputeTreeMetrics(const std::map<int, std::vector<int>>& treeIndicesMap)
		{
			if (_treeMetrics.empty())
				return;

			Timer timer;
			timer.Start();

			for (auto iter = treeIndicesMap.begin(); iter != treeIndicesMap.end(); ++iter)
			{
				int treeId = iter->first;
				const std::vector<int>& indices = iter->second;

				int index = treeId - 1;
				CHECK(index >= 0 && index < _treeMetrics.size());

				auto& metric = _treeMetrics[index];

				osg::BoundingBox bbox;
				computeMinMax3D(*_input, indices, bbox);

				metric.crownSize = sqrt((bbox.xMax() - bbox.xMin()) * (bbox.xMax() - bbox.xMin()) +
										(bbox.yMax() - bbox.yMin()) * (bbox.yMax() - bbox.yMin()));

				// 网格法计算面积
				int numCells = 0;
				double cellsize = 0.5;
				typedef std::array<int, 2> Index;
				std::unordered_map<Index, int, hash_eigen<Index>> countMap;

				for (size_t i = 0; i < indices.size(); ++i)
				{
					const auto& p = _input->points[indices[i]];

					int xi = std::floor((p.x - bbox.xMin()) / cellsize);
					int yi = std::floor((p.y - bbox.yMin()) / cellsize);

					Index pi = { xi, yi };
					countMap[pi]++;
				}


				for (auto iter2 = countMap.begin(); iter2 != countMap.end(); ++iter2)
				{
					if (iter2->second > 0)
						++numCells;
				}

				metric.crownArea = numCells * (cellsize * cellsize);
			}

			PCS_INFO("[ComputeTreeMetrics] 树木冠幅属性统计，用时 %.lf s.", timer.ElapsedSeconds());
		}

		pcl::PointCloud<pcl::PointXY>::Ptr Dalponte2016::CreateCloud2D(
			PointCloudViewPtr input,
			const std::vector<int>& indices)
		{
			CHECK(input);
			CHECK(!indices.empty());

			pcl::PointCloud<pcl::PointXY>::Ptr cloud2D(new pcl::PointCloud<pcl::PointXY>());

			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& p = input->points[indices[i]];
				cloud2D->points.push_back({ p.x, p.y });
			}

			cloud2D->width = cloud2D->size();
			cloud2D->height = 1;

			return cloud2D;
		}
	

		// TreeSegmentation 单木分割 Li2012算法
		//////////////////////////////////////////////////////////////////////////
		Li2012::Li2012(const TreeSegmentationOptions& options,
					   PointCloudViewPtr input,
					   const std::vector<int>& indices)
			: _options(options), _input(input)
		{
		}

		Li2012::~Li2012() {}

		std::vector<TreeMetrics> Li2012::GetTreeMetrics() const { return _treeMetrics; }

		void Li2012::Run()
		{
			double dt1 = _options.dt1;
			double dt2 = _options.dt2;
			double Zu = _options.Zu;
			double R = _options.R;
			double th_tree = _options.th_tree;
			double radius = _options.radius;

			if (!_input->normalized)
				return;

			if (_indices.empty())
				return;

			Timer timer;
			timer.Start();

			int numTree = 0;
			std::vector<int> idtree;

			SegmentTree(idtree, dt1, dt2, Zu, R, th_tree, radius);

			for (size_t i = 0; i < idtree.size(); ++i)
			{
				if (idtree.at(i) > 0)
					numTree++;
			}

			PCS_INFO("[TreeSegmentation] 单木分割完成，有效树木 %d 颗，用时 %.lf s.",
					 numTree,
					 timer.ElapsedSeconds());
		}

		std::vector<bool> Li2012::ComputeLocalMaxima(const std::vector<double>& ws,
													 double min_height,
													 bool circular)
		{
			double cellsize = 2.0;

			size_t npoints = _indices.size();
			std::vector<bool> filter(_input->size(), false);

			// 划分格网，加速检索
			Grid2D grid;
			ComputeLocalGridCells(cellsize, _input, _indices, grid);

			bool abort = false;
			bool vws = ws.size() > 1;

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)npoints; ++i)
			{
				if (abort)
					continue;

				const auto& p = _input->points[_indices[i]];

				double hws = (vws) ? (ws[i] / 2.0) : (ws[0] / 2.0);

				if (p.hag < min_height)
					continue;

				// 获取以当前点为中心的窗口内的点
				std::vector<int> indices;

				if (!circular)
				{
					Rectangle rect(p.x - hws, p.x + hws, p.y - hws, p.y + hws);
					GridLookup(grid, rect, indices);
				}
				else
				{
					Circle circle(p.x, p.y, hws);
					GridLookup(grid, circle, indices);
				}

				// 使用中心点初始化最高点
				double zmax = p.z;
				double is_local_max = true;

				// 在范围内查找比当前点更高的点
				for (size_t j  = 0; j < indices.size(); ++j)
				{
					const auto& q = _input->points[indices[j]];
					double z = q.z;

					// 找到一个更高的，则当前点不是局部最大值
					if (z > zmax)
					{
						is_local_max = false;
						break;
					}

					// 相同高度，如果这一个已经被标记为局部最大，不存在两个局部最大，保留第一个
					if (z == zmax && filter[indices[j]])
					{
						is_local_max = false;
						break;
					}
				}

				filter[_indices[i]] = is_local_max;
			}

			return filter;
		}

		void Li2012::SegmentTree(std::vector<int>& idtree,
								 double dt1 /*= 1.5*/,
								 double dt2 /*= 2.0*/,
								 double Zu /*= 15.0*/,
								 double R /*= 2.0*/,
								 double th_tree /*= 2.0*/,
								 double radius /*= 10.0*/)
		{
			osg::BoundingBox bbox;
			computeMinMax3D(*_input, _indices, bbox);
			
			double xmin = bbox.xMin();
			double ymin = bbox.yMin();

			size_t npoints = _indices.size();
			size_t ni = npoints;		// 点数量
			size_t n = ni;				// 剩余点数量
			size_t k = 1;				// 当前树ID

			// 每个点的ID（返回的对象）
			idtree.resize(_input->size(), NA_INTEGER);
			
			// 加快计算速度
			radius = radius * radius;
			dt1 = dt1 * dt1;
			dt2 = dt2 * dt2;

			std::vector<bool> is_local_max;
			
			if (R > 0)
			{
				// 计算局部最大值
				is_local_max = ComputeLocalMaxima({ R }, 0, true);
			}
			else
			{
				is_local_max.resize(_input->size(), true);
			}
			
			// U 要分类的点云
			std::vector<int> U = _indices;

			// N 和 P 分组
			std::vector<int> P, N;
			P.reserve(100);
			N.reserve(100);
			
			// 数据集外的虚拟点
			PointPCLH dummy;
			dummy.x = xmin - 100.0;
			dummy.y = ymin - 100.0;
			dummy.z = 0.0;

			// U 按z值降序排序
			std::sort(U.begin(), U.end(), [&](const int& lhs, const int& rhs) {
				return _input->points[lhs].z > _input->points[rhs].z;
			});


			while (n > 0)
			{
				int u = U[0];
				std::vector<bool> inN(n);

				if (_input->points[u].hag < th_tree)
				{
					// DO NOTHING
				}
				else
				{
					// 初始化，P和N为空
					P.clear();
					N.clear();

					// 元素 0 是当前最高点，位于P（目标树）中
					P.push_back(u);
					idtree[u] = k;

					// 添加虚拟点到 N
					N.push_back(-1);

					// 计算当前点 u 和 U 的所有其他点之间的距离。 
					// 这不是原始算法中的。这是减少计算时间的优化（见第136行）。
					std::vector<double> d = SquareDistance(U, u, dummy); 
					
					// 遍历U的每个点上（全局最大值已经在P中）
					for (size_t i = 1; i < n; ++i)
					{
						u = U[i];

						// 如果 d > radius，则此点离 u 较远，因此不在当前分段树中
						if (d[i] > radius)
						{
							inN[i] = true;
						}
						else  // 如果 d < radius，则根据Li等人的规则对点u进行分类
						{
							std::vector<double> dP = SquareDistance(P, u, dummy);
							std::vector<double> dN = SquareDistance(N, u, dummy);

							double dmin1 = *std::min_element(dP.begin(), dP.end());
							double dmin2 = *std::min_element(dN.begin(), dN.end());
							double dt = (_input->points[u].hag > Zu) ? dt2 : dt1;

							if (is_local_max[u]) // 如果 u 是局部最大值
							{
								if (dmin1 > dt || (dmin1 < dt && dmin1 > dmin2))
								{
									inN[i] = true;
									N.push_back(u);
								}
								else
								{
									P.push_back(u);
									idtree[u] = k;
								}
							}
							else // 如果 u 不是局部最大值
							{
								if (dmin1 <= dmin2)
								{
									P.push_back(u);
									idtree[u] = k;
								}
								else
								{
									inN[i] = true;
									N.push_back(u);
								}
							}
						}
					} // end for
				}

				// 将该点保留在N中，并使用重新分配的点重复循环
				std::vector<int> temp;
				temp.reserve(N.size() - 1);

				for (size_t i = 0; i < n; ++i)
				{
					if (inN[i])
						temp.push_back(U[i]);
				}

				std::swap(U, temp);
				n = U.size();
				++k; // 树ID增加
			} // end while

			std::map<int, std::vector<int>> treesCandidate, trees;
			ColorManager::treeSegments.resize(_input->size(), -1);
			ColorManager::treeColorMap.clear();

			for (size_t i = 0; i < idtree.size(); ++i)
			{
				if (idtree[i] <= 0)
					continue;

				int id = idtree[i];

				treesCandidate[id].push_back(i);
			}
			
			cv::RNG rng(time(0));
			int treeId = 1;

			for (auto iter = treesCandidate.begin(); iter != treesCandidate.end(); ++iter)
			{
				int oldId = iter->first;
				const auto& indices = iter->second;

				if (indices.size() < _options.min_pts)
					continue;

				trees[treeId] = indices;

				// 为有效树木分配颜色
				auto it = ColorManager::treeColorMap.find(treeId);

				if (it == ColorManager::treeColorMap.end())
				{
					float r, g, b;

					if (_options.green_color_style)
					{
						b = rng.uniform(0.1f, 0.3f);
						g = rng.uniform(0.3f, 0.8f);
						r = rng.uniform(0.1f, 0.3f);
					}
					else
					{
						b = rng.uniform(0.0f, 1.0f);
						g = rng.uniform(0.0f, 1.0f);
						r = rng.uniform(0.0f, 1.0f);
					}

					ColorManager::treeColorMap[treeId] = osg::Vec4(r, g, b, 1.0f);
				}

				// 设置点云颜色
				for (size_t i = 0; i < indices.size(); ++i)
				{
					auto& p = _input->points[indices[i]];
					p.data[3] = treeId; // 设置点所属树木 Id

					ColorManager::treeSegments[indices[i]] = treeId;
				}


				++treeId;
			}

			treesCandidate.clear();
		}

		std::vector<double> Li2012::SquareDistance(const std::vector<int>& indices,
												   int index,
												   const PointPCLH& dummy)
		{
			std::vector<double> y(indices.size());

			const auto& u = (index == -1) ? dummy : _input->points[index];

			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& p = (indices[i] == -1) ? dummy : _input->points[indices[i]];
				double dx = p.x - u.x;
				double dy = p.y - u.y;
				y[i] = dx * dx + dy * dy;
			}

			return y;
		}

		void WriteTreeMetrics(const std::vector<TreeMetrics>& treeMetrics, const std::string& path)
		{
			std::ofstream file(path, std::ios::trunc);
			CHECK_MSG(file.is_open(), path.c_str());

			// 确保存储在文本中不会丢失任何精度
			file.precision(17);

			file << "树木ID, X, Y, Z, 树高, 冠幅径, 冠幅面积" << std::endl;

			for (const auto& metric : treeMetrics)
			{
				file << metric.id << ", ";
				file << metric.x << ", ";
				file << metric.y << ", ";
				file << metric.z << ", ";
				file << metric.height << ", ";
				file << metric.crownSize << ", ";
				file << metric.crownArea << std::endl;
			}

			file.close();

			PCS_INFO("[WriteTreeMetrics] 树木属性写入 %s.", path.c_str());
		}

	}
}