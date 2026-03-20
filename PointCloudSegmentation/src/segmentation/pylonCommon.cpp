#include <src/segmentation/pylonCommon.h>
#include <src/segmentation/gridCell.h>

#include <src/algorithm/math.h>
#include <src/algorithm/geometry2d_op.h>
#include <src/core/private/rasterProcess.h>
#include <src/core/private/cloudProcess.h>
#include <src/core/private/statistics.h>
#include <src/plot/plotHandle.h>

// #define WRITE_PYLON_HEAD_MASK
// #define RENDER_LAYER_BOUND

extern std::string DebugDirectory;

namespace d3s {
	namespace pcs {

		osg::Vec3d GetSideVector(const Positions& positions, int i, int j)
		{
			CHECK(i >= 0 && i < (int)positions.size());
			CHECK(j >= 0 && j < (int)positions.size());

			osg::Vec3d front, side;
			const auto& p = positions[i];
			const auto& q = positions[j];

			front = q - p;
			front.z() = 0.0;
			front.normalize();
			side = front ^ osg::Z_AXIS;
			side.normalize();

			return side;
		}

		osg::Vec3d GetSideVector(const Positions& positions, int i, int j, int k)
		{
			CHECK(i >= 0 && i < (int)positions.size());
			CHECK(j >= 0 && j < (int)positions.size());
			CHECK(k >= 0 && k < (int)positions.size());

			osg::Vec3d side;
			const auto& prev = positions[i];
			const auto& cur = positions[j];
			const auto& next = positions[k];

			osg::Vec3d backward = (prev - cur);
			osg::Vec3d forward = (next - cur);
			forward.z() = backward.z() = 0.0;
			forward.normalize();
			backward.normalize();

			side = forward + backward;
			side.normalize();

			// 处理平行的状况
			if (fabs(forward * side) > (1.0 - 0.0001))
			{
				side = forward ^ osg::Z_AXIS;
				side.normalize();
			}

			return side;
		}

		void GetHeadIndices(PointCloudViewPtr input,
							const std::vector<int>& indices,
							double height,
							std::vector<int>& headIndices)
		{
			CHECK(height >= 0.0);

			if (indices.empty())
			{
				PCS_WARN("[GetPylonHeadIndices] 输入点云数量为空.");
				return;
			}

			// 计算上下边界时排除一定的噪声
			double zmin = DBL_MAX;
			double zmax = -DBL_MAX;

			computeMinMax(*input, indices, "z", 10, 0.5, zmin, zmax); // 0.5m 的间隔
			double zLower = std::max(zmin, zmax - height);

			headIndices.clear();
			headIndices.reserve(indices.size());

			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& p = input->points[indices[i]];

				if (p.z > zLower)
					headIndices.push_back(indices[i]);
			}

			headIndices.shrink_to_fit();
		}

		void GetHeadIndices(PointCloudViewPtr input,
							const std::vector<int>& indices,
							double height,
							double step,
							int minPts,
							std::vector<int>& headIndices)
		{
			CHECK(height >= 0.0);
			CHECK(minPts > 0);
			CHECK(step > 0.0);
			

			if (indices.empty())
			{
				PCS_WARN("[GetPylonHeadIndices] 输入点云数量为空.");
				return;
			}

			// 计算上下边界时排除一定的噪声
			double zmin = DBL_MAX;
			double zmax = -DBL_MAX;

			computeMinMax(*input, indices, "z", minPts, step, zmin, zmax); // 0.5m 的间隔
			double zLower = std::max(zmin, zmax - height);

			headIndices.clear();
			headIndices.reserve(indices.size());

			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& p = input->points[indices[i]];

				if (p.z > zLower)
					headIndices.push_back(indices[i]);
			}

			headIndices.shrink_to_fit();
		}

		void GetHeadDirection(PointCloudViewPtr input,
							  const std::vector<int>& indices,
							  double stdmul,
							  osg::Vec3d& dir,
							  int post)
		{
			if (indices.empty())
				return;

			osg::BoundingBox bbox;
			computeMinMax3D(*input, indices, bbox);
			double maxSide = std::max(bbox.xMax() - bbox.xMin(), bbox.yMax() - bbox.yMin());
			double cellsize = maxSide / 100;

			Grid2D grid;
			ComputeLocalGridCells(cellsize, input, indices, grid);

			std::vector<int> counts;
			int cmin = INT_MAX;
			int cmax = -INT_MAX;

			for (size_t i = 0; i < grid.size(); ++i)
			{
				const auto& cell = grid.at(i);

				if (cell.GetSize() <= 0)
					continue;

				cmin = std::min(cmin, (int)cell.GetSize());
				cmax = std::max(cmax, (int)cell.GetSize());
				counts.push_back((double)cell.GetSize());
			}

			// double stdmul = 1.5;
			// double stdmul = 2.0; // fix: 修复某些500kV铁塔塔头方向计算错误的情况
			// int mean = (int)Mean(counts);
			int stddev = (int)Stddev(counts);
			int nRows = grid.getNumRows();
			int nCols = grid.getNumColumns();

			RasterExtents extents(0, 0, nCols, nRows, 1);
			Rasterd mask(extents, "Tower body mask", 0.0);

			for (int r = 0; r < nRows; ++r)
			{
				for (int c = 0; c < nCols; ++c)
				{
					const auto& cell = grid.at(r, c);

					if (cell.GetSize() <= 0)
						continue;

					if (cell.GetSize() > stddev * stdmul)
						mask.at(c, r) = 1.0;
				}
			}

#ifdef WRITE_PYLON_HEAD_MASK
			writeImage(mask, DebugDirectory + StringPrintf("pylon%d_inliers.jpg", post), true);
#endif

			std::vector<osg::Vec3d> pts;
			for (int r = 0; r < nRows; ++r)
			{
				for (int c = 0; c < nCols; ++c)
				{
					if (mask.at(c, r) == 1.0)
					{
						double x = mask.xCellPos(c);
						double y = mask.yCellPos(r);
						pts.push_back(osg::Vec3d(x, y, 0.0));
					}
				}
			}

			osg::Vec3d major, middle, minor;
			getEigenVectors(pts, major, middle, minor);

			dir = major;
		}

		std::vector<Trunk> GetPoleTrunkCandidate(PointCloudViewPtr input,
												 const std::vector<int>& indices,
												 double layerStep,
												 double clusterTolerance,
												 int i)
		{
			double maxAreaPerTrunk = 0.25;
			double heightThreshold = layerStep * 0.8;

			std::vector<Trunk> trunks;
			std::vector<std::vector<int>> layers;
			sliceVerticalLayers(*input, indices, layerStep, layers);

			// 对每个层进行聚类
			for (size_t j = 0; j < layers.size(); ++j)
			{
				const auto& layer = layers.at(j);
				std::string layername = StringPrintf("layer_%d_%d", i, j);
				std::vector<std::vector<int>> clusters;

#ifdef RENDER_LAYER_BOUND
				osg::BoundingBox layer_bbox;
				computeMinMax3D(*input, layer, layer_bbox);
				RenderAABB(layer_bbox, input, layername);
#endif
				// 欧式聚类
				euclideanClusterSafe(*input,
									 layer,
									 clusterTolerance,
									 1,
									 std::numeric_limits<int>::max(),
									 clusters);

				for (size_t k = 0; k < clusters.size(); ++k)
				{
					const auto& cluster = clusters.at(k);
					osg::BoundingBox cluster_box;
					computeMinMax3D(*input, cluster, cluster_box);

					double lx = cluster_box.xMax() - cluster_box.xMin();
					double ly = cluster_box.yMax() - cluster_box.yMin();
					double lz = cluster_box.zMax() - cluster_box.zMin();

					if ((lx * ly < maxAreaPerTrunk) && (lz > heightThreshold))
					{
						Trunk trunk = { (int)j, cluster_box, cluster }; // 主干段
						trunks.push_back(trunk);
					}
				}
			}

			// 按层级升序排序
			std::sort(trunks.begin(), trunks.end(), [](const Trunk& lhs, const Trunk& rhs) {
				return lhs.layer < rhs.layer;
			});

			return trunks;
		}

		std::vector<Trunk> GetMainTrunks(const std::vector<Trunk>& trunks,
										 std::vector<std::vector<int>>& clusters)
		{
			int mainIdx = -1;
			int maxLayerDiff = 0;

			for (size_t i = 0; i < clusters.size(); ++i)
			{
				auto& trunkCluster = clusters.at(i);

				if (trunkCluster.empty())
					continue;

				// 按层级升序排序
				std::sort(trunkCluster.begin(),
						  trunkCluster.end(),
						  [&](const int& il, const int& ir) {
							  return trunks[il].layer < trunks[ir].layer;
						  });

				int istart = *(trunkCluster.begin());
				int iend = *(trunkCluster.end() - 1);
				int layerDiff = trunks[iend].layer - trunks[istart].layer;

				if (layerDiff > maxLayerDiff)
				{
					maxLayerDiff = layerDiff;
					mainIdx = i;
				}
			}

			std::vector<Trunk> result;

			if (mainIdx >= 0)
			{
				const auto& trunkCluster = clusters[mainIdx];

				for (size_t i = 0; i < trunkCluster.size(); ++i)
					result.push_back(trunks[trunkCluster[i]]);
			}

			return result;
		}

		void GetMainVector(PointCloudViewPtr input,
						   const std::vector<Trunk>& trunks,
						   osg::Vec3& center,
						   osg::Vec3& axis)
		{
			if (trunks.empty())
				return;

			std::vector<osg::Vec3d> pts;
			float size = (float)trunks.size();

			for (size_t i = 0; i < trunks.size(); ++i)
			{
				const auto& trunk = trunks.at(i);

				center.x() += trunk.bound.center().x() / size;
				center.y() += trunk.bound.center().y() / size;
				center.z() += trunk.bound.center().z() / size;

				for (size_t j = 0; j < trunk.indices.size(); ++j)
				{
					const auto& p = input->points[trunk.indices[j]];
					osg::Vec3d v = osg::Vec3d(p.x, p.y, p.z);
					pts.push_back(v);
				}
			}

			osg::Vec3d v1, v2, v3;
			getEigenVectors(pts, v1, v2, v3);
			v1.normalize();
			axis = v1;
		}

		osg::Vec3 GetCentroid(PointCloudViewPtr input, const std::vector<Trunk>& trunks)
		{
			osg::Vec3 centroid;

			if (trunks.empty())
				return centroid;

			/*std::vector<int> indices;

			for (size_t i = 0; i < trunks.size(); ++i)
			{
				const auto& trunk = trunks.at(i);
				indices.insert(indices.end(), trunk.indices.begin(), trunk.indices.end());
			}

			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& p = input->points[indices[i]];
				centroid.x() += p.x / (float)(indices.size());
				centroid.y() += p.y / (float)(indices.size());
				centroid.z() = std::max(centroid.z(), p.z);
			}*/


			float size = (float)trunks.size();

			for (size_t i = 0; i < trunks.size(); ++i)
			{
				const auto& trunk = trunks.at(i);

				centroid.x() += trunk.bound.center().x() / size;
				centroid.y() += trunk.bound.center().y() / size;
				centroid.z() += trunk.bound.center().z() / size;
				//centroid.z() = std::max(centroid.z(), trunk.bound.center().z());
			}

			return centroid;
		}

		void TrunkClustering(const std::vector<Trunk>& trunks,
							 double overlapThr,
							 std::vector<std::vector<int>>& clusters)
		{
			if (trunks.empty())
				return;

			std::vector<bool> processed(trunks.size(), false);

			for (int i = 0; i < (int)trunks.size(); ++i)
			{
				// 已处理过的，不再处理
				if (processed[i])
					continue;

				std::vector<int> seed_queue;

				int sq_idx = 0;
				seed_queue.push_back(i);
				processed[i] = true;

				while (sq_idx < (int)seed_queue.size())
				{
					int k = seed_queue[sq_idx];
					const auto& trunk = trunks.at(k);

					for (int j = k + 1; j < (int)trunks.size(); ++j)
					{
						if (processed[j])
							continue;

						const auto& trunkNbr = trunks.at(j);

						Rect rc1(trunk.bound.xMin(),
								 trunk.bound.yMin(),
								 trunk.bound.xMax(),
								 trunk.bound.yMax());
						Rect rc2(trunkNbr.bound.xMin(),
								 trunkNbr.bound.yMin(),
								 trunkNbr.bound.xMax(),
								 trunkNbr.bound.yMax());

						double overlap = rectIntersectionArea(rc1, rc2);
						if (overlap > overlapThr)
						{
							seed_queue.push_back(j);
							processed[j] = true;
						}
					}

					sq_idx++;
				} // end of while

				if (!seed_queue.empty())
					clusters.push_back(seed_queue);
			}
		}

	}
}
