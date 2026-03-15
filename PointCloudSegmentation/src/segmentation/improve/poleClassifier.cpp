//stdafx.h
#include "poleClassifier.h"

#include "../../core/private/statistics.h"
#include "../../core/api.h"
#include "../../core/private/cloudProcess.h"

#include "../../plot/plotHandle.h"

#include "../../algorithm/math.h"
#include "../../algorithm/geometry2d_op.h"

#include "../../segmentation/gridCell.h"
#include "../../segmentation/pylonCommon.h"
#include "../../utils/logging.h"


// #define RENDER_POSITIONS
#define RENDER_CLUSTER_BOUND

namespace d3s {
	namespace pcs {

		// PoleClassifier 电杆分类器
		//////////////////////////////////////////////////////////////////////////
		PoleClassifier::PoleClassifier(const PoleClassifyOptions& options,
									   PointCloudViewPtr input,
									   const std::vector<int>& indices)
			: _options(options), _input(input), _indices(indices)
		{
		}

		PoleClassifier::~PoleClassifier() {}

		PoleClassifier::Polygons PoleClassifier::GetPolePolygons() const { return _polePolygons; }

		PoleClassifier::Vectors PoleClassifier::GetPoleSides() const { return _poleSides; }

		void PoleClassifier::SetPolePositions(const Positions& polePositions)
		{
			_polePositions = polePositions;
		}

		PoleClassifier::Positions PoleClassifier::GetPolePositions() const
		{
			return _polePositions;
		}

		void PoleClassifier::Segment()
		{
			double poleRadius = _options.pole_radius;
			double poleHeadLength = _options.head_length;
			double T_h = _options.height_threshold;

			// 计算信息的电杆位置
			auto poleIndicesArr = GetPoleIndices(_input, _indices);
			auto poleAxes = ComputeTrunkCentroids(_input, poleIndicesArr);

			// 计算电杆边界和横担方向
			ComputePolePolygons(_input, poleIndicesArr, _polePolygons, _poleSides);
			CHECK(poleIndicesArr.size() == _polePositions.size());

#ifdef RENDER_POSITIONS
			RenderPylonPositions(GetPolePositions(), 0.5, _input);
#endif

			// 质心半径范围内的电杆候选点为电杆最终分类点
			for (size_t i = 0; i < poleIndicesArr.size(); ++i)
			{
				const auto& indices = poleIndicesArr.at(i);
				const auto& axis = poleAxes.at(i);
				const auto& polygon = _polePolygons.at(i);

				std::vector<int> poleIndices;

				for (size_t j = 0; j < indices.size(); ++j)
				{
					const auto& p = _input->points[indices[j]];
					osg::Vec3 P(p.x, p.y, p.z);
					osg::Vec3 Q = axis.center;
					osg::Vec3 v = axis.dir;
					v.normalize();

					double dist = ((P - Q) ^ v).length();

					if (dist < poleRadius)
						poleIndices.push_back(indices[j]);
				}

				// 杆头粗提取
				std::vector<int> headIndices;
				std::vector<int> headCandidate =
					GetPoleHeadCandidate(_input, indices, polygon, T_h);
				GetHeadIndices(_input, headCandidate, poleHeadLength, headIndices);

				poleIndices.insert(poleIndices.end(), headIndices.begin(), headIndices.end());
				setClassification(ePylon, poleIndices, *_input);


			}
		}

		std::vector<std::vector<int>> PoleClassifier::GetPoleIndices(
			PointCloudViewPtr input,
			const std::vector<int>& indices)
		{
			double size = _options.bound_size;
			const auto& positions = GetPolePositions();

			std::vector<osg::BoundingBox> poleBounds(positions.size());

			// 每个电杆位置的局部点云边界
			for (size_t i = 0; i < positions.size(); ++i)
			{
				const auto& position = positions.at(i);
				poleBounds[i] = osg::BoundingBox(position.x() - size,
												 position.y() - size,
												 input->bbox.zMin(),
												 position.x() + size,
												 position.y() + size,
												 input->bbox.zMax());
			}

			// 提取局部边界内点云
			std::vector<std::vector<int>> poleIndiceArr(poleBounds.size());

			for (int i = 0; i < (int)indices.size(); ++i)
			{
				const auto& p = input->points[indices[i]];

				for (size_t j = 0; j < poleBounds.size(); ++j)
				{
					const auto& bbox = poleBounds[j];

					if (p.x >= bbox.xMin() && p.x < bbox.xMax() && p.y >= bbox.yMin() &&
						p.y < bbox.yMax())
					{
						poleIndiceArr[j].push_back(indices[i]);
					}
				}
			}

			return poleIndiceArr;
		}

		std::vector<int> PoleClassifier::GetPoleHeadCandidate(
			PointCloudViewPtr input,
			const std::vector<int>& indices,
			const std::vector<osg::Vec3d>& polygon,
			double heightThr)
		{
			std::vector<int> indicesTemp;

			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& p = _input->points[indices[i]];
				if (pointInPolygon(polygon, p.x, p.y) && p.hag > heightThr)
					indicesTemp.push_back(indices[i]);
			}

			return indicesTemp;
		}

		std::vector<PoleClassifier::PoleAxis> PoleClassifier::ComputeTrunkCentroids(
			PointCloudViewPtr input,
			const std::vector<std::vector<int>>& arrIndices)
		{
			double maxDist = _options.pole_clustering_dist;
			double minAreaOverlap = _options.pole_min_area_overlap;

			std::vector<PoleClassifier::PoleAxis> poleAxes;
			poleAxes.reserve(arrIndices.size());

			// 计算每个电杆局部点云的电杆精确位置
			for (size_t i = 0; i < arrIndices.size(); ++i)
			{
				const auto& poleIndices = arrIndices.at(i);

				// 获得电杆主干
				std::vector<Trunk> trunks =
					GetPoleTrunkCandidate(input, poleIndices, _options.layer_step, maxDist, i);

				std::vector<std::vector<int>> trunkClusters;
				TrunkClustering(trunks, minAreaOverlap, trunkClusters);
				std::vector<Trunk> mainTrunks = GetMainTrunks(trunks, trunkClusters);

#ifdef RENDER_CLUSTER_BOUND
				for (size_t j = 0; j < mainTrunks.size(); ++j)
				{
					const auto& trunk = mainTrunks.at(j);
					std::string clustername = StringPrintf("cluster_%d_%d", i, j);
					RenderAABB(trunk.bound, input, clustername);
				}
#endif
				// 计算质心
				PoleAxis axis;
				GetMainVector(input, mainTrunks, axis.center, axis.dir);
				poleAxes.push_back(axis);

#ifdef RENDER_CLUSTER_BOUND
				std::vector<osg::Vec3d> path = { axis.center - axis.dir * 6.0,
												 axis.center + axis.dir * 6.0 };
				RenderLinePath(StringPrintf("pole_pca_v_%d", i),
							   path,
							   input,
							   osg::Vec4(1.f, 1.f, 0.f, 1.f));
#endif

			}

			return poleAxes;
		}

		void PoleClassifier::ComputePolePolygons(PointCloudViewPtr input,
												 const std::vector<std::vector<int>>& arrIndices,
												 Polygons& polePolygons,
												 Vectors& poleSides)
		{
			double defaultRadius = 1.0;
			double d1_scale = _options.d1_scale;
			double d2_scale = _options.d2_scale;
			double poleHeadLength = _options.head_length;
			double T_h = _options.height_threshold;

			Positions polePositions = GetPolePositions();
			Positions newPolePositions = polePositions;

			polePolygons.resize(polePositions.size());
			poleSides.resize(polePositions.size());
			std::vector<double> poleRadius(polePositions.size(), defaultRadius);

			for (size_t i = 0; i < polePositions.size(); ++i)
			{
				const auto& poleIndices = arrIndices[i];
				const auto& cur = polePositions[i];
				osg::Vec3d dir, side;

				// 计算杆塔的朝向
				if (i == 0)
					side = GetSideVector(polePositions, i, i + 1);
				else if (i == polePositions.size() - 1)
					side = GetSideVector(polePositions, i - 1, i);
				else
					side = GetSideVector(polePositions, i - 1, i, i + 1);

				dir = side ^ osg::Z_AXIS;
				double dLen = defaultRadius;
				std::vector<osg::Vec3d> polygon(4);

				polygon[0] = cur - dir * 0.3 * dLen - side * 1.1 * dLen;
				polygon[1] = cur + dir * 0.3 * dLen - side * 1.1 * dLen;
				polygon[2] = cur + dir * 0.3 * dLen + side * 1.1 * dLen;
				polygon[3] = cur - dir * 0.3 * dLen + side * 1.1 * dLen;

				// 获得塔头点云
				std::vector<int> headIndices;
				std::vector<int> headCandidate =
					GetPoleHeadCandidate(input, poleIndices, polygon, T_h);

				GetHeadIndices(input, headCandidate, poleHeadLength, 0.1, 10, headIndices);

				/*
				// 重新计算横担方向
				if (i == 0 || i == polePositions.size() - 1)
				{
					GetHeadDirection(input, headCandidate, 1.0, side, i);
					dir = side ^ osg::Z_AXIS;


					// 更新杆塔边界
					polygon[0] = cur - dir * 0.3 * dLen - side * 1.1 * dLen;
					polygon[1] = cur + dir * 0.3 * dLen - side * 1.1 * dLen;
					polygon[2] = cur + dir * 0.3 * dLen + side * 1.1 * dLen;
					polygon[3] = cur - dir * 0.3 * dLen + side * 1.1 * dLen;

					headCandidate = GetPoleHeadCandidate(input, poleIndices, polygon, T_h);

					// 再次获得纠正杆塔边界后的塔头点云
					GetHeadIndices(input, headCandidate, poleHeadLength, 0.1, 10, headIndices);
				}
				*/

				poleSides[i] = side; // 杆塔横担方向

				// 计算塔头半径
				if (!headIndices.empty())
				{
					poleRadius[i] = ComputePositionRadius(input,
														  headIndices,
														  poleSides[i],
														  newPolePositions[i],
														  i);
				}

				// 终端塔、转角塔不调整杆塔坐标
				newPolePositions[i] = polePositions[i];
			}

			SetPolePositions(newPolePositions);
			polePositions = GetPolePositions();

			// 第二次，根据实际杆塔位置和半径，构建杆塔边界
			for (size_t i = 0; i < polePositions.size(); ++i)
			{
				const auto& cur = polePositions[i];
				const auto& side = poleSides[i];
				osg::Vec3d dir = side ^ osg::Z_AXIS;

				double dLen = poleRadius[i];
				std::vector<osg::Vec3d> polygon(4);

				// 正方形
				polygon[0] = cur - dir * d1_scale * dLen - side * d2_scale * dLen;
				polygon[1] = cur + dir * d1_scale * dLen - side * d2_scale * dLen;
				polygon[2] = cur + dir * d1_scale * dLen + side * d2_scale * dLen;
				polygon[3] = cur - dir * d1_scale * dLen + side * d2_scale * dLen;

#ifdef _render_Pylon_Mask
				RenderPylonMask(polygon, input, StringPrintf("polemask%d", i));
#endif
				polePolygons[i] = polygon;
			}
		}


		double PoleClassifier::ComputePositionRadius(PointCloudViewPtr input,
													 const std::vector<int>& indices,
													 const osg::Vec3d& side,
													 osg::Vec3d& pos,
													 int post)
		{
			CHECK(!indices.empty());

			double towerHeadTolerance = 0.1;
			int towerHeadMinPts = 100;

			std::vector<int> headIndices = indices;

			// 初始杆塔边界范围内可能会存在少许植被点，过滤它们
			{
				std::vector<std::vector<int>> clusters;

				euclideanClusterSafe(*input,
									 headIndices,
									 towerHeadTolerance,
									 towerHeadMinPts,
									 std::numeric_limits<int>::max(),
									 clusters);

				std::string fmt = "[PoleClassifier] 杆头点云类簇数 %d";

				if (!clusters.empty())
				{
					int maxPts = 0;
					std::vector<int> mainCluster;

					fmt += "(";

					for (size_t i = 0; i < clusters.size(); ++i)
					{
						const auto& cluster = clusters.at(i);

						if (cluster.size() > maxPts)
						{
							mainCluster = cluster;
							maxPts = cluster.size();
						}

						fmt += std::to_string(cluster.size());
						if (i != clusters.size() - 1)
							fmt += ", ";
					}

					fmt += ")";

					if (mainCluster.size() > towerHeadMinPts)
						headIndices = mainCluster;
				}

				PCS_DEBUG(fmt.c_str(), clusters.size());
			}

			if (headIndices.empty())
				headIndices = indices;

			// 计算类簇点集在XY平面上的特征值、特征向量
			osg::Vec3d center;
			osg::BoundingBox towerHeadBound;
			std::vector<osg::Vec3d> pts;

			for (size_t i = 0; i < headIndices.size(); ++i)
			{
				const auto& p = input->points[headIndices[i]];
				osg::Vec3d v(p.x, p.y, p.z);

				towerHeadBound.expandBy(v);
				// center += v / (double)(finalIndices.size());
			}

			// 中心点采用边界框的中心，防止点云分布不均匀导致的偏移
			center = towerHeadBound.center();

			for (size_t i = 0; i < headIndices.size(); ++i)
			{
				const auto& p = input->points[headIndices[i]];
				osg::Vec3d v = osg::Vec3d(p.x, p.y, p.z) - center;
				v.z() = 0; // 只考虑 XY平面上的特征值、特征向量
				pts.push_back(v);
			}

			/*osg::Vec3d v1, v2, v3;
			getEigenVectors(pts, v1, v2, v3);
			v1.normalize();*/
			osg::Vec3d v1 = side;
			v1.normalize();

			double maxProj = -DBL_MAX;
			double minProj = DBL_MAX;

			for (size_t i = 0; i < pts.size(); ++i)
			{
				const osg::Vec3d& v = pts.at(i);
				maxProj = std::max(maxProj, v * v1);
				minProj = std::min(minProj, v * v1);
			}

			double projLen = maxProj - minProj + 0.5;
			double pylonRadius = projLen * 0.5;

			PCS_DEBUG("[PoleClassifier] 电杆点云主方向最大投影长度 %lf，铁塔点云半径 %lf",
					  projLen,
					  pylonRadius);

			/*std::vector<osg::Vec3d> path = { center, center + v1 * pylonRadius };
			RenderLinePath(StringPrintf("pole_pca_h_%d", post),
						   path,
						   input,
						   osg::Vec4(1.f, 1.f, 0.f, 1.f));*/

			pos = center;
			return pylonRadius;
		}
	}
}