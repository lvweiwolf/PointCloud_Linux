//stdafx.h
#include "wireClassifier.h"

#include "../../algorithm/geometry2d_op.h"

#include "../../core/private/cloudProcess.h"
#include "../../core/private/statistics.h"
#include "../../core/api.h"
#include "../../plot/plotHandle.h"

#include "../../segmentation/gridCell.h"
#include "../../segmentation/pylonCommon.h"
#include "../../utils/logging.h"
#include "../../utils/stringutil.h"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#define RENDER_SLCIE_BOUND
#define RENDER_TRUNKS
// #define RENDER_FITTING_CURVE

// #define CLUSTERING_USING_VOXELS

namespace d3s {
	namespace pcs {

		WireClassifier::WireClassifier(const WireClassifyOptions& options,
									   PointCloudViewPtr input,
									   const std::vector<int>& indices)
			: _options(options), _input(input), _indices(indices)
		{
		}

		WireClassifier::~WireClassifier() {}

		void WireClassifier::SetPolePositions(const Positions& polePositions)
		{
			_polePositions = polePositions;
		}

		void WireClassifier::SetPolePolygons(const Polygons& polePolygons)
		{
			_polePolygons = polePolygons;
		}

		void WireClassifier::SetPoleSides(const Vectors& poleSides) { _poleSides = poleSides; }

		void WireClassifier::Segment()
		{
			if (_polePositions.size() < 2) // 至少两基杆塔
				return;

			double step = _options.slice_step;
			double width = _options.slice_thickness;
			double layerStep = 0.5;
			double layerClusterDist = 0.1;
			double minOverlap = 0.005;
			
			int curveMinPts = _options.curve_min_points;

			Grid2D grid;
			ComputeLocalGridCells(1.0, _input, _indices, grid);

			const Positions& polePositions = _polePositions;
			const Polygons& polePolygons = _polePolygons;
			const Vectors& poleSides = _poleSides;
			PowerlineSpan& beforeSpan = _beforeSpan;
			PowerlineSpan& afterSpan = _afterSpan;
			std::vector<PowerlineSpan>& wireSpans = _wireSpans;

			CHECK(polePositions.size() == polePolygons.size());
			wireSpans.resize(polePositions.size() - 1);

			SpanResult spanBeforeFirst, spanAfterLast;
			SpanResultList spanResults(polePositions.size() - 1);

			std::vector<int> spanIndices;

			// 划分档，并提取档内的电力线
			for (int i = 0; i < (int)polePositions.size() - 1; ++i)
			{
				const auto& curPos = polePositions.at(i);
				const auto& nextPos = polePositions.at(i + 1);

				const auto& curPolygon = polePolygons.at(i);
				const auto& nextPolygon = polePolygons.at(i + 1);

				const auto& curSide = poleSides.at(i);
				const auto& nextSide = poleSides.at(i + 1);

				// 档空间信息
				SpanSpatial spatial = { i,		  curPos,	  nextPos,	  curSide,
										nextSide, curPolygon, nextPolygon };

				// 提取档内点云
				spanResults[i] = ExtractSpanResult(grid, spatial, step, width);

				for (size_t j = 0; j < spanResults[i].sliceInidces.size(); ++j)
				{
					const auto& sliceIndices = spanResults[i].sliceInidces[j];
					spanIndices.insert(spanIndices.end(), sliceIndices.begin(), sliceIndices.end());
				}
			}

			int n = polePositions.size();

			spanBeforeFirst = ExtractOutOfSpanResult(grid,
													 polePositions[0],
													 polePositions[0] - polePositions[1],
													 poleSides[0],
													 polePolygons[0],
													 step,
													 width,
													 0);

			spanAfterLast = ExtractOutOfSpanResult(grid,
												   polePositions[n - 1],
												   polePositions[n - 1] - polePositions[n - 2],
												   poleSides[n - 1],
												   polePolygons[n - 1],
												   step,
												   width,
												   1);


			
			
			for (size_t i = 0; i < spanBeforeFirst.sliceInidces.size(); ++i)
			{
				const auto& sliceIndices = spanBeforeFirst.sliceInidces[i];
				spanIndices.insert(spanIndices.end(), sliceIndices.begin(), sliceIndices.end());
			}

			for (size_t i = 0; i < spanAfterLast.sliceInidces.size(); ++i)
			{
				const auto& sliceIndices = spanAfterLast.sliceInidces[i];
				spanIndices.insert(spanIndices.end(), sliceIndices.begin(), sliceIndices.end());
			}

			// 计算档内点云线性特征
			std::vector<bool> linearity = ComputeLinearity(_input, spanIndices);

			cv::RNG rng(time(0));
			
#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)spanResults.size(); ++i)
			{
				int index = i;
				auto& result = spanResults[i];

				// 每个档内点云的电力线分类
				ClassifyInSpan(result);
				
				// 根据档内点云线性特征修复导线
				ReclassifyWithLinearity(linearity, result);

				int numPowerlines = result.powerlines.size();
				PCS_INFO("[WireClassifier] 提取档 %d 内电力线点云 %d", index, numPowerlines);

				//setClassification(eWater, result.powerlinesFail, *_input);
				setClassification(ePowerline, result.powerlines, *_input);
			
				// 根据拟合曲线修复导线
				wireSpans[i] = ReclassifyWithCurve(result, curveMinPts); 	

				setClassification(ePowerline, result.powerlines, *_input);

				// 获得档内剩余未分类点云
				std::vector<int> unclassified = GetUnclassified(result);
				std::vector<int> curveIndices;

				// 档内剩余未分类点中的线性部分
				ReclassifyWithLinearityAndCurve(linearity,
												_input,
												curveMinPts,
												unclassified,
												curveIndices);

				setClassification(ePowerline, curveIndices, *_input);

				std::vector<Trunk> trunks =
					GetPoleTrunkCandidate(_input, unclassified, layerStep, layerClusterDist, i);

				std::vector<std::vector<int>> trunkClusters;
				TrunkClustering(trunks, minOverlap, trunkClusters);

#ifdef RENDER_TRUNKS
				for (size_t j = 0; j < trunkClusters.size(); ++j)
				{
					const auto& trunkCluster = trunkClusters.at(j);

					float r, g, b;
					b = rng.uniform(0.0f, 1.0f);
					g = rng.uniform(0.0f, 1.0f);
					r = rng.uniform(0.0f, 1.0f);

					for (size_t k = 0; k < trunkCluster.size(); ++k)
					{
						const auto& trunkIdx = trunkCluster.at(k);
						std::string clustername = StringPrintf("cluster_trunk_%d_%d_%d", i, j, k);
						RenderAABB(trunks[trunkIdx].bound,
								   _input,
								   clustername,
								   osg::Vec4(r, g, b, 0.0f),
								   osg::Vec4(r, g, b, 1.0f));
					}
				
				}
#endif
				
				std::vector<std::vector<int>> poleIndicesList;
				ReclassifyPoleFromUnclassified(trunks,
											   trunkClusters,
											   unclassified,
											   poleIndicesList);

				for (size_t j = 0; j < poleIndicesList.size(); ++j)
				{
					const auto& poleIndices = poleIndicesList[j];
					setClassification(ePylon, poleIndices, *_input);
				}
			}

			// 处理前方档边界范围之外的点云
			{
				ClassifyInSpan(spanBeforeFirst);
				
				// 根据档内点云线性特征修复导线
				ReclassifyWithLinearity(linearity, spanBeforeFirst);

				int numPowerlines = spanBeforeFirst.powerlines.size();
				PCS_INFO("[wireClassifier] 提取档前方电力线点云 %d", numPowerlines);

				setClassification(ePowerline, spanBeforeFirst.powerlines, *_input);

				// 根据拟合曲线修复导线
				beforeSpan = ReclassifyWithCurve(spanBeforeFirst, curveMinPts);

				setClassification(ePowerline, spanBeforeFirst.powerlines, *_input);

				// 获得档内剩余未分类点云
				std::vector<int> unclassified = GetUnclassified(spanBeforeFirst);
				std::vector<int> curveIndices;

				// 档内剩余未分类点中的线性部分
				ReclassifyWithLinearityAndCurve(linearity,
												_input,
												curveMinPts,
												unclassified,
												curveIndices);

				setClassification(ePowerline, curveIndices, *_input);
			}

			// 处理前方档边界范围之外的点云
			{
				ClassifyInSpan(spanAfterLast);

				// 根据档内点云线性特征修复导线
				ReclassifyWithLinearity(linearity, spanAfterLast);

				int numPowerlines = spanAfterLast.powerlines.size();
				PCS_INFO("[wireClassifier] 提取档后方电力线点云 %d", numPowerlines);

				setClassification(ePowerline, spanAfterLast.powerlines, *_input);

				// 根据拟合曲线修复导线
				beforeSpan = ReclassifyWithCurve(spanAfterLast, curveMinPts);

				setClassification(ePowerline, spanAfterLast.powerlines, *_input);

				// 获得档内剩余未分类点云
				std::vector<int> unclassified = GetUnclassified(spanAfterLast);
				std::vector<int> curveIndices;

				// 档内剩余未分类点中的线性部分
				ReclassifyWithLinearityAndCurve(linearity,
												_input,
												curveMinPts,
												unclassified,
												curveIndices);

				setClassification(ePowerline, curveIndices, *_input);
			}
		}

		SpanResult WireClassifier::ExtractSpanResult(const Grid2D& grid,
													 const SpanSpatial& spatial,
													 double step,
													 double width)
		{
			osg::Vec3d p0 = spatial.p0;
			osg::Vec3d p1 = spatial.p1;
			p0.z() = p1.z() = std::max(p0.z(), p1.z());

			osg::Vec3d dir = p1 - p0; // 档方向
			dir.normalize();
			osg::Vec3d ortho = dir ^ osg::Z_AXIS;

			int sliceID = 0;
			// 起始塔边界，终止塔边界在档方向上的投影
			// double projection0 = getMaxProjection(p0, p1, spatial.polygon0);
			// double projection1 = getMaxProjection(p1, p0, spatial.polygon1);
			double height0 = std::min((spatial.polygon0[0] - spatial.polygon0[1]).length(),
									  (spatial.polygon0[0] - spatial.polygon0[3]).length());
			double height1 = std::min((spatial.polygon1[0] - spatial.polygon1[1]).length(),
									  (spatial.polygon1[0] - spatial.polygon1[3]).length());

			double width0 = std::max((spatial.polygon0[0] - spatial.polygon0[1]).length(),
									 (spatial.polygon0[0] - spatial.polygon0[3]).length());
			double width1 = std::max((spatial.polygon1[0] - spatial.polygon1[1]).length(),
									 (spatial.polygon1[0] - spatial.polygon1[3]).length());

			width = std::max(width, std::max(width0, width1));
			width += 2.0; // 扩展5m，提高稳定性
			step = std::max(width, step);

			Polygons slicePolygons;
			osg::Vec3d t0 = p0 + dir * height0 * 0.5;
			osg::Vec3d t1 = p1 - dir * height1 * 0.5;
			
			double spanLength = (t1 - t0).length();
			int numSteps = std::max(1, (int)std::floor(spanLength / step));
			step = spanLength / (double)numSteps;

			for (int i = 0; i < numSteps; ++i)
			{
				osg::Vec3d q0 = t0 + dir * i * step;
				osg::Vec3d q1 = t0 + dir * ((i + 1) * step);

				osg::Vec3d side0 = ortho;
				osg::Vec3d side1 = ortho;
				double side_len0 = width;
				double side_len1 = width;

				if (i == 0)
				{
					double cos0 = fabs(spatial.side0 * ortho);

					if (cos0 == 0.0)
						side0 = ortho;
					else
					{
						side0 = spatial.side0;
						side_len0 = width / cos0;
					}
				}
				else if (i == numSteps - 1)
				{
					double cos1 = fabs(spatial.side1 * ortho);

					if (cos1 == 0.0)
						side1 = ortho;
					else
					{
						side1 = spatial.side1;
						side_len1 = width / cos1;
					}
				}

				// 保持同侧
				if (dir * side0 < 0.0)
					side0 = -side0;

				if (side0 * side1 < 0.0)
					side1 = -side1;

				// 计算档范围边界
				std::vector<osg::Vec3d> polygon(4);
				polygon[0] = q0 + side0 * 0.5 * side_len0;
				polygon[1] = q1 + side1 * 0.5 * side_len1;
				polygon[2] = q1 - side1 * 0.5 * side_len1;
				polygon[3] = q0 - side0 * 0.5 * side_len0;

#ifdef RENDER_SLCIE_BOUND // DEBUG: 绘制当边界
				RenderPowerlineMask(polygon,
									_input,
									StringPrintf("sliceBound_%d_%d", spatial.id, sliceID));
#endif

				sliceID++;

				slicePolygons.push_back(polygon);
			}
			
			std::vector<std::vector<int>> sliceIndices;
			sliceIndices.resize(slicePolygons.size());

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)slicePolygons.size(); ++i)
			{
				const auto& polygon = slicePolygons.at(i);
				osg::BoundingBox bbox = getPolygonBound(polygon);

				for (size_t j = 0; j < grid.size(); ++j)
				{
					const auto& cell = grid.at(j);

					if (cell.GetSize() <= 0)
						continue;

					if (!(osg::maximum(cell.xMin(), bbox.xMin()) <=
							  osg::minimum(cell.xMax(), bbox.xMax()) &&
						  osg::maximum(cell.yMin(), bbox.yMin()) <=
							  osg::minimum(cell.yMax(), bbox.yMax())))
					{
						continue;
					}

					const auto& cellIndices = cell.GetIndices();

					for (size_t k = 0; k < cellIndices.size(); ++k)
					{
						const auto& p = _input->points[cellIndices[k]];

						if (pointInPolygon(polygon, p.x, p.y))
							sliceIndices[i].push_back(cellIndices[k]);
					}
				}
			}

			return { spatial.id, dir, sliceIndices, slicePolygons };
		}

		SpanResult WireClassifier::ExtractOutOfSpanResult(const Grid2D& grid,
														  const osg::Vec3d& p,
														  const osg::Vec3d& v,
														  const osg::Vec3d& side,
														  const std::vector<osg::Vec3d>& polygon,
														  double step,
														  double width,
														  int id)
		{
			osg::Vec3d dir = v;
			dir.z() = 0.0;
			dir.normalize();
			osg::Vec3d ortho = dir ^ osg::Z_AXIS;

			double height =
				std::min((polygon[0] - polygon[1]).length(), (polygon[0] - polygon[3]).length());

			Polygons slicePolygons;
			osg::Vec3d t = p + dir * height * 0.5;

			osg::Vec3d side0 = ortho;
			double side_len0 = width;

			// 裁剪的多边形在转角杆塔的边界范围上保持“贴合”
			{
				double cos0 = fabs(side * ortho);

				if (cos0 == 0.0)
					side0 = ortho;
				else
				{
					side0 = side;
					side_len0 = width / cos0;
				}

				// 保持同侧
				if (dir * side0 < 0.0)
					side0 = -side0;

				if (side0 * ortho < 0.0)
					ortho = -ortho;
			}

			double spanLength = step;
			int numSteps = std::max(1, (int)std::floor(spanLength / step));
			step = spanLength / (double)numSteps;

			for (double offset = 0.0; offset < spanLength; offset += step)
			{
				double len = (offset + step) >= spanLength ? (spanLength - offset) : step;

				osg::Vec3d q0 = t + dir * offset;
				osg::Vec3d q1 = t + dir * (offset + len);

				// 计算档范围边界
				std::vector<osg::Vec3d> polygon(4);
				polygon[0] = q0 + side0 * 0.5 * side_len0;
				polygon[1] = q1 + side0 * 0.5 * side_len0;
				polygon[2] = q1 - side0 * 0.5 * side_len0;
				polygon[3] = q0 - side0 * 0.5 * side_len0;

				slicePolygons.push_back(polygon);
			}

			std::vector<std::vector<int>> sliceIndices;
			sliceIndices.resize(slicePolygons.size());

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < (int)slicePolygons.size(); ++i)
			{
				const auto& polygon = slicePolygons.at(i);
				osg::BoundingBox bbox = getPolygonBound(polygon);

				for (size_t j = 0; j < grid.size(); ++j)
				{
					const auto& cell = grid.at(j);

					if (cell.GetSize() <= 0)
						continue;

					if (!(osg::maximum(cell.xMin(), bbox.xMin()) <=
							  osg::minimum(cell.xMax(), bbox.xMax()) &&
						  osg::maximum(cell.yMin(), bbox.yMin()) <=
							  osg::minimum(cell.yMax(), bbox.yMax())))
					{
						continue;
					}

					const auto& cellIndices = cell.GetIndices();

					for (size_t k = 0; k < cellIndices.size(); ++k)
					{
						const auto& p = _input->points[cellIndices[k]];

						if (pointInPolygon(polygon, p.x, p.y))
							sliceIndices[i].push_back(cellIndices[k]);
					}
				}
			}

#ifdef RENDER_SLCIE_BOUND
			int sliceID = 0;

			for (int i = 0; i < (int)slicePolygons.size(); ++i)
			{
				const auto& polygon = slicePolygons.at(i);

				// DEBUG: 绘制当边界
				if (!sliceIndices[i].empty())
				{
					RenderPowerlineMask(polygon,
										_input,
										StringPrintf("outsideSlicesBound_%d_%d", id, sliceID));
				}

				sliceID++;
			}
#endif

			return { id, dir, sliceIndices, slicePolygons };
		}

		void WireClassifier::ClassifyInSpan(SpanResult& result)
		{
			int minPts = _options.cluster_min_points;
			double T_r = _options.cluster_radius;
			double T_h = _options.cluster_th;
			double T_linearity = 0.98;
			double T_linearity2 = 0.90;

			CHECK(result.sliceInidces.size() == result.slicePolygons.size());
			
			// 每个切块内点云聚类，并分析类簇的线性性质
			for (size_t i = 0; i < result.sliceInidces.size(); ++i)
			{
				const auto& indices = result.sliceInidces.at(i);

				if (indices.empty())
					continue;

				std::vector<std::vector<int>> clusters;
				std::vector<std::vector<int>> clustersError, clustersOK;

#ifdef CLUSTERING_USING_VOXELS
				EuclideanVoxelCluster(_input,
									  indices,
									  0.05,
									  1,
									  std::numeric_limits<int>::max(),
									  clusters);
#else
				euclideanClusterSafe(*_input,
									 indices,
									 T_r,
									 1,
									 std::numeric_limits<int>::max(),
									 clusters);
#endif

				// 剔除非线性的点云簇
				for (size_t j = 0; j < clusters.size(); ++j)
				{
					const auto& cluster = clusters.at(j);

					if (cluster.empty())
						continue;

					double hmin = DBL_MAX;
					std::vector<osg::Vec3d> pts;

					for (size_t k = 0; k < cluster.size(); ++k)
					{
						const auto& p = _input->points[cluster[k]];
						hmin = std::min(hmin, (double)p.hag);

						osg::Vec3d v = osg::Vec3d(p.x, p.y, p.z);
						pts.push_back(v);
					}

					if (hmin < T_h)
						continue;

					// 计算 PCA 特征值 λ1 / (λ1 + λ2 + λ3), 其中 λ1 > λ2 > λ13
					double lamda1 = 0.0;
					double lamda2 = 0.0;
					double lamda3 = 0.0;
					osg::Vec3d v1, v2, v3;

					getEigenValues(pts, lamda1, lamda2, lamda3); // λ1 > λ2 > λ3
					getEigenVectors(pts, v1, v2, v3);
					v1.normalize();

					// 是否成功计算pca
					if (lamda1 == 0.0)
					{
						clustersError.push_back(cluster);
						continue;
					}

					// 是否呈水平分布(与垂直方向夹角小于 45° 则跳过)
					double VdotZ = fabs(v1 * osg::Z_AXIS);
					if (VdotZ > cos(osg::DegreesToRadians(45.0)))
						continue;

					// 是否满足线性特征
					double linearity = 1.0 - (lamda2 / lamda1);
					if (linearity < T_linearity)
					{
						/*if (linearity >= T_linearity2)
							clustersError.push_back(cluster);*/
						clustersError.push_back(cluster);
						continue;
					}

					if (cluster.size() < minPts)
					{
						clustersError.push_back(cluster);
						continue;
					}

					clustersOK.push_back(cluster);
				}

				// 切片内的电力线点
				std::vector<int> powerlinesOK, powerlinesFail;

				for (size_t j = 0; j < clustersOK.size(); ++j)
				{
					const auto& cluster = clustersOK.at(j);
					powerlinesOK.insert(powerlinesOK.end(), cluster.begin(), cluster.end());
				}

				// 根据已确定为电力线的点计算包围框，被完全包含在包围框内部的未分类类簇也可能为电力线
				osg::BoundingBox bbox;
				computeMinMax3D(*_input, powerlinesOK, bbox);
				bbox.expandBy(bbox.xMin() - T_r, bbox.yMin() - T_r, bbox.zMin() - T_r);
				bbox.expandBy(bbox.xMax() + T_r,
							  bbox.yMax() + T_r,
							  _input->bbox.zMax() /*bbox.zMax() + T_r*/);


				for (size_t j = 0; j < clustersError.size(); ++j)
				{
					size_t numContain = 0;
					const auto& cluster = clustersError.at(j);

					for (size_t k = 0; k < cluster.size(); ++k)
					{
						const auto& p = _input->points[cluster[k]];

						if (bbox.contains(osg::Vec3(p.x, p.y, p.z)))
							++numContain;
					}

					if (numContain == cluster.size())
					{
						powerlinesOK.insert(powerlinesOK.end(), cluster.begin(), cluster.end());
					}
					else
					{
						powerlinesFail.insert(powerlinesFail.end(), cluster.begin(), cluster.end());
					}
				}


				result.powerlines.insert(result.powerlines.end(),
										 powerlinesOK.begin(),
										 powerlinesOK.end());

				result.powerlinesFail.insert(result.powerlinesFail.end(),
											 powerlinesFail.begin(),
											 powerlinesFail.end());
			}
		}

		std::vector<bool> WireClassifier::ComputeLinearity(PointCloudViewPtr input,
														   const std::vector<int>& indices) 
		{
			double linearityRadius = _options.linearity_radius;
			double linearityThr = _options.linearity_threshold;

			std::vector<bool> linearity(input->size(), false);

			// 档内点云每个点的线性特征值计算
			std::vector<float> spanFeatures;
			computeCloudFeatures(*input, indices, linearityRadius, "linearity", spanFeatures);

			for (size_t i = 0; i < indices.size(); ++i)
			{
				if (spanFeatures[i] > linearityThr)
					linearity[indices[i]] = true;
			}

			return linearity;
		}

		void WireClassifier::ReclassifyWithLinearity(const std::vector<bool>& linearity,
													 SpanResult& result)
		{
			auto& powerlinesOK = result.powerlines;
			auto& powerlinesFail = result.powerlinesFail;

			for (auto iter = powerlinesFail.begin(); iter != powerlinesFail.end();)
			{
				auto index = *iter;

				if (linearity[index])
				{
					powerlinesOK.push_back(index);
					iter = powerlinesFail.erase(iter);
				}
				else
					++iter;
			}
		}

		PowerlineSpan WireClassifier::ReclassifyWithCurve(SpanResult& result, int curveMinPts)
		{
			double T_r = _options.cluster_radius;
			double T_linearity = 0.98;
			double tolerance = std::max(T_r * 0.5, 0.3);
			double curveExpandLen = _options.curve_expand_length;

			PowerlineSpan span; // 档内电力线回路和曲线信息

			if (result.powerlines.empty())
				return span;

			int label = 1;

			// 包含初次聚成功和失败的点云
			std::vector<int> indices;
			indices.insert(indices.end(), result.powerlines.begin(), result.powerlines.end());
			/*indices.insert(indices.end(),
						   result.powerlinesFail.begin(),
						   result.powerlinesFail.end());*/

			if (!indices.empty())
			{
				std::vector<std::vector<int>> clusters; // 档内点云整条电力线的聚类
				
#ifdef CLUSTERING_USING_VOXELS
				EuclideanVoxelCluster(_input,
									  indices,
									  0.05,
									  curveMinPts,
									  std::numeric_limits<int>::max(),
									  clusters);
#else
				// 线程安全欧式聚类
				euclideanClusterSafe(*_input,
									 indices,
									 tolerance,
									 curveMinPts,
									 std::numeric_limits<int>::max(),
									 clusters);
#endif

				// 排除非电力线类簇
				for (auto iter = clusters.begin(); iter != clusters.end();)
				{
					int numPowerline = 0;
					const std::vector<int>& cluster = *iter;

					for (size_t i = 0; i < cluster.size(); ++i)
					{
						const auto& p = _input->points[cluster[i]];

						if (p.label == ePowerline)
							++numPowerline;
					}

					if (numPowerline > 0.5 * cluster.size())
						++iter;
					else
						iter = clusters.erase(iter);
				}

				for (size_t i = 0; i < clusters.size(); ++i)
				{
					const auto& cluster = clusters.at(i);
				
					osg::Vec4ub c =
						ColorManager::colorTable[eGround +
												 label % (eNumOfClassification - eGround)];

					// 拟合单股导线的曲线
					auto curve = PowerlineCurveFitting(_input,
													   cluster,
													   curveExpandLen,
													   osg::Vec4((float)c.r() / 255.f,
																 (float)c.g() / 255.f,
																 (float)c.b() / 255.f,
																 1.0f)
#ifdef RENDER_FITTING_CURVE
														   ,
													   true
#endif
					);

					// 添加单股导线的曲线到当前档
					span.push_back(curve);

					label++;
				}

				auto& powerlinesFail = result.powerlinesFail;

				// 补充电力线点云
				for (size_t i = 0; i < span.size(); ++i)
				{
					const auto& curve = span.at(i);

					for (int j = (int)powerlinesFail.size() - 1; j >= 0; --j)
					{
						auto& p = _input->points[powerlinesFail[j]];

						if (p.label == ePowerline)
							continue;

						if (curve->NolimitDistanceTo(osg::Vec3d(p.x, p.y, p.z)) < T_r)
						{
							result.powerlines.push_back(powerlinesFail[j]);
							powerlinesFail.erase(powerlinesFail.begin() + j);
						}
					}
				}
			}

			// 剩余部分再次检测是否存在线性系数较高的点云类簇
			if (!result.powerlinesFail.empty())
			{
				int minPts = _options.cluster_min_points;
				const auto& indices = result.powerlinesFail;

				std::vector<std::vector<int>> clusters;

#ifdef CLUSTERING_USING_VOXELS
				EuclideanVoxelCluster(_input,
									  indices,
									  0.05,
									  1,
									  std::numeric_limits<int>::max(),
									  clusters);
#else
				// 线程安全欧式聚类
				euclideanClusterSafe(*_input,
									 indices,
									 tolerance,
									 1,
									 std::numeric_limits<int>::max(),
									 clusters);
#endif

				// 剔除非线性的点云簇
				for (size_t i = 0; i < clusters.size(); ++i)
				{
					const auto& cluster = clusters.at(i);

					if (cluster.size() < 2)
						continue;

					std::vector<osg::Vec3d> pts;

					for (size_t k = 0; k < cluster.size(); ++k)
					{
						const auto& p = _input->points[cluster[k]];
						osg::Vec3d v = osg::Vec3d(p.x, p.y, p.z);
						pts.push_back(v);
					}

					// 计算 PCA 特征值 λ1 / (λ1 + λ2 + λ3), 其中 λ1 > λ2 > λ13
					double lamda1 = 0.0;
					double lamda2 = 0.0;
					double lamda3 = 0.0;
					osg::Vec3d v1, v2, v3;

					getEigenValues(pts, lamda1, lamda2, lamda3); // λ1 > λ2 > λ3
					getEigenVectors(pts, v1, v2, v3);
					v1.normalize();

					// 是否成功计算pca
					if (lamda1 == 0.0)
						continue;

					// 是否呈水平分布(与垂直方向夹角小于 45° 则跳过)
					double VdotZ = fabs(v1 * osg::Z_AXIS);
					if (VdotZ > cos(osg::DegreesToRadians(45.0)))
						continue;

					// 是否满足线性特征
					double linearity = 1.0 - (lamda2 / lamda1);
					if (linearity < T_linearity)
						continue;

					if (cluster.size() < minPts)
						continue;

					result.powerlines.insert(result.powerlines.end(),
											 cluster.begin(),
											 cluster.end());
				}
			}

			return span;
		}

		void WireClassifier::ReclassifyWithLinearityAndCurve(const std::vector<bool>& linearity,
															 PointCloudViewPtr input,
															 int curveMinPts,
															 std::vector<int>& indices,
															 std::vector<int>& curveIndices)
		{
			CHECK(linearity.size() == input->size());

			double T_r = _options.cluster_radius;
			double T_h = _options.cluster_th;
			double T_linearity = 0.98;
			double tolerance = std::max(T_r * 0.5, 0.3);
			double curveExpandLen = 0.0;

			std::vector<int> linearIndices;

			for (size_t i = 0; i < indices.size(); ++i)
			{
				if (linearity[indices[i]])
					linearIndices.push_back(indices[i]);
			}

			if (linearIndices.empty())
				return;

			int label = 1;
			std::vector<std::vector<int>> clusters; // 档内点云整条电力线的聚类

#ifdef CLUSTERING_USING_VOXELS
			EuclideanVoxelCluster(_input,
								  indices,
								  0.05,
								  curveMinPts,
								  std::numeric_limits<int>::max(),
								  clusters);
#else
			// 线程安全欧式聚类
			euclideanClusterSafe(*_input,
								 linearIndices,
								 tolerance,
								 curveMinPts,
								 std::numeric_limits<int>::max(),
								 clusters);
#endif

			std::vector<PowerlineCurvePtr> curves;

			for (size_t i = 0; i < clusters.size(); ++i)
			{
				const auto& cluster = clusters.at(i);

				double hmin = DBL_MAX;
				std::vector<osg::Vec3d> pts;

				for (size_t k = 0; k < cluster.size(); ++k)
				{
					const auto& p = _input->points[cluster[k]];
					hmin = std::min(hmin, (double)p.hag);

					osg::Vec3d v = osg::Vec3d(p.x, p.y, p.z);
					pts.push_back(v);
				}

				if (hmin < T_h)
					continue;

				// 计算 PCA 特征值 λ1 / (λ1 + λ2 + λ3), 其中 λ1 > λ2 > λ13
				double lamda1 = 0.0;
				double lamda2 = 0.0;
				double lamda3 = 0.0;
				osg::Vec3d v1, v2, v3;

				getEigenValues(pts, lamda1, lamda2, lamda3); // λ1 > λ2 > λ3

				double linearity = 1.0 - (lamda2 / lamda1);

				if (linearity < T_linearity)
					continue;

				osg::Vec4ub c =
					ColorManager::colorTable[eGround + label % (eNumOfClassification - eGround)];

				// 拟合单股导线的曲线
				auto curve = PowerlineCurveFitting(_input,
												   cluster,
												   curveExpandLen,
												   osg::Vec4((float)c.r() / 255.f,
															 (float)c.g() / 255.f,
															 (float)c.b() / 255.f,
															 1.0f)
#ifdef RENDER_FITTING_CURVE
													   ,
												   true
#endif
				);

				curves.push_back(curve);
				label++;
			}

			for (size_t i = 0; i < curves.size(); ++i)
			{
				const auto& curve = curves.at(i);

				for (int j = (int)indices.size() - 1; j >= 0; --j)
				{
					const auto& p = _input->points[indices[j]];

					if (p.label == ePowerline)
						continue;

					if (curve->DistanceTo(osg::Vec3d(p.x, p.y, p.z)) < T_r)
					{
						curveIndices.push_back(indices[j]);
						indices.erase(indices.begin() + j);
					}
				}
			}
		}

		void WireClassifier::ReclassifyPoleFromUnclassified(
			std::vector<Trunk>& trunks,
			std::vector<std::vector<int>>& trunkClusters,
			const std::vector<int>& indices,
			std::vector<std::vector<int>>& poleIndicesList)
		{
			double poleRadius = 0.3;
			double head_length = 3.0;
			double head_radius = 1.5;

			std::vector<std::vector<Trunk>> poleList;

			for (size_t i = 0; i < trunkClusters.size(); ++i)
			{
				auto& trunkCluster = trunkClusters.at(i);

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

				if (layerDiff > 6)
				{
					std::vector<Trunk> poleTrunks;

					for (size_t k = 0; k < trunkCluster.size(); ++k)
						poleTrunks.push_back(trunks[trunkCluster[k]]);

					poleList.push_back(poleTrunks);
				}
			}

			std::vector<int> tempIndices = indices;
			std::vector<osg::Vec3d> centroids;

			// 电杆主体点云
			for (size_t i = 0; i < poleList.size(); ++i)
			{
				const auto& poleTrunks = poleList[i];

				std::vector<int> poleIndices;
				osg::Vec3 center, dir;
				GetMainVector(_input, poleTrunks, center, dir);

#ifdef RENDER_TRUNKS
				std::vector<osg::Vec3d> path = { center -dir * 6.0, center + dir * 6.0 };
				RenderLinePath(StringPrintf("span_pole_pca_v_%d", i),
							   path,
							   _input,
							   osg::Vec4(1.f, 1.f, 0.f, 1.f));
#endif

				for (int j = (int)tempIndices.size() - 1; j >= 0; j--)
				{
					const auto& p = _input->points[tempIndices[j]];
					osg::Vec3 P(p.x, p.y, p.z);
					osg::Vec3 Q = center;
					osg::Vec3 v = dir;
					v.normalize();

					double dist = ((P - Q) ^ v).length();
					/*double dist = std::sqrt((p.x - centroid.x()) * (p.x - centroid.x()) +
											(p.y - centroid.y()) * (p.y - centroid.y()));*/

					if (dist < poleRadius)
					{
						poleIndices.push_back(tempIndices[j]);
						tempIndices.erase(tempIndices.begin() + j);
					}
				}

				centroids.push_back(center);
				poleIndicesList.push_back(poleIndices);
			}

			// 电杆头部点云
			for (size_t i = 0; i < poleIndicesList.size(); ++i)
			{
				const auto& centroid = centroids[i];
				auto& poleIndices = poleIndicesList[i];
					
				// 统计电杆高度
				float zmax = -FLT_MAX;
				
				for (size_t j = 0; j < poleIndices.size(); ++j)
				{
					const auto& p = _input->points[poleIndices[j]];
					zmax = std::max(zmax, p.z);
				}

				// 将在杆塔横担半径范围内的，剩余的未分类点归纳到所属杆塔中去
				float lower = std::max(0.0f, zmax - (float)head_length);

				for (int j = (int)tempIndices.size() - 1; j >= 0; j--)
				{
					const auto& p = _input->points[tempIndices[j]];
					double dist = std::sqrt((p.x - centroid.x()) * (p.x - centroid.x()) +
											(p.y - centroid.y()) * (p.y - centroid.y()));

					if (p.z > lower &&  dist < head_radius)
					{
						poleIndices.push_back(tempIndices[j]);
						tempIndices.erase(tempIndices.begin() + j);
					}
				}
			}
		}

		std::vector<int> WireClassifier::GetUnclassified(SpanResult& result)
		{
			std::vector<int> unclassified;

			for (size_t i = 0; i < result.sliceInidces.size(); ++i)
			{
				const auto& slice = result.sliceInidces[i];

				for (size_t j = 0; j < slice.size(); ++j)
				{
					const auto& p = _input->points[slice[j]];

					if (p.label == eUnclassified)
						unclassified.push_back(slice[j]);
				}
			}

			return unclassified;
		}

	}
}