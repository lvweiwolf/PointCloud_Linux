//stdafx.h
#include "meshProcess.h"

#include <random>
#include "../../algorithm/vcgmesh.h"

#include "simpleMesh.h"
#include "../pointTypes.h"
#include <osg/BoundingBox>
#include "../../utils/logging.h"
namespace d3s {
	namespace pcs {

		double computeMeshArea(GenericMesh* mesh)
		{
			CHECK(mesh);

			if (!mesh)
				return -1.0;

			// 总面积
			double S = 0.0;
			size_t triCount = mesh->numTriangles();

			for (size_t fi = 0; fi < triCount; ++fi)
			{
				GenericTriangle* tri = mesh->getTriangle(fi);

				// 三角形顶点
				const CCVector3* O = tri->getA();
				const CCVector3* A = tri->getB();
				const CCVector3* B = tri->getC();

				// 计算三角形的面积(海伦公式)
				CCVector3 OA = *A - *O;
				CCVector3 OB = *B - *O;
				S += (OA ^ OB).length();
			}

			return S / 2.0;
		}

		PointCloudViewPtr samplePointsOnMesh(GenericMesh* mesh,
											 double sampleDensity,
											 NoiseFunc nosieFunc,
											 std::vector<size_t>* triIndices)
		{
			if (!mesh)
				return nullptr;

			double S = computeMeshArea(mesh);

			size_t theoreticNumberOfPoints = (size_t)(std::ceil(S * sampleDensity));

			return samplePointsOnMesh(mesh,
									  sampleDensity,
									  theoreticNumberOfPoints,
									  nosieFunc,
									  triIndices);
		}

		PointCloudViewPtr samplePointsOnMesh(GenericMesh* mesh,
											 size_t numberOfPoints,
											 NoiseFunc nosieFunc,
											 std::vector<size_t>* triIndices)
		{
			if (!mesh)
				return nullptr;

			double S = computeMeshArea(mesh);

			if (S < std::numeric_limits<double>::epsilon())
				return nullptr;

			double samplingDensity = numberOfPoints / S;

			return samplePointsOnMesh(mesh, samplingDensity, numberOfPoints, nosieFunc, triIndices);
		}

		PointCloudViewPtr samplePointsOnMesh(GenericMesh* mesh,
											 double samplingDensity,
											 size_t theoreticNumberOfPoints,
											 NoiseFunc nosieFunc,
											 std::vector<size_t>* triIndices)
		{
			if (theoreticNumberOfPoints < 1)
				return nullptr;

			CHECK(mesh);
			size_t triCount = (mesh ? mesh->numTriangles() : 0);

			if (triCount == 0)
				return nullptr;

			PointCloudViewPtr sampledCloud = new PointCloudView<PointPCLH>();

			try
			{
				sampledCloud->points.reserve(theoreticNumberOfPoints);
			}
			catch (const std::bad_alloc&)
			{
				// 内存不足
				delete sampledCloud;
				return nullptr;
			}

			if (triIndices)
			{
				triIndices->clear();

				try
				{
					triIndices->reserve(theoreticNumberOfPoints);
				}
				catch (const std::bad_alloc&)
				{
					// 内存不足
					delete sampledCloud;
					return nullptr;
				}
			}

			size_t addedPoints = 0;
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<double> dist(0, 1); // 随机分布

			for (size_t fi = 0; fi < triCount; ++fi)
			{
				const GenericTriangle* tri = mesh->getTriangle(fi);

				// 三角形顶点(OAB)
				const CCVector3* O = tri->getA();
				const CCVector3* A = tri->getB();
				const CCVector3* B = tri->getC();

				// 三角线边(OA 和 OB)
				CCVector3 u = *A - *O;
				CCVector3 v = *B - *O;

				// 计算（两倍）三角形面积
				CCVector3 N = u ^ v;
				double S = N.length() / 2.0;

				// 推导出要在该面上生成的点数
				double fPointsToAdd = S * samplingDensity;
				size_t pointsToAdd = (size_t)(fPointsToAdd);

				// 照顾小数部分
				double fracPart = fPointsToAdd - (double)pointsToAdd;

				if (fracPart > 0)
				{
					// 我们添加一个概率与其（相对）面积相同的点
					if (dist(gen) < fracPart)
						pointsToAdd += 1;
				}

				if (pointsToAdd)
				{
					if (addedPoints + pointsToAdd >= theoreticNumberOfPoints)
					{
						theoreticNumberOfPoints += pointsToAdd;

						try
						{
							sampledCloud->points.reserve(theoreticNumberOfPoints);
						}
						catch (const std::bad_alloc&)
						{
							// 内存不足
							delete sampledCloud;
							sampledCloud = nullptr;

							if (triIndices)
								triIndices->resize(0);

							break;
						}

						// 为三角形索引保留内存
						if (triIndices && triIndices->capacity() < theoreticNumberOfPoints)
						{
							try
							{
								triIndices->reserve(theoreticNumberOfPoints);
							}
							catch (const std::bad_alloc&)
							{
								// 内存不足
								delete sampledCloud;
								sampledCloud = nullptr;

								if (triIndices)
									triIndices->resize(0);

								break;
							}
						}
					}

					for (size_t i = 0; i < pointsToAdd; ++i)
					{
						// 三角形内部随机生成点
						double x = dist(gen);
						double y = dist(gen);

						// 生成的点是否位于（AB）的右侧
						if (x + y > 1.0)
						{
							x = 1.0 - x;
							y = 1.0 - y;
						}

						CCVector3 P = (*O) + u * x + v * y;
						PointPCLH point;
						point.x = P.x();
						point.y = P.y();
						point.z = P.z();

						if (nosieFunc)
						{
							point.x += nosieFunc();
							point.y += nosieFunc();
							point.z += nosieFunc();
						}

						sampledCloud->points.push_back(point);

						if (triIndices)
							triIndices->push_back(fi);

						++addedPoints;
					}
				}

				// TODO: 进度条
			}

			if (sampledCloud)
			{
				if (addedPoints)
				{
					// 应该始终满足条件: addedPoints＜theory NumberOfPoints
					sampledCloud->points.reserve(addedPoints);

					if (triIndices)
						triIndices->resize(addedPoints);
				}
				else
				{
					if (triIndices)
						triIndices->resize(0);
				}
			}

			return sampledCloud;
		}

		void ClipMesh(GenericMesh* mesh, osg::BoundingBox clipBBox, std::string savepath)
		{
			if (!mesh)
				return;

			// 转换网格
			CMeshO vcgmesh;
			UpdateDataMask(vcgmesh, MM_WEDGTEXCOORD);

			size_t numVertices = mesh->numVertices();
			size_t numTriangles = mesh->numTriangles();

			for (size_t i = 0; i < numVertices; ++i)
			{
				const auto& vp = mesh->getVertex(i);
				Point3m p(vp->x(), vp->y(), vp->z());
				vcg::tri::Allocator<CMeshO>::AddVertex(vcgmesh, p); // 添加顶点
			}

			// 添加面
			for (size_t i = 0; i < numTriangles; ++i)
			{
				size_t v1, v2, v3;
				mesh->getTriangle(i, v1, v2, v3);
				auto fi = vcg::tri::Allocator<CMeshO>::AddFace(vcgmesh, v1, v2, v3);

				if (fi->IsWedgeTexCoordEnabled())
				{
					for (int v = 0; v < 3; ++v)
					{
						fi->WT(v).u() = 0.0;
						fi->WT(v).v() = 0.0;
						fi->WT(v).n() = 0;
					}
				}
			}

			Point3m min(clipBBox.xMin(), clipBBox.yMin(), clipBBox.zMin());
			Point3m max(clipBBox.xMax(), clipBBox.yMax(), clipBBox.zMax());

			if (!vcgmesh.ClipExtents(min, max))
			{
				PCS_ERROR("[ClipMesh] 裁剪网格失败.");
				return;
			}

			auto result = vcg::tri::io::ExporterSTL<CMeshO>::Save(vcgmesh, savepath.c_str());
		}
	}
}