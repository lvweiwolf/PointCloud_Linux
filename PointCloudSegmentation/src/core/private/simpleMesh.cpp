#include <src/core/private/simpleMesh.h>
#include <src/utils/logging.h>

#include <osg/Geode>
#include <osg/Transform>
#include <osg/NodeVisitor>
#include <osg/TriangleFunctor>
#include <osg/Node>

namespace d3s {
	namespace pcs {

		struct CollectTriangleCache
		{
			osg::BoundingBox bbox;
			std::vector<osg::Vec3d> vertices;

			std::vector<VerticesIndexes> triangles;
			std::map<osg::Vec3d, size_t> vertex_index_map;
		};

		struct CollectTriangle
		{
			CollectTriangle() {}

			inline void operator()(const osg::Vec3d& v1,
								   const osg::Vec3d& v2,
								   const osg::Vec3d& v3,
								   bool treatVertexDataAsTemporary)
			{
				if (!cache)
					return;

				auto& bbox = cache->bbox;
				auto& vertices = cache->vertices;
				auto& triangles = cache->triangles;
				auto& indexmap = cache->vertex_index_map;

				osg::Vec3d v[3] = { v1 * m, v2 * m, v3 * m };

				for (int i = 0; i < 3; ++i)
				{
					auto iter = indexmap.find(v[i]);

					if (iter == indexmap.end())
					{
						indexmap[v[i]] = vertices.size();
						vertices.push_back(v[i]);
						bbox.expandBy(v[i]);
					}
				}

				triangles.push_back({ indexmap[v[0]], indexmap[v[1]], indexmap[v[2]] });
			}

			osg::Matrix m;
			CollectTriangleCache* cache = nullptr;
		};

		struct CollectTriangleVisitor : public osg::NodeVisitor
		{
			CollectTriangleVisitor() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}

			void apply(osg::Transform& transform)
			{
				osg::Matrix matrix;
				if (!_matrixStack.empty())
					matrix = _matrixStack.back();

				transform.computeLocalToWorldMatrix(matrix, this);

				pushMatrix(matrix);

				traverse(transform);

				popMatrix();
			}

			void apply(osg::Geode& geode)
			{
				osg::Matrix matrix;

				if (!_matrixStack.empty())
					matrix = _matrixStack.back();

				for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
				{
					osg::TriangleFunctor<CollectTriangle> triangleCollector;
					triangleCollector.cache = &_cache;
					triangleCollector.m = matrix;

					geode.getDrawable(i)->accept(triangleCollector);
				}
			}

			inline void pushMatrix(osg::Matrix& matrix) { _matrixStack.push_back(matrix); }

			inline void popMatrix() { _matrixStack.pop_back(); }

			std::vector<osg::Matrix> _matrixStack;
			CollectTriangleCache _cache;
		};


		SimpleMesh::SimpleMesh(osg::ref_ptr<osg::Node> meshNode)
		{
			if (meshNode.valid())
			{
				CollectTriangleVisitor ctv;
				meshNode->accept(ctv);

				_bbox = ctv._cache.bbox;
				_vertices = ctv._cache.vertices;
				_triangles = ctv._cache.triangles;

				computeNormals();
			}
		}

		SimpleMesh::~SimpleMesh() {}

		size_t SimpleMesh::numTriangles() const { return _triangles.size(); }

		size_t SimpleMesh::numVertices() const { return _vertices.size(); }

		GenericTriangle* SimpleMesh::getTriangle(size_t index)
		{
			CHECK(index >= 0 && index < _triangles.size());

			const auto& ti = _triangles.at(index);
			_dummyTriangle.A = _vertices.at(ti.i1);
			_dummyTriangle.B = _vertices.at(ti.i2);
			_dummyTriangle.C = _vertices.at(ti.i3);

			return &_dummyTriangle;
		}

		void SimpleMesh::getTriangle(size_t index, size_t& v1, size_t& v2, size_t& v3)
		{
			CHECK(index >= 0 && index < _triangles.size());
			const auto& ti = _triangles.at(index);

			v1 = ti.i1;
			v2 = ti.i2;
			v3 = ti.i3;
		}

		const CCVector3* SimpleMesh::getVertex(size_t index)
		{
			CHECK(index < _vertices.size());
			return &_vertices.at(index);
		}

		void SimpleMesh::getVertex(size_t index, CCVector3& P) const
		{
			CHECK(index < _vertices.size());
			P = _vertices.at(index);
		}

		bool SimpleMesh::normalsAvailable() const { return _vertices.size() == _normals.size(); }

		bool SimpleMesh::interpolateNormals(size_t index, const CCVector3& P, CCVector3& N)
		{
			CHECK(index >= 0 && index < _vertices.size());

			if (index >= _triangles.size())
			{
				// index out of range
				return false;
			}

			const auto& tri = _triangles[index];

			// intepolation weights
			CCVector3 weights;

			{
				CCVector3 A = _vertices[tri.i1];
				CCVector3 B = _vertices[tri.i2];
				CCVector3 C = _vertices[tri.i3];

				// 重心坐标系权重
				weights.x() = std::sqrt(((P - B) ^ (C - B)).length2()) /*/2*/;
				weights.y() = std::sqrt(((P - C) ^ (A - C)).length2()) /*/2*/;
				weights.z() = std::sqrt(((P - A) ^ (B - A)).length2()) /*/2*/;

				// 正则化
				double sum = weights.x() + weights.y() + weights.z();
				weights /= sum;
			}

			// 法线差值
			CCVector3 Nd(0, 0, 0);

			{
				const auto& N1 = _normals[tri.i1];
				Nd += N1 * weights[0];

				const auto& N2 = _normals[tri.i2];
				Nd += N2 * weights[1];

				const auto& N3 = _normals[tri.i3];
				Nd += N3 * weights[2];

				Nd.normalize();
			}

			N = Nd;

			return true;
		}

		void SimpleMesh::computeNormals()
		{
			_normals.resize(_vertices.size(), CCVector3(0, 0, 0));

			for (size_t fi = 0; fi < _triangles.size(); ++fi)
			{
				const auto& p1 = _vertices[_triangles[fi].i1];
				const auto& p2 = _vertices[_triangles[fi].i2];
				const auto& p3 = _vertices[_triangles[fi].i3];

				CCVector3 faceNormal = (p2 - p1) ^ (p3 - p1);
				_normals[_triangles[fi].i1] += faceNormal;
				_normals[_triangles[fi].i2] += faceNormal;
				_normals[_triangles[fi].i3] += faceNormal;
			}

			for (size_t vi = 0; vi < _normals.size(); ++vi)
			{
				_normals[vi].normalize();
			}
		}

	}
}