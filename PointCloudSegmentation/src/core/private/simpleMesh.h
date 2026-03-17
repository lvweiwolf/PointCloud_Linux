//////////////////////////////////////////////////////////////////////
// 文件名称：simpleMesh.h
// 功能描述：简易网格类
// 创建标识：吕伟	2022/12/5
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef SIMPLE_MESH_H_
#define SIMPLE_MESH_H_

#include <src/algorithm/mesh.h>

#include <osg/BoundingBox>
#include <osg/Node>

#include <vector>

namespace d3s {
	namespace pcs {

		class SimpleTriangle : public GenericTriangle
		{

		public:
			SimpleTriangle() : A(0, 0, 0), B(0, 0, 0), C(0, 0, 0) {}

			SimpleTriangle(const CCVector3& A_, const CCVector3& B_, const CCVector3& C_)
				: A(A_), B(B_), C(C_)
			{
			}

			virtual const CCVector3* getA() const override { return &A; }
			virtual const CCVector3* getB() const override { return &B; };
			virtual const CCVector3* getC() const override { return &C; };

			CCVector3 A;
			CCVector3 B;
			CCVector3 C;
		};

		class SimpleMesh : public GenericMesh
		{
		public:
			SimpleMesh(osg::ref_ptr<osg::Node> meshNode);

			~SimpleMesh() override;
			
			/**
			 *  @brief    网格三角面数量
			 *
			 *  @return   unsigned
			 */
			size_t numTriangles() const override;

			/**
			 *  @brief    网格顶点数量
			 *
			 *  @return   size_t
			 */
			size_t numVertices() const override;

			/**
			 *  @brief    获得网格三角面
			 *
			 *  @param    size_t index					三角面索引
			 *
			 *  @return   d3s::pcs::GenericTriangle*
			 */
			GenericTriangle* getTriangle(size_t index) override;

			void getTriangle(size_t index, size_t& v1, size_t& v2, size_t& v3) override;

			/**
			 *  @brief    获得网格顶点
			 *
			 *  @param    size_t index
			 *
			 *  @return   const d3s::pcs::CCVector3*
			 */
			const CCVector3* getVertex(size_t index) override;

			void getVertex(size_t index, CCVector3& P) const override;

			bool normalsAvailable() const override;

			/**
			 *  @brief    法线插值
			 *
			 *  @param    size_t index					三角面索引
			 *  @param    const CCVector3 & P			三角面内的点
			 *  @param    CCVector3 & N					差值的法线
			 *
			 *  @return   bool
			 */
			bool interpolateNormals(size_t index, const CCVector3& P, CCVector3& N) override;

			/**
			 *  @brief    计算网格法线
			 *
			 *  @return   void
			 */
			void computeNormals();

		protected:
			using TriangleIndexesContainer = std::vector<VerticesIndexes>;
			using VectorContainer = std::vector<CCVector3>;

			TriangleIndexesContainer _triangles; // 网格三角形
			VectorContainer _vertices;			 // 网格顶点
			VectorContainer _normals;			 // 顶点法线

			SimpleTriangle _dummyTriangle;
			osg::BoundingBox _bbox;
		};

	}
}

#endif // SIMPLE_MESH_H_
