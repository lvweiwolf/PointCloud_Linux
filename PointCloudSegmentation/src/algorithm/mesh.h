//////////////////////////////////////////////////////////////////////
// 文件名称：mesh.h
// 功能描述：模型网格
// 创建标识：吕伟	2022/12/5
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef MESH_H_
#define MESH_H_

#include <osg/Vec3d>
#include <array>
#include <algorithm>
namespace d3s {
	namespace pcs {

		typedef osg::Vec3d CCVector3;

		struct VerticesIndexes
		{
			union
			{
				struct
				{
					size_t i1, i2, i3;
				};

				size_t i[3];
			};

			VerticesIndexes() : i1(0), i2(0), i3(0) {}

			VerticesIndexes(size_t i1_, size_t i2_, size_t i3_) : i1(i1_), i2(i2_), i3(i3_) {}

			friend bool operator==(const VerticesIndexes& a, const VerticesIndexes& b);
		};


		inline bool operator==(const VerticesIndexes& a, const VerticesIndexes& b)
		{
			std::array<size_t, 3> aa{ { a.i1, a.i2, a.i3 } };
			std::array<size_t, 3> bb{ { b.i1, b.i2, b.i3 } };
			std::sort(aa.begin(), aa.end());
			std::sort(bb.begin(), bb.end());
			return aa == bb;
		}

		class GenericTriangle
		{
		public:
			virtual ~GenericTriangle() {};

			/**
			 *  @brief    获得三角形第一个顶点
			 *
			 *  @return   const CCVector3*
			 */
			virtual const CCVector3* getA() const = 0;

			/**
			 *  @brief    获得三角形第二个顶点
			 *
			 *  @return   const CCVector3*
			 */
			virtual const CCVector3* getB() const = 0;

			/**
			 *  @brief    获得三角形第三个顶点
			 *
			 *  @return   const CCVector3*
			 */
			virtual const CCVector3* getC() const = 0;
		};

		class GenericMesh
		{
		public:
			virtual ~GenericMesh() {};

			/**
			 *  @brief    网格三角面数量
			 *
			 *  @return   unsigned
			 */
			virtual size_t numTriangles() const = 0;

			/**
			 *  @brief    网格顶点数量
			 *
			 *  @return   size_t
			 */
			virtual size_t numVertices() const = 0;

			/**
			 *  @brief    获得网格三角面
			 *
			 *  @param    size_t index					三角面索引
			 *
			 *  @return   d3s::pcs::GenericTriangle*
			 */
			virtual GenericTriangle* getTriangle(size_t index) = 0;


			virtual void getTriangle(size_t index, size_t& v1, size_t& v2, size_t& v3) = 0;

			/**
			 *  @brief    获得网格顶点
			 *
			 *  @param    size_t index
			 *
			 *  @return   const d3s::pcs::CCVector3*
			 */
			virtual const CCVector3* getVertex(size_t index) = 0;

			virtual void getVertex(size_t index, CCVector3& P) const = 0;

			virtual bool normalsAvailable() const { return false; }

			/**
			 *  @brief    法线差值
			 *
			 *  @param    size_t index					三角面索引
			 *  @param    const CCVector3 & P			三角面内的点
			 *  @param    CCVector3 & N					差值的法线
			 *
			 *  @return   bool
			 */
			virtual bool interpolateNormals(size_t index, const CCVector3& P, CCVector3& N)
			{
				(void)index;
				(void)P;
				(void)N;
				return false;
			}
		};
	}
}

#endif // MESH_H_
