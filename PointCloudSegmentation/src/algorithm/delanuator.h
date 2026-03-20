/////////////////////////////////////////////////////////////////////
// 文件名称：simple_delanuator.h
// 功能描述：简易 Delaunay 2D 三角剖分，用于生成TIN
// 创建标识：吕伟	2022/4/8
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef DELAUNATOR_H_
#define DELAUNATOR_H_

#include <stdexcept>
#include <vector>
#include <limits>

namespace d3s {
	namespace pcs {

		constexpr std::size_t INVALID_INDEX = (std::numeric_limits<std::size_t>::max)();

		// 2D 顶点
		class Point
		{
		public:
			Point(double x, double y) : m_x(x), m_y(y) {}
			Point() : m_x(0), m_y(0) {}

			double x() const { return m_x; }

			double y() const { return m_y; }

		private:
			double m_x;
			double m_y;
		};

		class delaunator_error : public std::runtime_error
		{
		public:
			explicit delaunator_error(const std::string& message)
				: std::runtime_error(message.c_str())
			{
			}
		};


		// 2D Delaunay三角剖分
		class Delaunator
		{

		public:
			std::vector<double> const& coords;
			std::vector<std::size_t> triangles;
			std::vector<std::size_t> halfedges;
			std::vector<std::size_t> hull_prev;
			std::vector<std::size_t> hull_next;
			std::vector<std::size_t> hull_tri;
			std::size_t hull_start;

			Delaunator(std::vector<double> const& in_coords);

			double get_hull_area();

		private:
			std::vector<std::size_t> m_hash;
			Point m_center;
			std::size_t m_hash_size;
			std::vector<std::size_t> m_edge_stack;

			std::size_t legalize(std::size_t a);
			std::size_t hash_key(double x, double y) const;
			std::size_t add_triangle(std::size_t i0,
									 std::size_t i1,
									 std::size_t i2,
									 std::size_t a,
									 std::size_t b,
									 std::size_t c);
			void link(std::size_t a, std::size_t b);
		};

	}
}


#endif // DELAUNATOR_H_