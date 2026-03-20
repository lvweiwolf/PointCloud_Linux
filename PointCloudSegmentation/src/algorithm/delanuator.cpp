
#include <src/algorithm/delanuator.h>

#include <cmath>
#include <algorithm>

namespace d3s {
	namespace pcs {

		inline size_t fast_mod(const size_t i, const size_t c) { return i >= c ? i % c : i; }

		// 累积错误更少的处理
		inline double sum(const std::vector<double>& x)
		{
			double sum = x[0];
			double err = 0.0;

			for (size_t i = 1; i < x.size(); i++)
			{
				const double k = x[i];
				const double m = sum + k;
				err += std::fabs(sum) >= std::fabs(k) ? sum - m + k : k - m + sum;
				sum = m;
			}
			return sum + err;
		}

		inline double dist(const double ax, const double ay, const double bx, const double by)
		{
			const double dx = ax - bx;
			const double dy = ay - by;
			return dx * dx + dy * dy;
		}

		inline double circumradius(const double ax,
								   const double ay,
								   const double bx,
								   const double by,
								   const double cx,
								   const double cy)
		{
			const double dx = bx - ax;
			const double dy = by - ay;
			const double ex = cx - ax;
			const double ey = cy - ay;

			const double bl = dx * dx + dy * dy;
			const double cl = ex * ex + ey * ey;
			const double d = dx * ey - dy * ex;

			const double x = (ey * bl - dy * cl) * 0.5 / d;
			const double y = (dx * cl - ex * bl) * 0.5 / d;

			if ((bl > 0.0 || bl < 0.0) && (cl > 0.0 || cl < 0.0) && (d > 0.0 || d < 0.0))
			{
				return x * x + y * y;
			}
			else
			{
				return (std::numeric_limits<double>::max)();
			}
		}

		inline bool orient(const double px,
						   const double py,
						   const double qx,
						   const double qy,
						   const double rx,
						   const double ry)
		{
			return (qy - py) * (rx - qx) - (qx - px) * (ry - qy) < 0.0;
		}

		inline Point circumcenter(const double ax,
								  const double ay,
								  const double bx,
								  const double by,
								  const double cx,
								  const double cy)
		{
			const double dx = bx - ax;
			const double dy = by - ay;
			const double ex = cx - ax;
			const double ey = cy - ay;

			const double bl = dx * dx + dy * dy;
			const double cl = ex * ex + ey * ey;
			const double d = dx * ey - dy * ex;

			const double x = ax + (ey * bl - dy * cl) * 0.5 / d;
			const double y = ay + (dx * cl - ex * bl) * 0.5 / d;

			return Point(x, y);
		}


		inline bool in_circle(const double ax,
							  const double ay,
							  const double bx,
							  const double by,
							  const double cx,
							  const double cy,
							  const double px,
							  const double py)
		{
			const double dx = ax - px;
			const double dy = ay - py;
			const double ex = bx - px;
			const double ey = by - py;
			const double fx = cx - px;
			const double fy = cy - py;

			const double ap = dx * dx + dy * dy;
			const double bp = ex * ex + ey * ey;
			const double cp = fx * fx + fy * fy;

			return (dx * (ey * cp - bp * fy) - dy * (ex * cp - bp * fx) +
					ap * (ex * fy - ey * fx)) < 0.0;
		}

		constexpr double epsilon = std::numeric_limits<double>::epsilon();

		inline bool check_pts_equal(double x1, double y1, double x2, double y2)
		{
			return std::fabs(x1 - x2) <= epsilon && std::fabs(y1 - y2) <= epsilon;
		}

		// 随着实际角度单调增加，不需要复杂的三角函数
		inline double pseudo_angle(const double dx, const double dy)
		{
			const double p = dx / (std::abs(dx) + std::abs(dy));
			return (dy > 0.0 ? 3.0 - p : 1.0 + p) / 4.0; // [0..1)
		}

		struct DelaunatorPoint
		{
			std::size_t i;
			double x;
			double y;
			std::size_t t;
			std::size_t prev;
			std::size_t next;
			bool removed;
		};

		Delaunator::Delaunator(std::vector<double> const& in_coords)
			: coords(in_coords),
			  triangles(),
			  halfedges(),
			  hull_prev(),
			  hull_next(),
			  hull_tri(),
			  hull_start(),
			  m_hash(),
			  m_hash_size(),
			  m_edge_stack()
		{
			std::size_t n = coords.size() >> 1;

			double max_x = (std::numeric_limits<double>::min)();
			double max_y = (std::numeric_limits<double>::min)();
			double min_x = (std::numeric_limits<double>::max)();
			double min_y = (std::numeric_limits<double>::max)();
			std::vector<std::size_t> ids;
			ids.reserve(n);

			for (std::size_t i = 0; i < n; i++)
			{
				const double x = coords[2 * i];
				const double y = coords[2 * i + 1];

				if (x < min_x)
					min_x = x;
				if (y < min_y)
					min_y = y;
				if (x > max_x)
					max_x = x;
				if (y > max_y)
					max_y = y;

				ids.push_back(i);
			}
			const double cx = (min_x + max_x) / 2;
			const double cy = (min_y + max_y) / 2;
			double min_dist = (std::numeric_limits<double>::max)();

			std::size_t i0 = INVALID_INDEX;
			std::size_t i1 = INVALID_INDEX;
			std::size_t i2 = INVALID_INDEX;

			// 选择一个靠近质心的种子点
			for (std::size_t i = 0; i < n; i++)
			{
				const double d = dist(cx, cy, coords[2 * i], coords[2 * i + 1]);
				if (d < min_dist)
				{
					i0 = i;
					min_dist = d;
				}
			}

			const double i0x = coords[2 * i0];
			const double i0y = coords[2 * i0 + 1];

			min_dist = (std::numeric_limits<double>::max)();

			// 找到离种子最近的点
			for (std::size_t i = 0; i < n; i++)
			{
				if (i == i0)
					continue;
				const double d = dist(i0x, i0y, coords[2 * i], coords[2 * i + 1]);
				if (d < min_dist && d > 0.0)
				{
					i1 = i;
					min_dist = d;
				}
			}

			double i1x = coords[2 * i1];
			double i1y = coords[2 * i1 + 1];

			double min_radius = (std::numeric_limits<double>::max)();

			// 找到第三个点，它与前两个点形成最小的外接圆
			for (std::size_t i = 0; i < n; i++)
			{
				if (i == i0 || i == i1)
					continue;

				const double r = circumradius(i0x, i0y, i1x, i1y, coords[2 * i], coords[2 * i + 1]);

				if (r < min_radius)
				{
					i2 = i;
					min_radius = r;
				}
			}

			if (!(min_radius < (std::numeric_limits<double>::max())))
			{
				throw delaunator_error("All points collinear");
			}

			double i2x = coords[2 * i2];
			double i2y = coords[2 * i2 + 1];

			if (orient(i0x, i0y, i1x, i1y, i2x, i2y))
			{
				std::swap(i1, i2);
				std::swap(i1x, i2x);
				std::swap(i1y, i2y);
			}

			m_center = circumcenter(i0x, i0y, i1x, i1y, i2x, i2y);

			// 计算距中心的距离一次，以避免每次比较都需要计算距离。
			std::vector<double> dists;
			dists.reserve(n);
			for (size_t i = 0; i < n; i++)
			{
				const double& x = coords[2 * i];
				const double& y = coords[2 * i + 1];
				dists.push_back(dist(x, y, m_center.x(), m_center.y()));
			}

			// 按距离种子三角形外心的距离对点进行排序
			std::sort(ids.begin(), ids.end(), [&dists](std::size_t i, std::size_t j) {
				return dists[i] < dists[j];
			});

			// 初始化哈希表以存储前进凸包的边
			m_hash_size = static_cast<std::size_t>(std::llround(std::ceil(std::sqrt(n))));
			m_hash.resize(m_hash_size);
			std::fill(m_hash.begin(), m_hash.end(), INVALID_INDEX);

			// 初始化阵列以跟踪前进凸包的边
			hull_prev.resize(n);
			hull_next.resize(n);
			hull_tri.resize(n);

			hull_start = i0;

			size_t hull_size = 3;

			hull_next[i0] = hull_prev[i2] = i1;
			hull_next[i1] = hull_prev[i0] = i2;
			hull_next[i2] = hull_prev[i1] = i0;

			hull_tri[i0] = 0;
			hull_tri[i1] = 1;
			hull_tri[i2] = 2;

			m_hash[hash_key(i0x, i0y)] = i0;
			m_hash[hash_key(i1x, i1y)] = i1;
			m_hash[hash_key(i2x, i2y)] = i2;

			std::size_t max_triangles = n < 3 ? 1 : 2 * n - 5;
			triangles.reserve(max_triangles * 3);
			halfedges.reserve(max_triangles * 3);
			add_triangle(i0, i1, i2, INVALID_INDEX, INVALID_INDEX, INVALID_INDEX);
			double xp = std::numeric_limits<double>::quiet_NaN();
			double yp = std::numeric_limits<double>::quiet_NaN();
			for (std::size_t k = 0; k < n; k++)
			{
				const std::size_t i = ids[k];
				const double x = coords[2 * i];
				const double y = coords[2 * i + 1];

				// 在重复点附近，跳过
				if (k > 0 && check_pts_equal(x, y, xp, yp))
					continue;
				xp = x;
				yp = y;

				// 跳过种子三角形点
				if (check_pts_equal(x, y, i0x, i0y) || check_pts_equal(x, y, i1x, i1y) ||
					check_pts_equal(x, y, i2x, i2y))
					continue;

				// 使用边散列在凸面外壳上查找可见边
				std::size_t start = 0;

				size_t key = hash_key(x, y);
				for (size_t j = 0; j < m_hash_size; j++)
				{
					start = m_hash[fast_mod(key + j, m_hash_size)];
					if (start != INVALID_INDEX && start != hull_next[start])
						break;
				}

				start = hull_prev[start];
				size_t e = start;
				size_t q;

				while (q = hull_next[e],
					   !orient(x,
							   y,
							   coords[2 * e],
							   coords[2 * e + 1],
							   coords[2 * q],
							   coords[2 * q + 1]))
				{ // TODO: does it works in a same way as in JS
					e = q;
					if (e == start)
					{
						e = INVALID_INDEX;
						break;
					}
				}

				if (e == INVALID_INDEX)
					continue; // 可能是一个接近重复点；跳过它

				// 从点开始添加第一个三角形
				std::size_t t =
					add_triangle(e, i, hull_next[e], INVALID_INDEX, INVALID_INDEX, hull_tri[e]);

				hull_tri[i] = legalize(t + 2);
				hull_tri[e] = t;
				hull_size++;

				// 沿着外壳向前走，添加更多三角形并递归翻转
				std::size_t next = hull_next[e];
				while (q = hull_next[next],
					   orient(x,
							  y,
							  coords[2 * next],
							  coords[2 * next + 1],
							  coords[2 * q],
							  coords[2 * q + 1]))
				{
					t = add_triangle(next, i, q, hull_tri[i], INVALID_INDEX, hull_tri[next]);
					hull_tri[i] = legalize(t + 2);
					hull_next[next] = next; // 标记为删除
					hull_size--;
					next = q;
				}

				// 从另一边向后走，添加更多三角形并翻转
				if (e == start)
				{
					while (q = hull_prev[e],
						   orient(x,
								  y,
								  coords[2 * q],
								  coords[2 * q + 1],
								  coords[2 * e],
								  coords[2 * e + 1]))
					{
						t = add_triangle(q, i, e, INVALID_INDEX, hull_tri[e], hull_tri[q]);
						legalize(t + 2);
						hull_tri[q] = t;
						hull_next[e] = e; // 标记为删除
						hull_size--;
						e = q;
					}
				}

				// 更新外壳索引
				hull_prev[i] = e;
				hull_start = e;
				hull_prev[next] = i;
				hull_next[e] = i;
				hull_next[i] = next;

				m_hash[hash_key(x, y)] = i;
				m_hash[hash_key(coords[2 * e], coords[2 * e + 1])] = e;
			}
		}

		double Delaunator::get_hull_area()
		{
			std::vector<double> hull_area;
			size_t e = hull_start;
			do
			{
				hull_area.push_back((coords[2 * e] - coords[2 * hull_prev[e]]) *
									(coords[2 * e + 1] + coords[2 * hull_prev[e] + 1]));
				e = hull_next[e];
			} while (e != hull_start);
			return sum(hull_area);
		}

		std::size_t Delaunator::legalize(std::size_t a)
		{
			std::size_t i = 0;
			std::size_t ar = 0;
			m_edge_stack.clear();

			// 使用固定大小的堆栈消除递归
			while (true)
			{
				const size_t b = halfedges[a];

				/* 如果这对三角形不满足Delaunay条件（p1在[p0，pl，pr]的外接圆内），翻转它们，
				 * 然后对新的一对三角形执行相同的检查/递归翻转
				 *
				 *           pl                    pl
				 *          /||\                  /  \
				 *       al/ || \bl            al/    \a
				 *        /  ||  \              /      \
				 *       /  a||b  \    flip    /___ar___\
				 *     p0\   ||   /p1   =>   p0\---bl---/p1
				 *        \  ||  /              \      /
				 *       ar\ || /br             b\    /br
				 *          \||/                  \  /
				 *           pr                    pr
				 */
				const size_t a0 = 3 * (a / 3);
				ar = a0 + (a + 2) % 3;

				if (b == INVALID_INDEX)
				{
					if (i > 0)
					{
						i--;
						a = m_edge_stack[i];
						continue;
					}
					else
					{
						// i = INVALID_INDEX;
						break;
					}
				}

				const size_t b0 = 3 * (b / 3);
				const size_t al = a0 + (a + 1) % 3;
				const size_t bl = b0 + (b + 2) % 3;

				const std::size_t p0 = triangles[ar];
				const std::size_t pr = triangles[a];
				const std::size_t pl = triangles[al];
				const std::size_t p1 = triangles[bl];

				const bool illegal = in_circle(coords[2 * p0],
											   coords[2 * p0 + 1],
											   coords[2 * pr],
											   coords[2 * pr + 1],
											   coords[2 * pl],
											   coords[2 * pl + 1],
											   coords[2 * p1],
											   coords[2 * p1 + 1]);

				if (illegal)
				{
					triangles[a] = p1;
					triangles[b] = p0;

					auto hbl = halfedges[bl];

					// 外壳另一侧的边缘交换（罕见）；修复半边参照
					if (hbl == INVALID_INDEX)
					{
						std::size_t e = hull_start;
						do
						{
							if (hull_tri[e] == bl)
							{
								hull_tri[e] = a;
								break;
							}
							e = hull_prev[e];
						} while (e != hull_start);
					}
					link(a, hbl);
					link(b, halfedges[ar]);
					link(ar, bl);
					std::size_t br = b0 + (b + 1) % 3;

					if (i < m_edge_stack.size())
					{
						m_edge_stack[i] = br;
					}
					else
					{
						m_edge_stack.push_back(br);
					}
					i++;
				}
				else
				{
					if (i > 0)
					{
						i--;
						a = m_edge_stack[i];
						continue;
					}
					else
					{
						break;
					}
				}
			}
			return ar;
		}

		std::size_t Delaunator::hash_key(const double x, const double y) const
		{
			const double dx = x - m_center.x();
			const double dy = y - m_center.y();
			return fast_mod(static_cast<std::size_t>(std::llround(std::floor(
								pseudo_angle(dx, dy) * static_cast<double>(m_hash_size)))),
							m_hash_size);
		}

		std::size_t Delaunator::add_triangle(std::size_t i0,
											 std::size_t i1,
											 std::size_t i2,
											 std::size_t a,
											 std::size_t b,
											 std::size_t c)
		{
			std::size_t t = triangles.size();
			triangles.push_back(i0);
			triangles.push_back(i1);
			triangles.push_back(i2);
			link(t, a);
			link(t + 1, b);
			link(t + 2, c);
			return t;
		}

		void Delaunator::link(const std::size_t a, const std::size_t b)
		{
			std::size_t s = halfedges.size();
			if (a == s)
			{
				halfedges.push_back(b);
			}
			else if (a < s)
			{
				halfedges[a] = b;
			}
			else
			{
				throw delaunator_error("Cannot link edge");
			}
			if (b != INVALID_INDEX)
			{
				std::size_t s2 = halfedges.size();
				if (b == s2)
				{
					halfedges.push_back(a);
				}
				else if (b < s2)
				{
					halfedges[b] = a;
				}
				else
				{
					throw delaunator_error("Cannot link edge");
				}
			}
		}
	}
}
