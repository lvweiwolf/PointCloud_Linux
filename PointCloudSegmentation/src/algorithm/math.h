/////////////////////////////////////////////////////////////////////
// 文件名称：math.h
// 功能描述：数据计算相关工具
// 创建标识：吕伟	2022/4/9
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef MATH_H_
#define MATH_H_

#ifndef PI
#define PI (3.14159265358979323846)
#endif

#include <src/utils/logging.h>

#include <cmath>
#include <vector>
#include <algorithm>

namespace d3s {
	namespace pcs {

		template <class T>
		const T& clamp(const T& t, const T& minimum, const T& maximum)
		{
			return ((t < minimum) ? minimum : ((t > maximum) ? maximum : t));
		}

		inline double sround(double r) { return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5); }

		template <typename T_IN, typename T_OUT>
		bool numericCast(T_IN in, T_OUT& out)
		{
			if (std::is_same<T_IN, T_OUT>::value)
			{
				out = static_cast<T_OUT>(in);
				return true;
			}

			if (std::is_integral<T_OUT>::value)
				in = static_cast<T_IN>(sround((double)in));

			if ((std::is_same<T_OUT, double>::value) ||
				(in <= static_cast<double>((std::numeric_limits<T_OUT>::max)()) &&
				 in >= static_cast<double>(std::numeric_limits<T_OUT>::lowest())))
			{
				out = static_cast<T_OUT>(in);
				return true;
			}

			return false;
		}

		/**
		 *  @brief    重心坐标系差值
		 *
		 *  @prarm	 double x1				|
		 *  @prarm	 double y1				|
		 *  @prarm	 double z1				|
		 *  @prarm	 double x2				|
		 *  @prarm	 double y2				|		三角形顶点坐标
		 *  @prarm	 double z2				|
		 *  @prarm	 double x3				|
		 *  @prarm	 double y3				|
		 *  @prarm	 double z3				|
		 *  @prarm	 double x				三角形内部点的投影坐标 x
		 *  @prarm	 double y				三角形内部点的投影坐标 y
		 *
		 *  @return   double
		 */
		double barycentricInterpolation(double x1,
										double y1,
										double z1,
										double x2,
										double y2,
										double z2,
										double x3,
										double y3,
										double z3,
										double x,
										double y);

		/**
		 *  @brief    确定向量的中值。对于空向量返回NaN。
		 *
		 *  @param    const std::vector<T> & elems
		 *  @return   double
		 */
		template <typename T>
		double Median(const std::vector<T>& elems)
		{
			CHECK(!elems.empty());

			const size_t mid_idx = elems.size() / 2;

			std::vector<T> ordered_elems = elems;
			std::nth_element(ordered_elems.begin(),
							 ordered_elems.begin() + mid_idx,
							 ordered_elems.end());

			if (elems.size() % 2 == 0)
			{
				const T mid_element1 = ordered_elems[mid_idx];
				const T mid_element2 =
					*std::max_element(ordered_elems.begin(), ordered_elems.begin() + mid_idx);
				return (mid_element1 + mid_element2) / 2.0;
			}
			else
			{
				return ordered_elems[mid_idx];
			}
		}

		/**
		 *  @brief    确定一个向量的平均值
		 *
		 *  @param    const std::vector<T> & elems
		 *  @return   double
		 */
		template <typename T>
		double Mean(const std::vector<T>& elems)
		{
			CHECK(!elems.empty());
			double sum = 0;

			for (const auto el : elems)
			{
				sum += static_cast<double>(el);
			}

			return sum / elems.size();
		}

		/**
		 *  @brief    确定一个向量的样本方差
		 *
		 *  @param    const std::vector<T> & elems
		 *  @return   double
		 */
		template <typename T>
		double Variance(const std::vector<T>& elems)
		{
			const double ave = Mean(elems);
			double var = 0;

			for (const auto el : elems)
			{
				const double diff = el - ave;
				var += diff * diff;
			}

			return var / (elems.size() - 1);
		}

		/**
		 *  @brief    确定向量的样本标准差
		 *
		 *  @param    const std::vector<T> & elems
		 *  @return   double
		 */
		template <typename T>
		double Stddev(const std::vector<T>& elems)
		{
			return std::sqrt(Variance(elems));
		}

	}
}

#endif // MATH_H_