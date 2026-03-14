/////////////////////////////////////////////////////////////////////
// 文件名称：nonelinear_least_squares.h
// 功能描述：非线性最小二乘，求解最优化问题
// 创建标识：吕伟	2022/4/25
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef NONELINEAR_LEAST_SQUARES_H_
#define NONELINEAR_LEAST_SQUARES_H_

#include <vector>

namespace d3s {
	namespace pcs {

		struct CatenaryResidual
		{
			CatenaryResidual(double x, double y) : _x(x), _y(y) {}

			template <typename T>
			bool operator()(const T* const c1, const T* const c2, const T* const c3, T* residual) const
			{
				// residual[0] = _y - exp(m[0] * _x + c[0]);
				
				const T& a = *c1;
				const T& b = *c2;
				const T& c = *c3;

				// residual[0] = _y - (a + c * ceres::cosh((_x - b) / c));
				residual[0] = _y - (a * _x * _x + b * _x + c);
					

				return true;
			}

		private:
			const double _x;
			const double _y;
		};

		double polynomial(double x, const double* coefficients, int degree);

		/**
		 *  @brief    曲线拟合
		 *
		 *  @param    double * data							样本数据 (x, y)
		 *  @param    unsigned int size						样本数据内存大小
		 *  @param    double & a							曲线方程系数 a
		 *  @param    double & b							曲线方程系数 b
		 *  @param    double & c							曲线方程系数 c
		 *
		 *  @return   bool
		 */
		bool curveFitting(double* data, unsigned int size, double& a, double& b, double& c);

		/**
		 *  @brief    多项式拟合
		 *
		 *  @param    double * data							样本数据 (x, y)		
		 *  @param    unsigned int size						样本数据内存大小
		 *  @param    std::vector<double> & weights			多项式系数
		 * 
		 *	Example: 
		 *			  std::vector<double> weights = {1.0, 1.0, 1.0, 1.0} // a、b、c、d
		 *			  polynomialFitting(data, size, weights);	// y = ax^3 + bx^2 + cx + d
		 *
		 *  @return   bool
		 */
		bool polynomialFitting(double* data,
							   unsigned int size,
							   std::vector<double>& weights);
	}
}


#endif // NONELINEAR_LEAST_SQUARES_H_