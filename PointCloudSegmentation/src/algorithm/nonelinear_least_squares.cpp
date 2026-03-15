#include <src/algorithm/nonelinear_least_squares.h>
#include <src/utils/logging.h>

// Ceres
#include <ceres/ceres.h>
#include <ceres/solver.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <ceres/cost_function.h>

#ifdef WIN32
#ifdef _DEBUG
#pragma comment(lib, "ceres-debug.lib")
#else
#pragma comment(lib, "ceres.lib")
#endif
#endif

namespace d3s {
	namespace pcs {
		// 曲线拟合实现
		//////////////////////////////////////////////////////////////////////////
		bool curveFitting(double* data, unsigned int size, double& a, double& b, double& c)
		{
			CHECK_MSG(size % 2 == 0, "曲线拟合样本数据错误.");
			// PCS_INFO("初始化系数 a: %lf b: %lf c: %lf.", a, b, c);

			int numSamples = size / 2;

			ceres::Problem problem;

			for (int i = 0; i < numSamples; ++i)
			{
				double x = data[i * 2];
				double y = data[i * 2 + 1];

				ceres::CostFunction* cost_function =
					new ceres::AutoDiffCostFunction<CatenaryResidual, 1, 1, 1, 1>(
						new CatenaryResidual(x, y));

				// problem.AddResidualBlock(cost_function, nullptr, &a, &b, &c);
				problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), &a, &b, &c);
			}

			ceres::Solver::Options options;
			options.linear_solver_type = ceres::DENSE_QR;
			options.minimizer_progress_to_stdout = true;
			options.logging_type = ceres::SILENT;

			// options.max_num_iterations = 100;
			// options.max_linear_solver_iterations = 200;

			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);

			// PCS_INFO(StringPrintf("%s", summary.BriefReport()));
			// PCS_INFO(StringPrintf("最终拟合系数 a: %lf b: %lf c: %lf.", a, b, c));

			return true;
		}



		// 多项式拟合实现
		//////////////////////////////////////////////////////////////////////////
		double polynomial(double x, const double* coefficients, int degree)
		{
			double result = 0.0;

			for (int i = 0; i <= degree; i++)
			{
				result += coefficients[i] * pow(x, i);
			}

			return result;
		}

		class PolynomialCostFunction
		{
		public:
			PolynomialCostFunction(double x, double y) : _x(x), _y(y) {}

			template <typename T>
			bool operator()(const T* const coeffs, T* residual) const
			{
				residual[0] = T(_y) - coeffs[0];
				T xn = T(_x);

				for (int i = 1; i <= N; ++i)
				{
					residual[0] -= coeffs[i] * xn;
					xn *= T(_x);
				}

				return true;
			}

			static const int N = 7;

		private:
			const double _x, _y;
		};

		bool polynomialFitting(double* data, unsigned int size, std::vector<double>& weights)
		{
			int numSamples = size / 2;

			ceres::Problem problem;

			for (int i = 0; i < numSamples; ++i)
			{
				double x = data[i * 2];
				double y = data[i * 2 + 1];

				ceres::CostFunction* cost_function = new ceres::
					AutoDiffCostFunction<PolynomialCostFunction, 1, PolynomialCostFunction::N + 1>(
						new PolynomialCostFunction(x, y));

				problem.AddResidualBlock(cost_function, nullptr, &weights[0]);
			}

			ceres::Solver::Options options;
			options.linear_solver_type = ceres::DENSE_QR;
			options.minimizer_progress_to_stdout = true;
			// options.logging_type = ceres::SILENT;

			options.max_num_iterations = 1000;
			// options.max_linear_solver_iterations = 200;

			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);

			PCS_INFO(StringPrintf("%s", summary.BriefReport().c_str()));
			// PCS_INFO(StringPrintf("最终拟合系数 a: %lf b: %lf c: %lf.", a, b, c));

			return true;
		}

	}
}
