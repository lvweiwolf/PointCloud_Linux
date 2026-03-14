//////////////////////////////////////////////////////////////////////
// 文件名称：anderson_acceleration.h
// 功能描述：安德森加速法，加速迭代收敛
// 创建标识：吕伟	2022/12/16
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef ANDERSON_ACCELERATION_H_
#define ANDERSON_ACCELERATION_H_

#include <Eigen/Dense>

#ifdef USE_FLOAT_SCALAR
typedef float Scalar
#else
typedef double Scalar;
#endif

#ifdef EIGEN_DONT_ALIGN
#define EIGEN_ALIGNMENT Eigen::DontAlign
#else
#define EIGEN_ALIGNMENT Eigen::AutoAlign
#endif

	template <int Rows, int Cols, int Options = (Eigen::ColMajor | EIGEN_ALIGNMENT)>
	using MatrixT =
		Eigen::Matrix<Scalar, Rows, Cols, Options>; ///< A typedef of the dense matrix of Eigen.

typedef MatrixT<2, 1> Vector2;							  ///< A 2d column vector.
typedef MatrixT<2, 2> Matrix22;							  ///< A 2 by 2 matrix.
typedef MatrixT<2, 3> Matrix23;							  ///< A 2 by 3 matrix.
typedef MatrixT<3, 1> Vector3;							  ///< A 3d column vector.
typedef MatrixT<3, 2> Matrix32;							  ///< A 3 by 2 matrix.
typedef MatrixT<3, 3> Matrix33;							  ///< A 3 by 3 matrix.
typedef MatrixT<3, 4> Matrix34;							  ///< A 3 by 4 matrix.
typedef MatrixT<4, 1> Vector4;							  ///< A 4d column vector.
typedef MatrixT<4, 4> Matrix44;							  ///< A 4 by 4 matrix.
typedef MatrixT<4, Eigen::Dynamic> Matrix4X;			  ///< A 4 by n matrix.
typedef MatrixT<3, Eigen::Dynamic> Matrix3X;			  ///< A 3 by n matrix.
typedef MatrixT<Eigen::Dynamic, 3> MatrixX3;			  ///< A n by 3 matrix.
typedef MatrixT<2, Eigen::Dynamic> Matrix2X;			  ///< A 2 by n matrix.
typedef MatrixT<Eigen::Dynamic, 2> MatrixX2;			  ///< A n by 2 matrix.
typedef MatrixT<Eigen::Dynamic, 1> VectorX;				  ///< A nd column vector.
typedef MatrixT<Eigen::Dynamic, Eigen::Dynamic> MatrixXX; ///< A n by m matrix.
typedef Eigen::Matrix<Scalar, 12, 12, 0, 12, 12> EigenMatrix12;

// eigen quaternions
typedef Eigen::AngleAxis<Scalar> EigenAngleAxis;
typedef Eigen::Quaternion<Scalar, Eigen::DontAlign> EigenQuaternion;

namespace d3s {
	namespace pcs {
		template <typename DerivedM>
		void matrix_to_list(const Eigen::DenseBase<DerivedM>& M,
							std::vector<typename DerivedM::Scalar>& V)
		{
			using namespace std;
			V.resize(M.size());

			for (int j = 0; j < M.cols(); j++)
			{
				for (int i = 0; i < M.rows(); i++)
				{
					V[i + j * M.rows()] = M(i, j);
				}
			}
		}

		template <typename DerivedV, typename mType>
		bool median(const Eigen::MatrixBase<DerivedV>& V, mType& m)
		{
			using namespace std;
			if (V.size() == 0)
			{
				return false;
			}
			vector<typename DerivedV::Scalar> vV;
			matrix_to_list(V, vV);

			size_t n = vV.size() / 2;
			nth_element(vV.begin(), vV.begin() + n, vV.end());

			if (vV.size() % 2 == 0)
			{
				nth_element(vV.begin(), vV.begin() + n - 1, vV.end());
				m = 0.5 * (vV[n] + vV[n - 1]);
			}
			else
			{
				m = vV[n];
			}

			return true;
		}

		class AndersonAcceleration
		{
		public:
			AndersonAcceleration() : m_(-1), dim_(-1), iter_(-1), col_idx_(-1) {}

			void replace(const Scalar* u) { current_u_ = Eigen::Map<const VectorX>(u, dim_); }

			const VectorX& compute(const Scalar* g)
			{
				assert(iter_ >= 0);

				Eigen::Map<const VectorX> G(g, dim_);
				current_F_ = G - current_u_;

				if (iter_ == 0)
				{
					prev_dF_.col(0) = -current_F_;
					prev_dG_.col(0) = -G;
					current_u_ = G;
				}
				else
				{
					prev_dF_.col(col_idx_) += current_F_;
					prev_dG_.col(col_idx_) += G;

					Scalar eps = 1e-14;
					Scalar scale = std::max(eps, prev_dF_.col(col_idx_).norm());
					dF_scale_(col_idx_) = scale;
					prev_dF_.col(col_idx_) /= scale;

					int m_k = std::min(m_, iter_);


					if (m_k == 1)
					{
						theta_(0) = 0;
						Scalar dF_sqrnorm = prev_dF_.col(col_idx_).squaredNorm();
						M_(0, 0) = dF_sqrnorm;
						Scalar dF_norm = std::sqrt(dF_sqrnorm);

						if (dF_norm > eps)
						{
							theta_(0) =
								(prev_dF_.col(col_idx_) / dF_norm).dot(current_F_ / dF_norm);
						}
					}
					else
					{
						// Update the normal equation matrix, for the column and row corresponding
						// to the new dF column
						VectorX new_inner_prod =
							(prev_dF_.col(col_idx_).transpose() * prev_dF_.block(0, 0, dim_, m_k))
								.transpose();
						M_.block(col_idx_, 0, 1, m_k) = new_inner_prod.transpose();
						M_.block(0, col_idx_, m_k, 1) = new_inner_prod;

						// Solve normal equation
						cod_.compute(M_.block(0, 0, m_k, m_k));
						theta_.head(m_k) =
							cod_.solve(prev_dF_.block(0, 0, dim_, m_k).transpose() * current_F_);
					}

					// Use rescaled theata to compute new u
					current_u_ =
						G - prev_dG_.block(0, 0, dim_, m_k) *
								((theta_.head(m_k).array() / dF_scale_.head(m_k).array()).matrix());
					col_idx_ = (col_idx_ + 1) % m_;
					prev_dF_.col(col_idx_) = -current_F_;
					prev_dG_.col(col_idx_) = -G;
				}

				iter_++;
				return current_u_;
			}
			void reset(const Scalar* u)
			{
				iter_ = 0;
				col_idx_ = 0;
				current_u_ = Eigen::Map<const VectorX>(u, dim_);
			}

			// m: number of previous iterations used
			// d: dimension of variables
			// u0: initial variable values
			void init(int m, int d, const Scalar* u0)
			{
				assert(m > 0);
				m_ = m;
				dim_ = d;
				current_u_.resize(d);
				current_F_.resize(d);
				prev_dG_.resize(d, m);
				prev_dF_.resize(d, m);
				M_.resize(m, m);
				theta_.resize(m);
				dF_scale_.resize(m);
				current_u_ = Eigen::Map<const VectorX>(u0, d);
				iter_ = 0;
				col_idx_ = 0;
			}

		private:
			VectorX current_u_;
			VectorX current_F_;
			MatrixXX prev_dG_;
			MatrixXX prev_dF_;
			MatrixXX M_;	   // Normal equations matrix for the computing theta
			VectorX theta_;	   // theta value computed from normal equations
			VectorX dF_scale_; // The scaling factor for each column of prev_dF
			Eigen::CompleteOrthogonalDecomposition<MatrixXX> cod_;

			int m_;		  // Number of previous iterates used for Andreson Acceleration
			int dim_;	  // Dimension of variables
			int iter_;	  // Iteration count since initialization
			int col_idx_; // Index for history matrix column to store the next value
			int m_k_;

			Eigen::Matrix4d current_T_;
			Eigen::Matrix4d current_F_T_;

			MatrixXX T_prev_dF_;
			MatrixXX T_prev_dG_;
		};


	}
}

#endif // ANDERSON_ACCELERATION_H_