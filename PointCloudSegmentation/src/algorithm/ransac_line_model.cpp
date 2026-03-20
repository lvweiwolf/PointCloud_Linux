#include <src/algorithm/ransac_line_model.h>
#include <src/utils/logging.h>
#include <src/core/pointTypes.hpp>

// #include "../utils/stringutil.h"

#include <pcl/common/eigen.h>
#include <pcl/sample_consensus/eigen.h>
#include <pcl/sample_consensus/sac_model.h>

#include <atomic>

namespace d3s {
	namespace pcs {
		RansacModelLine2D::RansacModelLine2D(const PointCloudConstPtr& cloud,
											 bool random /*= false*/)
			: SampleConsensusModel<PointT>(cloud, random)
		{
			model_name_ = "RansacModelLine2D";
			sample_size_ = 2; // 两个点确定一条直线
			model_size_ = 4;  // 直线的起点和方向，共计4个参数
		}

		RansacModelLine2D::RansacModelLine2D(const PointCloudConstPtr& cloud,
											 const std::vector<int>& indices,
											 bool random /*= false*/)
			: SampleConsensusModel<PointT>(cloud, indices, random)
		{
			model_name_ = "RansacModelLine2D";
			sample_size_ = 2;
			model_size_ = 4;
		}

		bool RansacModelLine2D::computeModelCoefficients(const std::vector<int>& samples,
														 Eigen::VectorXf& model_coefficients)
		{
			// 拟合直线最少需要两个样本
			if (samples.size() != 2)
			{
				PCS_ERROR(
					StringPrintf("[RansacModelLine2D::computeModelCoefficients] Invalid set of "
								 "samples given (%lu)!",
								 samples.size()));

				return false;
			}

			// 两点重合，跳过
			if (fabs(input_->points[samples[0]].x - input_->points[samples[1]].x) <=
					std::numeric_limits<float>::epsilon() &&
				fabs(input_->points[samples[0]].y - input_->points[samples[1]].y) <=
					std::numeric_limits<float>::epsilon())
			{
				return false;
			}

			// 直线起点位置
			model_coefficients.resize(4);
			model_coefficients[0] = input_->points[samples[0]].x;
			model_coefficients[1] = input_->points[samples[0]].y;

			// 直线方向
			model_coefficients[2] = input_->points[samples[1]].x - model_coefficients[0];
			model_coefficients[3] = input_->points[samples[1]].y - model_coefficients[1];

			model_coefficients.template tail<2>().normalize();
			return true;
		}

		void RansacModelLine2D::getDistancesToModel(const Eigen::VectorXf& model_coefficients,
													std::vector<double>& distances)
		{
			// 需要一组有效的模型系数
			if (!isModelValid(model_coefficients))
				return;

			distances.resize(indices_->size());

			// 获取直线点和方向
			Eigen::Vector3f line_pt(model_coefficients[0], model_coefficients[1], 0);
			Eigen::Vector3f line_dir(model_coefficients[2], model_coefficients[3], 0);
			line_dir.normalize();

			// 迭代三维点并计算它们到直线的距离
			int indiceSize = (int)indices_->size();

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < indiceSize; ++i)
			{
				Eigen::Vector3f pt(input_->points[(*indices_)[i]].x,
								   input_->points[(*indices_)[i]].y,
								   0.0f);

				// 计算从点到线的距离
				// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
				// Need to estimate sqrt here to keep MSAC and friends general
				distances[i] = sqrt((line_pt - pt).cross(line_dir).squaredNorm());
			}
		}

		void RansacModelLine2D::selectWithinDistance(const Eigen::VectorXf& model_coefficients,
													 const double threshold,
													 std::vector<int>& inliers)
		{
			// 需要一组有效的模型系数
			if (!isModelValid(model_coefficients))
				return;

			double sqr_threshold = threshold * threshold;

			std::atomic<int> nr_p(0);
			inliers.resize(indices_->size());
			error_sqr_dists_.resize(indices_->size());

			// 获取直线点和方向
			Eigen::Vector3f line_pt(model_coefficients[0], model_coefficients[1], 0);
			Eigen::Vector3f line_dir(model_coefficients[2], model_coefficients[3], 0);
			line_dir.normalize();

			// 迭代三维点并计算它们到直线的距离
			int indiceSize = (int)indices_->size();

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < indiceSize; ++i)
			{
				Eigen::Vector3f pt(input_->points[(*indices_)[i]].x,
								   input_->points[(*indices_)[i]].y,
								   0.0f);

				// 计算从点到线的距离
				// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
				double sqr_distance = sqrt((line_pt - pt).cross(line_dir).squaredNorm());

				if (sqr_distance < sqr_threshold)
				{
					// 返回距离小于阈值的点的索引
					inliers[nr_p] = (*indices_)[i];
					error_sqr_dists_[nr_p] = sqr_distance;
					++nr_p;
				}
			}

			inliers.resize(nr_p);
			error_sqr_dists_.resize(nr_p);
		}

		int RansacModelLine2D::countWithinDistance(const Eigen::VectorXf& model_coefficients,
												   const double threshold)
		{
			// 需要一组有效的模型系数
			if (!isModelValid(model_coefficients))
				return (0);

			double sqr_threshold = threshold * threshold;

			std::atomic<int> nr_p(0);

			// 获取直线点和方向
			Eigen::Vector3f line_pt(model_coefficients[0], model_coefficients[1], 0);
			Eigen::Vector3f line_dir(model_coefficients[2], model_coefficients[3], 0);
			line_dir.normalize();

			// 迭代三维点并计算它们到直线的距离
			int indiceSize = (int)indices_->size();

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < indiceSize; ++i)
			{
				Eigen::Vector3f pt(input_->points[(*indices_)[i]].x,
								   input_->points[(*indices_)[i]].y,
								   0.0f);

				// 计算从点到线的距离
				// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
				double sqr_distance = (line_pt - pt).cross(line_dir).squaredNorm();

				if (sqr_distance < sqr_threshold)
					nr_p++;
			}

			return (nr_p);
		}

		void RansacModelLine2D::optimizeModelCoefficients(const std::vector<int>& inliers,
														  const Eigen::VectorXf& model_coefficients,
														  Eigen::VectorXf& optimized_coefficients)
		{
			optimized_coefficients = model_coefficients;

			// 需要一组有效的模型系数
			if (!isModelValid(model_coefficients))
			{
				return;
			}

			// 需要至少两个有效点用来计算直线模型系数
			if (inliers.size() <= 2)
			{
				PCS_ERROR(StringPrintf(
					"[RansacModelLine2D::optimizeModelCoefficients] Not enough inliers "
					"found to support a model (%lu)! Returning the same coefficients.",
					inliers.size()));

				return;
			}

			tmp_inliers_ = &inliers;

			OptimizationFunctor functor(static_cast<int>(inliers.size()), this);
			Eigen::NumericalDiff<OptimizationFunctor> num_diff(functor);

			Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, float> lm(
				num_diff);

			int info = lm.minimize(optimized_coefficients);

			// Compute the L2 norm of the residuals
			PCS_INFO(StringPrintf(
				"[pcl::SampleConsensusModelCircle2D::optimizeModelCoefficients] LM solver finished "
				"with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g"
				"\nFinal solution: %g %g %g %g\n",
				info,
				lm.fvec.norm(),
				model_coefficients[0],
				model_coefficients[1],
				model_coefficients[2],
				model_coefficients[3],
				optimized_coefficients[0],
				optimized_coefficients[1],
				optimized_coefficients[2],
				optimized_coefficients[3]));
		}

		void RansacModelLine2D::projectPoints(const std::vector<int>& inliers,
											  const Eigen::VectorXf& model_coefficients,
											  PointCloud& projected_points,
											  bool copy_data_fields /*= true*/)
		{
			//// 需要一组有效的模型系数
			if (!isModelValid(model_coefficients))
				return;

			// 获取直线点和方向
			Eigen::Vector3f line_pt(model_coefficients[0], model_coefficients[1], 0);
			Eigen::Vector3f line_dir(model_coefficients[2], model_coefficients[3], 0);
			line_dir.normalize();

			projected_points.header = input_->header;
			projected_points.is_dense = input_->is_dense;

			// 是否将所有数据字段从输入云中复制到投影云中？
			if (copy_data_fields)
			{
				// 分配足够的空间并复制基本内容
				projected_points.points.resize(input_->points.size());
				projected_points.width = input_->width;
				projected_points.height = input_->height;

				typedef typename pcl::traits::fieldList<PointT>::type FieldList;

				// 迭代所有点
				for (size_t i = 0; i < projected_points.points.size(); ++i)
				{
					// 迭代每个字段
					pcl::for_each_type<FieldList>(
						pcl::NdConcatenateFunctor<PointT, PointT>(input_->points[i],
																  projected_points.points[i]));
				}

				// 迭代三维点并计算它们到直线的距离
				for (size_t i = 0; i < inliers.size(); ++i)
				{
					Eigen::Vector3f pt(input_->points[(*indices_)[i]].x,
									   input_->points[(*indices_)[i]].y,
									   0.0f);

					// double k = (DOT_PROD_3D (points[i], p21) - dotA_B) / dotB_B;
					float k = (pt.dot(line_dir) - line_pt.dot(line_dir)) / line_dir.dot(line_dir);

					Eigen::Vector3f pp = line_pt + k * line_dir;
					// 计算点在直线上的投影 (pointProj = A + k * B)
					projected_points.points[inliers[i]].x = pp[0];
					projected_points.points[inliers[i]].y = pp[1];
				}
			}
			else
			{
				// 分配足够的空间并复制基本内容
				projected_points.points.resize(inliers.size());
				projected_points.width = static_cast<uint32_t>(inliers.size());
				projected_points.height = 1;

				typedef typename pcl::traits::fieldList<PointT>::type FieldList;

				for (size_t i = 0; i < inliers.size(); ++i)
				{
					pcl::for_each_type<FieldList>(
						pcl::NdConcatenateFunctor<PointT, PointT>(input_->points[inliers[i]],
																  projected_points.points[i]));
				}

				// 迭代三维点并计算它们到直线的距离
				for (size_t i = 0; i < inliers.size(); ++i)
				{
					Eigen::Vector3f pt(input_->points[inliers[i]].x,
									   input_->points[inliers[i]].y,
									   0);

					// double k = (DOT_PROD_3D (points[i], p21) - dotA_B) / dotB_B;
					float k = (pt.dot(line_dir) - line_pt.dot(line_dir)) / line_dir.dot(line_dir);

					Eigen::Vector3f pp = line_pt + k * line_dir;
					// 计算点在直线上的投影 (pointProj = A + k * B)
					projected_points.points[i].x = pp[0];
					projected_points.points[i].y = pp[1];
				}
			}
		}

		bool RansacModelLine2D::doSamplesVerifyModel(const std::set<int>& indices,
													 const Eigen::VectorXf& model_coefficients,
													 const double threshold)
		{
			// 需要一组有效的模型系数
			if (!isModelValid(model_coefficients))
				return false;

			// 获取直线点和方向
			Eigen::Vector3f line_pt(model_coefficients[0], model_coefficients[1], 0);
			Eigen::Vector3f line_dir(model_coefficients[2], model_coefficients[3], 0);
			line_dir.normalize();

			double sqr_threshold = threshold * threshold;
			// 迭代三维点并计算它们到直线的距离
			for (std::set<int>::const_iterator it = indices.begin(); it != indices.end(); ++it)
			{
				Eigen::Vector3f pt(input_->points[*it].x, input_->points[*it].y, 0);

				// 计算从点到线的距离
				// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
				if ((line_pt - pt).cross(line_dir).squaredNorm() > sqr_threshold)
				{
					return false;
				}
			}

			return true;
		}

		bool RansacModelLine2D::isSampleGood(const std::vector<int>& samples) const
		{
			if ((input_->points[samples[0]].x != input_->points[samples[1]].x) &&
				(input_->points[samples[0]].y != input_->points[samples[1]].y))
			{
				return true;
			}

			return true;
		}

	}
}