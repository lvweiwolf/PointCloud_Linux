#include <src/algorithm/registration.h>
#include <src/algorithm/icp_fast_robust.h>
#include <src/utils/logging.h>

namespace d3s {
	namespace pcs {
		bool registration(Vertices& vertices_input,
						  Vertices& vertices_target,
						  Vertices& normal_input,
						  Vertices& normal_target,
						  ICPMethod method,
						  Transform& transform,
						  Scalar& mse)
		{
			// 缩放（正则化）
			Eigen::Vector3d source_scale, target_scale;
			source_scale =
				vertices_input.rowwise().maxCoeff() - vertices_input.rowwise().minCoeff();
			target_scale =
				vertices_target.rowwise().maxCoeff() - vertices_target.rowwise().minCoeff();
			double scale = std::max(source_scale.norm(), target_scale.norm());

			 PCS_DEBUG("[Registration] scale = %lf", scale);

			vertices_input /= scale;
			vertices_target /= scale;

			// 均值偏移
			VectorN source_mean, target_mean;
			source_mean = vertices_input.rowwise().sum() / double(vertices_input.cols());
			target_mean = vertices_target.rowwise().sum() / double(vertices_target.cols());
			vertices_input.colwise() -= source_mean;
			vertices_target.colwise() -= target_mean;

			MatrixXX final_trans;
			int nIter = 0;

			// 设置ICP配准参数
			ICP::Parameters pars;

			// Sparse ICP配准参数
			SICP::Parameters spars;
			spars.p = 0.4;
			spars.print_icpn = false;

			// 执行配准
			 PCS_DEBUG("[Registration] 开始执行配准...");

			FastRobustICP<3> fricp;
			double begin_reg = omp_get_wtime();

			switch (method)
			{
			case ICPMethod::NormalICP:
			{
				pars.f = ICP::NONE;
				pars.use_AA = false;
				fricp.point_to_point(vertices_input,
									 vertices_target,
									 source_mean,
									 target_mean,
									 pars);
				final_trans = pars.res_trans;
				mse = pars.convergence_gt_mse;
				nIter = pars.convergence_iter;
				break;
			}
			case ICPMethod::AA_ICP:
			{
				AAICP::point_to_point_aaicp(vertices_input,
											vertices_target,
											source_mean,
											target_mean,
											pars);
				final_trans = pars.res_trans;
				mse = pars.convergence_gt_mse;
				nIter = pars.convergence_iter;
				break;
			}
			case FICP:
			{
				pars.f = ICP::NONE;
				pars.use_AA = true;
				fricp.point_to_point(vertices_input,
									 vertices_target,
									 source_mean,
									 target_mean,
									 pars);
				final_trans = pars.res_trans;
				mse = pars.convergence_gt_mse;
				nIter = pars.convergence_iter;
				break;
			}
			case RICP:
			{
				pars.f = ICP::WELSCH;
				pars.use_AA = true;
				fricp.point_to_point(vertices_input,
									 vertices_target,
									 source_mean,
									 target_mean,
									 pars);
				final_trans = pars.res_trans;
				mse = pars.convergence_gt_mse;
				nIter = pars.convergence_iter;
				break;
			}
			case PPL:
			{
				pars.f = ICP::NONE;
				pars.use_AA = false;

				if (normal_target.size() == 0)
				{
					PCS_WARN("目标模型不包含法线，无法执行 Point-to-plane 方法!");
					return false;
				}

				fricp.point_to_plane(vertices_input,
									 vertices_target,
									 normal_input,
									 normal_target,
									 source_mean,
									 target_mean,
									 pars);

				final_trans = pars.res_trans;
				mse = pars.convergence_gt_mse;
				nIter = pars.convergence_iter;
				break;
			}
			case RPPL:
			{
				pars.nu_end_k = 1.0 / 6;
				pars.f = ICP::WELSCH;
				pars.use_AA = true;
				if (normal_target.size() == 0)
				{
					PCS_WARN("目标模型不包含法线，无法执行 Point-to-plane 方法!");
					return false;
				}

				fricp.point_to_plane_GN(vertices_input,
										vertices_target,
										normal_input,
										normal_target,
										source_mean,
										target_mean,
										pars);

				final_trans = pars.res_trans;
				mse = pars.convergence_gt_mse;
				nIter = pars.convergence_iter;
				break;
			}
			case SparseICP:
			{
				SICP::point_to_point(vertices_input,
									 vertices_target,
									 source_mean,
									 target_mean,
									 spars);
				final_trans = spars.res_trans;
				mse = spars.convergence_gt_mse;
				nIter = spars.convergence_iter;
				break;
			}
			case SICPPPL:
			{
				if (normal_target.size() == 0)
				{
					PCS_WARN("目标模型不包含法线，无法执行 Point-to-plane 方法!");
					return false;
				}
				SICP::point_to_plane(vertices_input,
									 vertices_target,
									 normal_target,
									 source_mean,
									 target_mean,
									 spars);
				final_trans = spars.res_trans;
				mse = spars.convergence_gt_mse;
				nIter = spars.convergence_iter;
				break;
			}
			}

			// 恢复缩放尺度
			vertices_input = scale * vertices_input;
			final_trans.block(0, 3, 3, 1) *= scale;

			transform = final_trans;
			double end_reg = omp_get_wtime();
			double time = end_reg - begin_reg;

			PCS_DEBUG("[Registration] 配准完成, MSE=%lf,  Iters=%d, Time=%.3f s.", mse, nIter,
				time);

			return true;
		}
	}
}
