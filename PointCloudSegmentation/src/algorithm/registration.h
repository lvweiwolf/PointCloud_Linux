//////////////////////////////////////////////////////////////////////
// 文件名称：registration.h
// 功能描述：点云配准算法
// 创建标识：吕伟	2022/12/16
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef REGISTRATION_H_
#define REGISTRATION_H_

#include <Eigen/Dense>

namespace d3s {
	namespace pcs {

		typedef double Scalar;

		typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Vertices;
		typedef Eigen::Matrix<Scalar, 3, 1> VectorN;
		typedef Eigen::Matrix<Scalar, 4, 4> Transform;

		enum ICPMethod
		{
			NormalICP,
			AA_ICP,
			FICP,
			RICP,
			PPL,
			RPPL,
			SparseICP,
			SICPPPL
		};

		/**
		 *  @brief    点云配准算法
		 *
		 *  @param    Vertices & vertices_input		配准点云
		 *  @param    Vertices & vertices_target	参考点云
		 *  @param    Vertices & normal_input		配准点云法向
		 *  @param    Vertices & normal_target		参考点云法向
		 *  @param    ICPMethod method				配准方法
		 *  @param    Transform & transform			变换矩阵
		 *  @param    Scalar & mse					配准均方误差
		 *
		 *  @return   bool
		 */
		bool registration(Vertices& vertices_input,
						  Vertices& vertices_target,
						  Vertices& normal_input,
						  Vertices& normal_target,
						  ICPMethod method,
						  Transform& transform,
						  Scalar& mse);
	}
}

#endif // REGISTRATION_H_
