/////////////////////////////////////////////////////////////////////
// 文件名称：ransac_line_model.h
// 功能描述：ransac直线检测
// 创建标识：吕伟	2022/4/16
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef RANSAC_LINE_MODEL_H_
#define RANSAC_LINE_MODEL_H_

#include <src/core/pointTypes.h>

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>

namespace d3s {
	namespace pcs {

		class RansacModelLine2D : public pcl::SampleConsensusModel<PointPCLH>
		{
		public:
			typedef PointPCLH PointT;
			typedef typename pcl::PointCloud<PointT> PointCloud;
			typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
			typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
			typedef typename pcl::search::Search<PointT>::Ptr SearchPtr;

			typedef boost::shared_ptr<RansacModelLine2D> Ptr;
			typedef boost::shared_ptr<const RansacModelLine2D> ConstPtr;


			/**
			 *  @brief    拟合线模型 RansacModelLine 构造函数
			 *
			 *  @param    const PointCloudConstPtr & cloud	输入点云数据
			 *  @param    bool random
			 * 如果为true，则将随机种子设置为当前时间，否则设置为12345（默认值：false）
			 *
			 *  @return
			 */
			RansacModelLine2D(const PointCloudConstPtr& cloud, bool random = false);

			/**
			 *  @brief    拟合线模型 RansacModelLine 构造函数
			 *
			 *  @param    const PointCloudConstPtr & cloud	输入点云数据
			 *  @param    const std::vector<int> & indices  从输入点云中使用的点的索引列表
			 *  @param    bool random
			 * 如果为true，则将随机种子设置为当前时间，否则设置为12345（默认值：false）
			 *
			 *  @return
			 */
			RansacModelLine2D(const PointCloudConstPtr& cloud,
							  const std::vector<int>& indices,
							  bool random = false);

			/** @brief 空析构 */
			virtual ~RansacModelLine2D() {}

			/**
			 *  @brief    检查给定的索引样本是否可以形成有效的线模型，从这些样本计算模型系数，
			 *            并将其存储在 model_coefficients_ 中。直线系数由点和直线方向表示
			 *
			 *  @param    const std::vector<int> & samples  创建模型的最佳样本
			 *  @param    Eigen::VectorXf & model_coefficients 存储模型系数
			 *
			 *  @return   bool
			 */
			bool computeModelCoefficients(const std::vector<int>& samples,
										  Eigen::VectorXf& model_coefficients);


			/**
			 *  @brief    计算从云数据到给定直线模型的所有平方距离
			 *
			 *  @param    const Eigen::VectorXf & model_coefficients	模型系数
			 *  @param    std::vector<double> & distances				所有点到模型的欧式距离
			 *
			 *  @return   void
			 */
			void getDistancesToModel(const Eigen::VectorXf& model_coefficients,
									 std::vector<double>& distances);


			/**
			 *  @brief    选择所有与给定模型系数有关的点作为有效值(inliers)
			 *
			 *  @param    const Eigen::VectorXf & model_coefficients	模型系数
			 *  @param    const double threshold
			 * 一个最大容许距离阈值，用于从离群值确定边界
			 *  @param    std::vector<int> & inliers					有效值
			 *
			 *  @return   void
			 */
			void selectWithinDistance(const Eigen::VectorXf& model_coefficients,
									  const double threshold,
									  std::vector<int>& inliers);


			/**
			 *  @brief    将所有与给定模型系数有关的点计算为内联线。
			 *
			 *  @param    const Eigen::VectorXf & model_coefficients	模型系数
			 *  @param    const double threshold						满足模型的阈值
			 *
			 *  @return   int
			 */
			virtual int countWithinDistance(const Eigen::VectorXf& model_coefficients,
											const double threshold);


			/**
			 *  @brief
			 * 使用给定的inlier集合重新计算直线系数，并将其返回给用户。例如：直线拟合的SVD分解
			 *
			 *  @param    const std::vector<int> & inliers				满足模型的有效值
			 *  @param    const Eigen::VectorXf & model_coefficients	细化前的模型系数
			 *  @param    Eigen::VectorXf & optimized_coefficients		细化后的模型系数
			 *
			 *  @return   void
			 */
			void optimizeModelCoefficients(const std::vector<int>& inliers,
										   const Eigen::VectorXf& model_coefficients,
										   Eigen::VectorXf& optimized_coefficients);


			/**
			 *  @brief    创建一个新的点云，将内点投影到线模型上。
			 *
			 *  @param    const std::vector<int> & inliers				满足模型的有效值
			 *  @param    const Eigen::VectorXf & model_coefficients	模型系数
			 *  @param    PointCloud & projected_points
			 *  @param    bool copy_data_fields 如果需要复制其他数据字段，则设置为true
			 *
			 *  @return   void
			 */
			void projectPoints(const std::vector<int>& inliers,
							   const Eigen::VectorXf& model_coefficients,
							   PointCloud& projected_points,
							   bool copy_data_fields = true);


			/**
			 *  @brief    在数据子集上验证模型
			 *
			 *  @param    const std::set<int> & indices
			 *  @param    const Eigen::VectorXf & model_coefficients	模型系数
			 *  @param    const double threshold						阈值
			 *
			 *  @return   bool
			 */
			bool doSamplesVerifyModel(const std::set<int>& indices,
									  const Eigen::VectorXf& model_coefficients,
									  const double threshold);

			/** \brief Return an unique id for this model (SACMODEL_LINE). */
			inline pcl::SacModel getModelType() const { return (pcl::SACMODEL_LINE); }

		protected:
			using SampleConsensusModel<PointT>::sample_size_;
			using SampleConsensusModel<PointT>::model_size_;

			const std::vector<int>* tmp_inliers_;

			/** \brief Functor for the optimization function */
			struct OptimizationFunctor : pcl::Functor<float>
			{
				/**
				 *  @brief    优化器构造
				 *
				 *  @param    int m_data_points				点数量
				 *  @param    RansacModelLine2D * model		优化模型
				 *
				 *  @return
				 */
				OptimizationFunctor(int m_data_points, RansacModelLine2D* model)
					: pcl::Functor<float>(m_data_points), model_(model)
				{
				}

				/**
				 *  @brief    最小化损失函数
				 *
				 *  @param    const Eigen::VectorXf & x		模型系数
				 *  @param    Eigen::VectorXf & fvec		误差
				 *
				 *  @return   int
				 */
				int operator()(const Eigen::VectorXf& x, Eigen::VectorXf& fvec) const
				{
					Eigen::Vector3f line_pt(x[0], x[1], 0);
					Eigen::Vector3f line_dir(x[2], x[3], 0);
					line_dir.normalize();

					for (int i = 0; i < values(); ++i)
					{
						// Compute the difference between the center of the circle and the datapoint
						// X_i
						Eigen::Vector3f pt(model_->input_->points[(*model_->tmp_inliers_)[i]].x,
										   model_->input_->points[(*model_->tmp_inliers_)[i]].y,
										   0.0f);

						fvec[i] = (line_pt - pt).cross(line_dir).squaredNorm();
					}

					return (0);
				}

				RansacModelLine2D* model_;
			};

			/**
			 *  @brief    检查样本是否能很好的被模型表达
			 *
			 *  @param    const std::vector<int> & samples
			 *
			 *  @return   bool
			 */
			bool isSampleGood(const std::vector<int>& samples) const;
		};

	}
}

#endif // RANSAC_LINE_MODEL_H_