/////////////////////////////////////////////////////////////////////
// 文件名称：cloudProcess.h
// 功能描述：点云处理接口
// 创建标识：吕伟	2022/6/14
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef CLOUD_PROCESS_H_
#define CLOUD_PROCESS_H_

#include <src/core/pointTypes.h>

namespace d3s {
	namespace pcs {

		/**
		 *  @brief    欧式聚类(基于距离的区域生长)
		 *
		 *  @prarm	 PointCloudView<PointPCLH> & pcv					点云数据
		 *  @prarm	 const std::vector<int> & indices					点云索引
		 *  @prarm	 double tolerance									邻近查找半径
		 *  @prarm	 std::vector<std::vector<int>> & clusters			聚类结果
		 *
		 *  @return   void
		 */
		void euclideanCluster(PointCloudView<PointPCLH>& pcv,
							  const std::vector<int>& indices,
							  double tolerance,
							  unsigned int minPts,
							  unsigned int maxPts,
							  std::vector<std::vector<int>>& clusters);

		/**
		 *  @brief    欧式聚类(线程安全版本)
		 *
		 *  @prarm	 PointCloudView<PointPCLH> & pcv					点云数据
		 *  @prarm	 const std::vector<int> & indices					点云索引
		 *  @prarm	 double tolerance									邻近查找半径
		 *  @prarm	 std::vector<std::vector<int>> & clusters			聚类结果
		 *
		 *  @return   void
		 */
		void euclideanClusterSafe(const PointCloudView<PointPCLH>& pcv,
								  const std::vector<int>& indices,
								  double tolerance,
								  unsigned int minPts,
								  unsigned int maxPts,
								  std::vector<std::vector<int>>& clusters);


		/**
		 *  @brief    计算点云特征
		 *
		 *  @param    const PointCloudView<PointPCLH> & pcv				点云数据
		 *  @param    const std::vector<int> & indices					点云索引
		 *  @param    double radius										邻域查找半径
		 *  @param    std::string featureType							计算特征类型
		 *  @param    std::vector<float> & features						特征结果
		 *
		 *  @return   void
		 */
		void computeCloudFeatures(const PointCloudView<PointPCLH>& pcv,
								  const std::vector<int>& indices,
								  double radius,
								  std::string featureType,
								  std::vector<float>& features);

		/**
		 *  @brief    半径查询
		 *
		 *  @param    const std::vector<osg::Vec3d> & points			点云数据
		 *  @param    double radius										查找半径
		 *  @param    std::vector<std::vector<int>> & neigborsList		半径范围内的邻域
		 *  @param    std::vector<std::vector<float>> & distancesList	距离
		 *
		 *  @return   void
		 */
		void radiusSearch(const std::vector<osg::Vec3d>& points,
						  double radius,
						  std::vector<std::vector<int>>& neigborsList,
						  std::vector<std::vector<float>>& distancesList);


		/**
		 *  @brief    点云垂直方向且分层
		 *
		 *  @param    const PointCloudView<PointPCLH> & pcv				点云数据
		 *  @param    const std::vector<int> & indices					点云索引
		 *  @param    double step										垂直切分步长
		 *  @param    std::vector<std::vector<int>> & layers			切分结果
		 *
		 *  @return   void
		 */
		void sliceVerticalLayers(const PointCloudView<PointPCLH>& pcv,
								 const std::vector<int>& indices,
								 double step,
								 std::vector<std::vector<int>>& layers);

		/**
		 *  @brief    基于法线平面的点云平面提取和分割
		 *
		 *  @param    PointCloudView<PointPCLH> & pcv					点云数据
		 *  @param    const std::vector<int> & indices					拟合平面原始点云索引
		 *  @param    int minPts										平面拟合最少点数
		 *  @param    double distanceThreshold							平面拟合距离阈值
		 *  @param    std::vector<std::vector<int>> & inliers			拟合成功内点
		 *  @param    std::vector<std::vector<float>> & coefficens		平面稀疏
		 *
		 *  @return   void
		 */
		void planarSegment(PointCloudView<PointPCLH>& pcv,
						   const std::vector<int>& indices,
						   int minPts,
						   double distanceThreshold,
						   std::vector<std::vector<int>>& inliers,
						   std::vector<std::vector<float>>& coefficens);

		/**
		 *  @brief    获取PCA向量
		 *
		 *  @param    const std::vector<osg::Vec3d> & pts				计算点
		 *  @param    osg::Vec3d & major								主轴
		 *  @param    osg::Vec3d & middle								次轴1
		 *  @param    osg::Vec3d & minor								次轴2
		 *
		 *  @return   void
		 */
		void getEigenVectors(const std::vector<osg::Vec3d>& pts,
							 osg::Vec3d& major,
							 osg::Vec3d& middle,
							 osg::Vec3d& minor);

		/**
		 *  @brief    获取PCA各分量的值
		 *
		 *  @param    const std::vector<osg::Vec3d> & pts				计算点
		 *  @param    double & majorValue								主方向分量
		 *  @param    double & middleValue								次方向分量1
		 *  @param    double & minorValue								次方向分量2
		 *
		 *  @return   void
		 */
		void getEigenValues(const std::vector<osg::Vec3d>& pts,
							double& majorValue,
							double& middleValue,
							double& minorValue);


		/**
		 *  @brief    使用ICP Point-to-Point方法配准点云
		 *
		 *  @param    const PointCloudView<PointPCLH> & input			输入点云
		 *  @param    const PointCloudView<PointPCLH> & target			目标点云
		 *  @param    osg::Matrixd & finalTransform						评估变换矩阵
		 *  @param    double & rms										均方误差
		 *
		 *  @return   bool
		 */
		bool RegistrationPCL(const PointCloudView<PointPCLH>& input,
							 const PointCloudView<PointPCLH>& target,
							 osg::Matrix& matrix,
							 double& rms);

		bool Registration(const PointCloudView<PointPCLH>& input,
						  const PointCloudView<PointPCLH>& target,
						  osg::Matrix& matrix,
						  double& rms);

		double ComputeRMS(const PointCloudView<PointPCLH>& input,
						  const PointCloudView<PointPCLH>& target,
						  const osg::Matrix& matrix);

	}
}

#endif // CLOUD_PROCESS_H_
