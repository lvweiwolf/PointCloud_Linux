/////////////////////////////////////////////////////////////////////
// 文件名称：statistics.h
// 功能描述：统计相关接口
// 创建标识：吕伟	2022/6/14
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef STATISTICS_H_
#define STATISTICS_H_

#include <src/core/pointTypes.h>

namespace d3s {
	namespace pcs {

		/**
		 *  @brief    按照比例统计最大/最小值
		 *
		 *  @param    const PointCloudView<PointPCLH> & pcv				点云数据
		 *  @param    const std::string & fieldname						字段名称
		 *  @param    int minpts,										区间最小点数
		 *  @param    double & min										最小值
		 *  @param    double & max										最大值
		 *
		 *  @return   void
		 */
		void computeMinMax(const PointCloudView<PointPCLH>& pcv,
						   const std::string& fieldname,
						   int minpts,
						   double step,
						   double& min,
						   double& max);

		void computeMinMax(const PointCloudView<PointPCLH>& pcv,
						   const std::vector<int>& indices,
						   const std::string& fieldname,
						   int minpts,
						   double step,
						   double& min,
						   double& max);

		void computeMinMax(const PointCloudView<PointPCLH>& pcv,
						   const std::vector<int>& indices,
						   const std::string& fieldname,
						   ClassificationType label,
						   int minpts,
						   double step,
						   double& min,
						   double& max);

		/**
		 *  @brief    获得点云的边界
		 *
		 *  @param    const PointCloudView<PointPCLH> & pcv				点云数据
		 *  @param    const std::vector<int> & indices					计算边界的点云索引
		 *  @param    osg::BoundingBox & bbox							边界范围
		 *
		 *  @return   void
		 */
		void computeMinMax3D(const PointCloudView<PointPCLH>& pcv,
							 const std::vector<int>& indices,
							 osg::BoundingBox& bbox);

		void computeMinMax3D(const PointCloudView<PointPCLH>& pcv, osg::BoundingBox& bbox);
	}
}

#endif // STATISTICS_H_
