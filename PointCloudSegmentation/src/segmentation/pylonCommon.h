//////////////////////////////////////////////////////////////////////
// 文件名称：pylonCommon.h
// 功能描述：杆塔公共工具
// 创建标识：吕伟	2022/12/8
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef PYLONCOMMON_H_
#define PYLONCOMMON_H_

#include <src/core/pointTypes.h>

#include <osg/Vec3d>
#include <osg/BoundingBox>

#include <vector>


namespace d3s {
	namespace pcs {

		typedef std::vector<osg::Vec3d> Positions;

		osg::Vec3d GetSideVector(const Positions& positions, int i, int j);

		osg::Vec3d GetSideVector(const Positions& positions, int i, int j, int k);

		void GetHeadIndices(PointCloudViewPtr input,
							const std::vector<int>& indices,
							double height,
							std::vector<int>& headIndices);

		void GetHeadIndices(PointCloudViewPtr input,
							const std::vector<int>& indices,
							double height,
							double step,
							int minPts,
							std::vector<int>& headIndices);
		

		void GetHeadDirection(PointCloudViewPtr input,
							  const std::vector<int>& indices,
							  double stdmul,
							  osg::Vec3d& dir,
							  int post);

		struct Trunk
		{
			Trunk() : layer(0) {}
			Trunk(int layer_, const osg::BoundingBox& bbox, const std::vector<int>& indices_)
				: layer(layer_), bound(bbox), indices(indices_)
			{
			}

			int layer;
			osg::BoundingBox bound;
			std::vector<int> indices;
		};


		std::vector<Trunk> GetPoleTrunkCandidate(PointCloudViewPtr input,
												 const std::vector<int>& indices,
												 double layerStep,
												 double clusterTolerance,
												 int i);

		/**
		 *  @brief    获得主干
		 *
		 *  @param    const std::vector<Trunk> & trunks			电杆主干候选
		 *  @param    std::vector<std::vector<int>> & clusters	聚类索引
		 *
		 *  @return   std::vector<d3s::pcs::Trunk>				电杆主干
		 */
		std::vector<Trunk> GetMainTrunks(const std::vector<Trunk>& trunks,
										 std::vector<std::vector<int>>& clusters);

		/**
		 *  @brief    获得中心
		 *
		 *  @param    PointCloudViewPtr input					点云数据
		 *  @param    const std::vector<Trunk> & trunks			电杆主干
		 *
		 *  @return   osg::Vec3
		 */
		void GetMainVector(PointCloudViewPtr input,
						   const std::vector<Trunk>& trunks,
						   osg::Vec3& center,
						   osg::Vec3& axis);

		/**
		 *  @brief    主干聚类，聚类条件是边界框在XY平面投影有重叠
		 *
		 *  @param    const std::vector<Trunk> & trunks			电杆主干候选
		 *  @param    double overlapThr							最小重叠面积
		 *  @param    std::vector<std::vector<int>> & clusters	聚类索引
		 *
		 *  @return   void
		 */
		void TrunkClustering(const std::vector<Trunk>& trunks,
							 double overlapThr,
							 std::vector<std::vector<int>>& clusters);
	}
}

#endif // PYLONCOMMON_H_
