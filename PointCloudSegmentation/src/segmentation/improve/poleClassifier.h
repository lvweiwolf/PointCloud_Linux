//////////////////////////////////////////////////////////////////////
// 文件名称：poleClassifier.h
// 功能描述：电杆分类
// 创建标识：吕伟	2023/8/17
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////

#pragma once
#include <vector>
#include "../../core/pointTypes.h"
#include <osg/Vec3d>
namespace d3s {
	namespace pcs {

		struct PoleClassifyOptions
		{
			double bound_size = 1.5;
			double layer_step = 0.5;
			
			double pole_radius = 0.3;
			double pole_min_area_overlap = 0.005;
			double pole_clustering_dist = 0.1;

			double height_threshold = 4.0;
			double head_length = 3.0;
			double d1_scale = 0.6; // 杆塔边界d1方向缩放比例
			double d2_scale = 0.8; // 杆塔边界d2方向缩放比例
		};

		class PoleClassifier
		{
		public:
			typedef std::vector<osg::Vec3d> Positions;
			typedef std::vector<osg::Vec3d> Vectors;
			typedef std::vector<std::vector<osg::Vec3d>> Polygons;

			PoleClassifier(const PoleClassifyOptions& options,
						   PointCloudViewPtr input,
						   const std::vector<int>& indices);

			~PoleClassifier();

			/**
			 *  @brief    获得电杆边界信息
			 *
			 *  @return   d3s::pcs::PylonSegmentation::PylonPolygons
			 */
			Polygons GetPolePolygons() const;

			/**
			 *  @brief    获得电杆横担方向
			 *
			 *  @return   std::vector<osg::Vec3d>
			 */
			Vectors GetPoleSides() const;

			/**
			 *  @brief    设置电杆坐标信息
			 *
			 *  @prarm	 const PolePositions & polePositions
			 *
			 *  @return   void
			 */
			void SetPolePositions(const Positions& polePositions);

			/**
			 *  @brief   获得杆塔坐标信息
			 *
			 *  @return   d3s::pcs::PoleClassifier::Positions
			 */
			Positions GetPolePositions() const;

			/**
			 *  @brief    执行分割程序
			 *
			 *  @return   void
			 */
			void Segment();

		private:

			struct PoleAxis
			{
				PoleAxis() {}
				PoleAxis(const osg::Vec3& center_, const osg::Vec3& dir_)
					: center(center_), dir(dir_)
				{
				}

				osg::Vec3 center;
				osg::Vec3 dir;
			};

			/**
			 *  @brief    获得电杆局部点云
			 *
			 *  @param    PointCloudViewPtr input					点云数据
			 *  @param    const std::vector<int> & indices			点云索引
			 *
			 *  @return   std::vector<std::vector<int>>
			 */
			std::vector<std::vector<int>> GetPoleIndices(PointCloudViewPtr input,
														 const std::vector<int>& indices);

			std::vector<int> GetPoleHeadCandidate(PointCloudViewPtr input,
												  const std::vector<int>& indices,
												  const std::vector<osg::Vec3d>& polygon,
												  double heightThr);

 			std::vector<PoleAxis> ComputeTrunkCentroids(
				PointCloudViewPtr input,
				const std::vector<std::vector<int>>& arrIndices);


			void ComputePolePolygons(PointCloudViewPtr input,
									 const std::vector<std::vector<int>>& arrIndices,
									 Polygons& polePolygons,
									 Vectors& poleSides);

			double ComputePositionRadius(PointCloudViewPtr input,
										 const std::vector<int>& indices,
										 const osg::Vec3d& side,
										 osg::Vec3d& pos,
										 int post);

		private:
			PoleClassifyOptions _options; // 杆塔点云提取参数
			PointCloudViewPtr _input;	  // 输入点云数据
			std::vector<int> _indices;	  // 处理点云的索引列表

			Positions _polePositions; // 杆塔坐标
			Polygons _polePolygons;  // 杆塔边界
			Vectors _poleSides;	  // 杆塔横担方向
		};

	}
}
