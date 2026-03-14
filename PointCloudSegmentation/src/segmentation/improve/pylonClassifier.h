/////////////////////////////////////////////////////////////////////
// 文件名称：pylonClassifier.h
// 功能描述：杆塔分类提取
// 创建标识：吕伟	2022/9/17
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once
#include <vector>
#include "../../core/pointTypes.h"
namespace d3s {
	namespace pcs {

		class Grid2D;
		class LocationGrid;

		struct PylonClassifyOptions
		{
			double cell_size; // 格网单元大小

			int num_neighbors;			 // 格网邻域范围
			double height_threshold;	 // 过滤格网内点云的高度阈值

			double min_body_growing_height; // 塔体生长最小高度
			double max_body_growing_height; // 塔体生长最大高度
			double body_growing_step;		// 塔体生长步长
			double body_growing_resolition; // 塔体生长掩码分辨率

			double head_length;	   // 塔头长度
			double d1_scale;	   // 杆塔边界d1方向缩放比例
			double d2_scale;	   // 杆塔边界d2方向缩放比例
			double terminal_scale; // 终端杆塔边界缩放比例
			double corner_scale;   // 转角杆塔边界缩放比例
		};

		typedef std::vector<std::vector<int>> VerticalLayers;

		class PylonClassifier
		{
		public:
			typedef std::vector<osg::Vec3d> Positions;
			typedef std::vector<osg::Vec3d> Vectors;
			typedef std::vector<std::vector<osg::Vec3d>> Polygons;


			PylonClassifier(const PylonClassifyOptions& options,
							PointCloudViewPtr input,
							const std::vector<int>& indices);

			~PylonClassifier();

			/**
			 *  @brief    获得铁塔边界信息
			 *
			 *  @return   d3s::pcs::PylonSegmentation::PylonPolygons
			 */
			Polygons GetPylonPolygons() const;

			/**
			 *  @brief    获得杆塔横担方向
			 *
			 *  @return   std::vector<osg::Vec3d>
			 */
			Vectors GetPylonSides() const;

			/**
			 *  @brief    设置铁塔坐标信息
			 *
			 *  @prarm	 const PylonPositions & pylonPositions
			 *
			 *  @return   void
			 */
			void SetPylonPositions(const Positions& pylonPositions);


			/**
			 *  @brief   获得杆塔坐标信息
			 *
			 *  @return   d3s::pcs::PylonClassifier::Positions
			 */
			Positions GetPylonPositions() const;

			/**
			 *  @brief    执行分割程序
			 *
			 *  @return   void
			 */
			void Segment();

		private:
			/**
			 *  @brief    根据杆塔位置计算杆塔多边形边界
			 *
			 *  @prarm	 Positions & pylonPositions					铁塔坐标
			 *  @prarm	 std::vector<osg::Vec3d> & pylonSides		杆塔横担方向
			 *
			 *  @return   d3s::pcs::PylonClassifier::Polygons
			 */
			void ComputePylonPolygons(const Grid2D& grid,
									  Positions& pylonPositions,
									  Polygons& pylonPolygons,
									  Vectors& pylonSides);


			/**
			 *  @brief    获得多边形边界范围内的点云索引
			 *
			 *  @prarm	 const Grid2D & grid						格网
			 *  @prarm	 const std::vector<osg::Vec3d> & polygon	多边形边界
			 *
			 *  @return   std::vector<int>
			 */
			std::vector<int> GetPylonCandidate(const Grid2D& grid,
											   const std::vector<osg::Vec3d>& polygon,
											   double heightThr);

			/**
			*  @brief    根据杆塔候选点云计算实际杆塔边界尺寸
			*
			*  @prarm	 const std::vector<int> & indices			杆塔候选点云
			*  @prarm	 osg::Vec3d & pos							杆塔坐标
			*  @prarm	 int post									后缀
			*
			*  @return   double
			*/
			double ComputePositionRadius(const std::vector<int>& indices,
										 osg::Vec3d& pos,
										 int post);

			/**
			 *  @brief    移除杆塔上方悬空
			 *
			 *  @param    double step								空层检测步长
			 *  @param    std::vector<int> & indices				杆塔候选点云
			 *
			 *  @return   void
			 */
			void RemoveNoiseAbove(double step, std::vector<int>& indices);

			/**
			 *  @brief    创建垂直方向点云数据层
			 *
			 *  @prarm	 const std::vector<int> & indices		点云索引
			 *  @prarm	 double lower							下界
			 *  @prarm	 double upper							上界
			 *  @prarm	 double step							步长，层的高度
			 *
			 *  @return   d3s::pcs::VerticalLayers
			 */
			VerticalLayers CreateVerticalLayers(const std::vector<int>& indices,
												double lower,
												double upper,
												double step);


		private:
			PylonClassifyOptions _options; // 铁塔点云提取参数
			PointCloudViewPtr _input;	   // 输入点云数据
			std::vector<int> _indices;	   // 处理点云的索引列表

			Positions _pylonPositions; // 杆塔坐标
			Polygons _pylonPolygons;   // 杆塔边界
			Vectors _pylonSides;	   // 杆塔横担方向
		};

	}
}
