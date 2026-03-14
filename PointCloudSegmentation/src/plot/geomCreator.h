#pragma once
#include <osg/Geode>
#include <osg/BoundingBox>
#include <osg/Vec3>
#include <osg/Vec3d>
#include <osg/Vec4>
#include <vector>
#include "../core/pointTypes.h"
#include <pcl/point_cloud.h>
namespace osg {
	class Geode;
}

namespace d3s {
	namespace pcs {

		/**
		 *  @brief    创建坐标轴
		 *
		 *  @param    osg::BoundingBox bbox
		 *
		 *  @return   osg::Geode*
		 */
		osg::Geode* CreateAxis(osg::BoundingBox bbox = osg::BoundingBox(osg::Vec3(0, 0, 0),
																		osg::Vec3(150, 150, 150)));


		/**
		*  @brief    创建长方体
		*
		*  @prarm	 const osg::BoundingBox & bbox						边界框数据
		*  @prarm	 const osg::Vec4 & clrFill							面填充颜色
		*  @prarm	 const osg::Vec4 & clrBorder						边填充颜色
		*
		*  @return   osg::Geode*
		*/
		osg::Geode* CreateCube(const osg::BoundingBox& bbox,
							  const osg::Vec4& clrFill,
							   const osg::Vec4& clrBorder,
							   const osg::Vec3d& offset = osg::Vec3d());


		/**
		*  @brief    创建球体
		*
		*  @prarm	 const osg::Vec3d & position						球心的坐标
		*  @prarm	 double radius										球体半径
		*  @prarm	 const osg::Vec4 & color							球体颜色
		*  @prarm	 const osg::Vec3d & offset							坐标偏移
		*
		*  @return   osg::Geode*
		*/
		osg::Geode* CreateSphere(const osg::Vec3d& position,
								 double radius,
								 const osg::Vec4& color,
								 const osg::Vec3d& offset = osg::Vec3d());

		/**
		*  @brief    创建多段线
		*
		*  @prarm	 const std::vector<osg::Vec3d> & path				顶点路径
		*  @prarm	 const osg::Vec4 & color							颜色
		*  @prarm	 const osg::Vec3d & offset							偏移坐标
		*  @prarm	 int linewidth										线宽
		*
		*  @return   osg::Geode*
		*/
		osg::Geode* CreatePolyline(const std::vector<osg::Vec3d>& path,
								   const osg::Vec4& color,
								   const osg::Vec3d& offset = osg::Vec3d(),
								   int linewidth = 3);
		
		/**
		 *  @brief    创建点云
		 *
		 *  @param    const pcl::PointCloud<pcl::PointPCLH> & points	点云原始数据
		 *  @param    const std::vector<unsigned int> & indices			需要渲染索引
		 *
		 *  @return   osg::Geode*
		 */
		osg::Geode* CreatePointCloud(const pcl::PointCloud<PointPCLH>& points,
									 const std::vector<int>& indices);

		osg::Geode* CreatePointCloud(PointCloudViewPtr pointCloudPtr,
									 const std::vector<int>& indices);
	}
}