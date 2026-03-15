//////////////////////////////////////////////////////////////////////
// 文件名称：pointCloudReader.h
// 功能描述：点云读取
// 创建标识：吕伟	2022/4/6
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once
#include <osg/Vec3d>
#include <osg/BoundingBox>
#include <string>
#include <cstdint>
#include "../core/pointTypes.h"
class LASreader;
class LASheader;

namespace d3s {
	namespace pcs {
		namespace io {

			class PointCloudReader
			{
			public:
				PointCloudReader(const std::string& filename);

				~PointCloudReader();

				/**
				 *  @brief    是否可用
				 *
				 *  @return   bool
				 */
				bool IsValid();

				/**
				 *  @brief    点的总数
				 *
				 *  @return   size_t
				 */
				size_t GetNumOfPoints();

				/**
				 *  @brief    从文件读取一个点到内存
				 *
				 *  @param    PointPCLH & point
				 *
				 *  @return   bool
				 */
				bool ReadNextPoint(PointRGBI& point);
				bool ReadNextPoint(PointPCLH& point);

				bool ReadNextPoint2(PointPCLH& point);
				bool Seek(std::size_t pos);

				/**
				 *  @brief    获得点云偏移点
				 *
				 *  @return   osg::Vec3d
				 */
				osg::Vec3d GetOffset() { return _offset; }
				osg::Vec3d GetCenter() { return _center; }

				/**
				 *  @brief    获得包围框
				 *
				 *  @return   osg::BoundingBox
				 */
				osg::BoundingBox GetBoundingBox() { return _bbox; }


				int GetEPSG() { return _epsg; }

			private:
				LASheader* _lasHeader;
				LASreader* _lasReader;
				int _epsg;

				osg::BoundingBox _bbox;
				osg::Vec3d _offset;
				osg::Vec3d _center;

				osg::Vec3d _scale;
			};

		}
	}
}