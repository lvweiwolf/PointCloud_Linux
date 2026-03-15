//////////////////////////////////////////////////////////////////////
// 文件名称：meshProcess.h
// 功能描述：网格相关处理
// 创建标识：吕伟	2022/12/5
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef MESHPROCESS_H_
#define MESHPROCESS_H_

#include <src/core/pointTypes.h>

#include <osg/BoundingBox>

#include <functional>

namespace d3s {
	namespace pcs {

		class GenericMesh;

		typedef std::function<double()> NoiseFunc;

		/**
		 *  @brief    计算网格模型表面积
		 *
		 *  @param    GenericMesh * mesh					网格模型
		 *
		 *  @return   double
		 */
		double computeMeshArea(GenericMesh* mesh);

		/**
		 *  @brief    从网格上采样点云
		 *
		 *  @param    GenericMesh * mesh					网格模型
		 *  @param    double sampleDensity					采样密度(三角形内)
		 *  @param    std::vector<size_t> * triIndices		每个采样点的三角形索引（仅输出-可选）
		 *
		 *  @return   d3s::pcs::PointCloudViewPtr
		 */
		PointCloudViewPtr samplePointsOnMesh(GenericMesh* mesh,
											 double sampleDensity,
											 NoiseFunc nosieFunc = nullptr,
											 std::vector<size_t>* triIndices = nullptr);

		/**
		 *  @brief    从网格上采样点云
		 *
		 *  @param    GenericMesh * mesh					网格模型
		 *  @param    size_t numberOfPoints					采样点数量
		 *  @param    std::vector<size_t> * triIndices		每个采样点的三角形索引（仅输出-可选）
		 *
		 *  @return   d3s::pcs::PointCloudViewPtr
		 */
		PointCloudViewPtr samplePointsOnMesh(GenericMesh* mesh,
											 size_t numberOfPoints,
											 NoiseFunc nosieFunc = nullptr,
											 std::vector<size_t>* triIndices = nullptr);

		PointCloudViewPtr samplePointsOnMesh(GenericMesh* mesh,
											 double samplingDensity,
											 size_t theoreticNumberOfPoints,
											 NoiseFunc nosieFunc = nullptr,
											 std::vector<size_t>* triIndices = nullptr);


		void ClipMesh(GenericMesh* mesh, osg::BoundingBox clipBBox, std::string savepath);
	}
}

#endif // MESHPROCESS_H_