//////////////////////////////////////////////////////////////////////
// 文件名称：PCQueryWrapperToolkit.h
// 功能描述：点云查询包装工具
// 创建标识：李成 2022/12/05
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef PCQUERYWRAPPERTOOLKIT_H_
#define PCQUERYWRAPPERTOOLKIT_H_

#include <Segment/PointCloudManagerDef.h>

class CPCQueryWrapperToolkit
{
public:
	enum EPlaneType
	{
		eXYPlane = 0, // xy平面
		eXZPlane = 1, // xz平面
		eYZPlane = 2  // yz平面
	};

	/**
	 * 获取降噪等级配置
	 * @param [in]
	 * @return
	 */
	static pc::data::SDenoiseCfg GetDenoiseParam();

	/**
	 * 获取pagedlod文件
	 * @param [in] pPointCloudPagedLod
	 * @return
	 */
	static pc::data::tagPagedLodFile GetPagedLodModelPath(
		pc::data::CModelNodePtr pPointCloudPagedLod);

	/**
	 *  函数介绍    	查询元素包围盒列表（static）
	 *  输入参数    	vPointCloudElements	即将被查询包围盒的点云元素
	 *  输入参数    	vpwMatrix	vpw变换矩阵
	 *  输入参数    	vSelectPoints	框选点信息
	 *  输入参数    	boundToPointNum	包围盒信息列表
	 *  输出参数     vecResult 在框选范围中的点云元素
	 *  返回值   	void
	 */
	static void QueryBoundingBoxList(const pc::data::CModelNodeVector& vPointCloudElements,
									 const osg::Matrix& vpwMatrix,
									 const std::vector<osg::Vec3d>& vSelectPoints,
									 pc::data::PointCloudBoundToPointNum& boundToPointNum,
									 pc::data::CModelNodeVector& vecResult);

	/**
	 * 查询元素包围盒列表
	 * @param [in] vPointCloudElements	即将被查询包围盒的点云元素
	 *
	 * @param [in] boundingBox			查询包围盒大小
	 * @param [in] boundToPointNum		包围盒信息列表
	 *
	 * @param [in] vPointCloudElements	在包围盒中的点云元素
	 * @return
	 */
	static void QueryBoundingBoxList(const pc::data::CModelNodeVector& vPointCloudElements,
									 const osg::BoundingBox& boundingBox,
									 pc::data::PointCloudBoundToPointNum& boundToPointNum,
									 pc::data::CModelNodeVector& vecResult);

	/*
	 * 函数介绍：获取Eigen点（static）
	 * 输入参数：const std::vector<osg::Vec3d> &vPoints			点集
	 * 输出参数：osg::Vec3d &outMajorPoint						主要点
	 *			osg::Vec3d &outMiddlePoint						中心点
	 *			osg::Vec3d &outMinorPoint						较小点
	 * 返回值  ：void
	 */
	static void GetEigenVectors(osg::Vec3d& outMajorPoint,
								osg::Vec3d& outMiddlePoint,
								osg::Vec3d& outMinorPoint,
								const std::vector<osg::Vec3d>& vPoints);

	/**
	 *  函数介绍    	测量boundbox 平面面积
	 *
	 *  输入参数    	const osg::BoundingBox & boundBox		包围盒
	 *  输入参数    	EPlaneType eType						平面类型
	 *  输出参数
	 *  返回值   	double
	 */
	static double CalcArea(const osg::BoundingBox& boundBox, EPlaneType eType);
};

#endif // PCQUERYWRAPPERTOOLKIT_H_