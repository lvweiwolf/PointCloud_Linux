/*---------------------------------------------------------------------
*文件名称：PointCloudDistanceMeasure.h
*功能描述：点云距离计算
*创建标识：陶子龙2022.7.6.
*
*修改标识：
*修改描述：
----------------------------------------------------------------------*/
#ifndef POINTCLOUDDISTANCEMEASURE_H_
#define POINTCLOUDDISTANCEMEASURE_H_

#include <include/PointCloudManagerExport.h>

#include <osg/Vec3d>

#include <vector>

///////////////////////////////////////////////////////////////////////分析类型距离///////////////////////////////////////////////////////////////////////
class POINTCLOUDMANAGER_EXPORT CPointCloudDistanceMeasure
{
public:
	enum EMeasureType
	{
		eHorizontalMeasure = 0, // 水平距离（投影到平面上的距离）
		eVerticalMeasure = 1,	// 垂直距离（高度差值）
		eClearanceMeasure = 2,	// 净空距离（两点的距离）
	};

	struct MeasureResult
	{
		osg::Vec3d minContrastPt;  // 最短距离参考点
		osg::Vec3d minReferencePt; // 最短距离比较点
	};

public:
	CPointCloudDistanceMeasure(void);
	~CPointCloudDistanceMeasure(void);

public:
	/*------------------------------------------------------------------------距离------------------------------------------------------------------------*/
	/*
	 * 函数介绍：计算两份点云的最小距离（static）
	 * 输入参数：EDistanceType eMeasureType						距离的类型
	 *			const std::vector<osg::Vec3d> &vContrastPoints	比较点云
	 *			const std::vector<osg::Vec3d> &vReferencePoints	参考点云
	 * 输出参数：osg::Vec3d &minOutContrastPoint					最近距离时比较点云点
	 *			osg::Vec3d &minOutReferencePoint				最近距离时参考点云点
	 * 返回值  ：double											两份点云的最小距离
	 */
	static double GetMinDistance(osg::Vec3d& minOutContrastPoint,
								 osg::Vec3d& minOutReferencePoint,
								 EMeasureType eMeasureType,
								 const std::vector<osg::Vec3d>& vContrastPoints,
								 const std::vector<osg::Vec3d>& vReferencePoints);

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

	/*
	 * 函数介绍：快速获取两份大点云的距离（通常指百万级点云的距离）
	 * 输入参数：
	 *			const std::vector<osg::Vec3d> &minContrastPt	比较点云
	 * *			double dContrastThin	比较点云的抽稀间距
	 *			const std::vector<osg::Vec3d> &minReferencePt	参考点云
	 * *			double dReferenceThin	参考点云的抽稀间距
	 * 输出参数：osg::Vec3d &minContrastPt					最近距离时比较点云点
	 *			osg::Vec3d &minReferencePt				最近距离时参考点云点
	 * 返回值  ：double											两份点云的最小距离
	 */
	static double QuickGetMinClearanceDistance(
		CPointCloudDistanceMeasure::MeasureResult& measureResult,
		const std::vector<osg::Vec3d>& vContrastPts,
		const std::vector<osg::Vec3d>& vReferencePts,
		double dReferenceThin,
		double dContrastThin);

protected:
	/*------------------------------------------------------------------------距离------------------------------------------------------------------------*/
	/*
	 * 函数介绍：获取两份点云的最小水平距离（投影到平面上的距离）（static）
	 * 输入参数：const std::vector<osg::Vec3d> &vContrastPoints	比较点云
	 *			const std::vector<osg::Vec3d> &vReferencePoints	参考点云
	 * 输出参数：osg::Vec3d &minOutContrastPoint					最近距离时比较点云点
	 *			osg::Vec3d &minOutReferencePoint				最近距离时参考点云点
	 * 返回值  ：double											两份点云的最小距离
	 */
	static double GetMinHorizontalDistance(osg::Vec3d& minOutContrastPoint,
										   osg::Vec3d& minOutReferencePoint,
										   const std::vector<osg::Vec3d>& vContrastPoints,
										   const std::vector<osg::Vec3d>& vReferencePoints);

	/*
	 * 函数介绍：获取两份点云的最小垂直距离（高度差值）（static）
	 * 输入参数：const std::vector<osg::Vec3d> &vContrastPoints	比较点云
	 *			const std::vector<osg::Vec3d> &vReferencePoints	参考点云
	 * 输出参数：osg::Vec3d &minOutContrastPoint					最近距离时比较点云点
	 *			osg::Vec3d &minOutReferencePoint				最近距离时参考点云点
	 * 返回值  ：double											两份点云的最小距离
	 */
	static double GetMinVerticalDistance(osg::Vec3d& minOutContrastPoint,
										 osg::Vec3d& minOutReferencePoint,
										 const std::vector<osg::Vec3d>& vContrastPoints,
										 const std::vector<osg::Vec3d>& vReferencePoints);

	/*
	 * 函数介绍：获取两份点云的最小净空距离（两点的距离）（static）
	 * 输入参数：const std::vector<osg::Vec3d> &vContrastPoints	比较点云
	 *			const std::vector<osg::Vec3d> &vReferencePoints	参考点云
	 * 输出参数：osg::Vec3d &minOutContrastPoint					最近距离时比较点云点
	 *			osg::Vec3d &minOutReferencePoint				最近距离时参考点云点
	 * 返回值  ：double											两份点云的最小距离
	 */
	static double GetMinClearanceDistance(osg::Vec3d& minOutContrastPoint,
										  osg::Vec3d& minOutReferencePoint,
										  const std::vector<osg::Vec3d>& vContrastPoints,
										  const std::vector<osg::Vec3d>& vReferencePoints);

	/*
	 * 函数介绍：获取点云Z最小最大值对应的点（static）
	 * 输入参数：const std::vector<osg::Vec3d> &vPoints			点云
	 * 输出参数：osg::Vec3d &minZPoint							Z最小值对应的点
	 *			osg::Vec3d &maxZPoint							Z最大值对应的点
	 * 返回值  ：bool											是否获取到
	 */
	static bool GetMinMaxZPoint(osg::Vec3d& outMinZPoint,
								osg::Vec3d& outMaxZPoint,
								const std::vector<osg::Vec3d>& vPoints);

	/*
	 * 函数介绍：获取点云Z值最近的点（static）
	 * 输入参数：const std::vector<osg::Vec3d> &vPoints			点云
	 *			double dZ										Z参考值
	 * 输出参数：osg::Vec3d &outCloserZPoint						Z值最近的点
	 * 返回值  ：bool											是否获取到
	 */
	static bool GetCloserZPoint(osg::Vec3d& outCloserZPoint,
								const std::vector<osg::Vec3d>& vPoints,
								double dZ);

	/*
	 * 函数介绍：获取投影到Z为0的平面点（static）
	 * 输入参数：std::vector<osg::Vec3d> &vPoints				点云
	 * 输出参数：std::vector<osg::Vec3d> &vOutZeroZPoints		投影到Z为0的平面点
	 * 返回值  ：void
	 */
	static void GetZeroZPoints(std::vector<osg::Vec3d>& vOutZeroZPoints,
							   const std::vector<osg::Vec3d>& vPoints);
};

#endif // POINTCLOUDDISTANCEMEASURE_H_