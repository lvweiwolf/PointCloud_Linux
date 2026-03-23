#ifndef PCALGORITHM_TOOLKIT_H_
#define PCALGORITHM_TOOLKIT_H_

#include <include/PointCloudManagerExport.h>

#include <Segment/PointCloudManagerDef.h>

class POINTCLOUDMANAGER_EXPORT CPCAlgorithmToolkit
{
public:
	/**
	 * 仅检测范围内是否存在最近点
	 * @param [in] pcElements    点云数据
	 * @param [in] travelInfo
	 * @param [in] result
	 * @return
	 */
	static bool CheckInRangePointCloud(const pc::data::CModelNodeVector& pcElements,
									   const pc::data::tagTravelInfo& travelInfo,
									   pc::data::tagPointIndex& result);

	/**
	 * 求一个点到线段的最近距离
	 * @param [in] C
	 * @param [in] A
	 * @param [in] B
	 * @param [in] nearestPoint
	 * @param [in]  bool& bIn 投影点是否在内部
	 * @return 点C到线段AB的最近距离；nearestPoint是线段AB上距离点C最近的点
	 */
	template <class T>
	static float PointToLineSegmentLength2(const T& C,
										   const T& A,
										   const T& B,
										   T& nearestPoint,
										   bool& bIn)
	{
		T ac = C - A;
		T ab = B - A; // 线段所在直线向量

		float f = ac * ab;
		if (f < 0) // C点在AB上的投影位于A的左侧
		{
			nearestPoint = A;
			bIn = false;
			return ac.length2();
		}
		else if (f > ab * ab) // C点在AB上的投影位于B的右侧
		{
			nearestPoint = B;
			bIn = false;
			return (B - C).length2();
		}
		else // C点在AB上的投影位于线段AB内部
		{
			float abLen2 = ab.length2(); // ab长度的平方
			nearestPoint = A;
			if (abLen2 - 0.0f > EPSILON) // ab长度不等于0，亦即A、B两点不重合
			{
				nearestPoint += ab * (f / abLen2);
			}
			bIn = true;
			return (nearestPoint - C).length2();
		}
	}

	/*
	 * 函数介绍：计算两份点云的最小距离（static）
	 * 输入参数：EDistanceType eMeasureType						距离的类型
	 *			const std::vector<pc::Vec3d> &vContrastPoints	比较点云
	 *			const std::vector<pc::Vec3d> &vReferencePoints	参考点云
	 * 输出参数：pc::Vec3d &minOutContrastPoint					最近距离时比较点云点
	 *			pc::Vec3d &minOutReferencePoint				最近距离时参考点云点
	 * 返回值  ：double											两份点云的最小距离
	 */
	static double GetMinDistance(osg::Vec3d& minOutContrastPoint,
								 osg::Vec3d& minOutReferencePoint,
								 const std::vector<osg::Vec3d>& vContrastPoints,
								 const std::vector<osg::Vec3d>& vReferencePoints,
								 const pc::data::EMeasureType& measureType);

	/*
	 * 函数介绍：快速获取两份大点云的距离（通常指百万级点云的距离）
	 * 输入参数：const std::vector<pc::Vec3d> &minContrastPt	比较点云
	 *			const std::vector<pc::Vec3d> &vReferencePoints	参考点云
	 *			double dReferenceThin	参考点云的抽稀间距(米)
	 *       	double dContrastThin	比较点云的抽稀间距（米）
	 * 输出参数：IPointCloudDistanceMeasureService::MeasureResult &measureResul
	 *最短距离返回结果 返回值  ：double 两份点云的最小距离
	 */
	static double QuickGetMinClearanceDistance(pc::data::MeasureResult& measureResult,
											   const std::vector<osg::Vec3d>& vContrastPts,
											   const std::vector<osg::Vec3d>& vReferencePts,
											   double dReferenceThin,
											   double dContrastThin);



	/**
	 * 判断配置文内的计算树木信息系数是否有效(L>0、0<M<1、0<N<1)
	 * @return
	 */
	static bool IsValidCalTreeInfoCoefficient();

	/**
	 * 计算树木信息
	 * @param [in] pcElements	点云数据
	 * @param [in] boundingBox	范围包围盒
	 * @param [in] clusterBoxMap	簇对应的包围盒
	 * @param [out] treeInfoMap	树木信息
	 * @return
	 */
	static bool ComputeTreeInfo(const pc::data::CModelNodeVector& pcElements,
								const std::map<unsigned, osg::BoundingBox>& clusterBoxMap,
								std::map<unsigned, pc::data::treeInfo>& treeInfoMap);

	static void ComputeClusterArea(const pc::data::CModelNodeVector& pcElements,
								   const std::vector<std::pair<int, osg::BoundingBox>>& clusterIDs,
								   std::vector<double>& areas);


	/**
	 * 计算区域内点云密度
	 * @param [in] pcElements			点云包装对象列表
	 * @param [in] polygonParam			多边形区域参数
	 * @param [out] dPolygonArea			多边形面积
	 * @param [out] dDensity				密度
	 * @param [out] dPtMaxHeightDiffer	最大高差
	 * @param [in] bNeewHigHlight		是否需要高亮
	 * @return
	 */
	static bool ComputeDensity(const pc::data::CModelNodeVector& pcElements,
							   const pc::data::SPolygonParam& polygonParam,
							   double& dPolygonArea,
							   double& dDensity,
							   double& dPtMaxHeightDiffer,
							   const bool bNeewHigHlight = false);

	/**
	 * 创建图片
	 * @param [in] colorList		二维数组颜色列表（0-1）
	 * @param [in] strPath		路径
	 * @param [in] bSameColor	是否相同颜色；相同颜色需提供宽、高
	 * @param [in] nWidth		宽
	 * @param [in] nHeight		高
	 * @return
	 */
	static bool CreateImage(const std::vector<std::vector<osg::Vec4>>& colorList,
							const CString& strPath,
							bool bSameColor = false,
							int nWidth = 0,
							int nHeight = 0);

	/**
	 * 聚类
	 * @param [in] pcElements		点云
	 * @param [out] clusterList		拟合簇
	 * @param [in] nType				拟合类型（需拟合的分类类型）
	 * @param [in] vpwMatrix			聚类距离阈值
	 * @param [in] selectPointList	多边形屏幕点
	 * @param [in] bFastExecute		是否快速执行（快速执行误差较大）
	 * @param [in] nMinPointSize		最小聚类点个数
	 * @return
	 */
	static bool EuclideanCluster(const pc::data::CModelNodeVector& pcElements,
								 std::vector<std::vector<osg::Vec3d>>& clusterList,
								 const unsigned& nType,
								 const osg::Matrix& vpwMatrix,
								 const std::vector<osg::Vec3d>& selectPointList,
								 const bool& bFastExecute = false,
								 const unsigned& nMinPointSize = 3);

	/**
	 * 全局快速聚类
	 * @param [in] pcElements		点云
	 * @param [in] nType			聚类类型
	 * @param [in] resultBoxs	聚类结果包围盒
	 * @return
	 */
	static bool GlFastEuclidean(const pc::data::CModelNodeVector& pcElements,
								const unsigned& nType,
								std::vector<osg::BoundingBox>& towerBoxs);

	/**
	 * 计算点云包围盒方向
	 * @param [in] pcElements		点云
	 * @param [in] startPt		起点
	 * @param [in] pcBoxDir		包围盒方向
	 * @return
	 */
	static bool CalculatePointCloudDir(
		const pc::data::CModelNodeVector& pcElements,
		const osg::Vec3& startPt,
		std::vector<std::pair<osg::BoundingBox, osg::Vec3>>& pcBoxDir);

	struct SPowerParam
	{
		SPowerParam() : _dTowerR(0) {}
		SPowerParam(const osg::Vec3& pt, double dR) : _towerPt(pt), _dTowerR(dR) {}
		osg::Vec3 _towerPt;
		double _dTowerR;
	};
	/**
	 * 计算线路走向（通过距离最近方法判断下一杆塔）
	 * @param [in] towerPts		杆塔集合
	 * @param [in] startPt		线路起点
	 * @param [in] dMaxSpan		最大跨度
	 * @return
	 */
	static bool CalculatePowerLineDir(std::vector<SPowerParam>& towerPts,
									  const osg::Vec3d& startPt,
									  const double& dMaxSpan);

	/**
	 * 拟合聚类
	 * @param [in] cluster	聚类簇
	 * @param [out] curve	拟合曲线
	 * @param [in] dT		拟合间距
	 * @return
	 */
	static bool ApproxCluster(const std::vector<osg::Vec3d>& cluster,
							  std::vector<osg::Vec3d>& curve,
							  const double& dT = 0.01);

	static std::vector<int> Approx(const std::vector<osg::Vec3d>& cluster,
								   const std::vector<osg::Vec3d>& points,
								   const double& dHresholsd);
	/**
	 * 点投影到地面
	 * @param [in] pcElements 点云元素
	 * @param [in] vecPnt 待投影的点
	 * @return
	 */
	static std::vector<osg::Vec3d> GetPointsProjectionPlace(
		const pc::data::CModelNodeVector& pcElements,
		const std::vector<osg::Vec3d>& vecPnt);

	/**
	 * 计算节点包围盒大小
	 * @param [in] pageLOD		图元
	 * @param [in] boundingBox	包围盒
	 * @param [in] nPointCount	点个数
	 * @return
	 */
	static bool ComputeBoundingBox(pc::data::CModelNodePtr pageLOD,
								   osg::BoundingBox& boundingBox,
								   size_t& nPointCount);

	/**
	 * 计算点云之间的最小距离
	 * @param [in] vContrastPoints
	 * @param [in] vReferencePoints
	 * @return
	 */
	static float ComputeMinDistance(const osg::Vec3d& point,
									const std::vector<osg::Vec3d>& vecPoints,
									osg::Vec3d& closestPoint);

	/**
	 * 计算点云到线段的最短距离
	 * @param [in] point
	 * @param [in] vecPoints
	 * @param [in] closestPoint
	 * @return
	 */
	static float ComputeMinDistance(const osg::Vec3d& lineStart,
									const osg::Vec3d& lineEnd,
									const std::vector<osg::Vec3d>& vecPoints,
									osg::Vec3d& closestPoint);


	/**
	 * 计算多边形范围内的的地面点云TIN面积（调用者需控制多边形范围，减少内存峰值开销）
	 * @param [in] pcElements	点云
	 * @param [in] polygnPnts		多边形范围（空间二维坐标，忽略Z值）
	 * @param [in] nGroundType		地形类型
	 * @return 面积
	 */
	static double CalculateGroundTinArea(const pc::data::CModelNodeVector& pcElements,
										 const std::vector<osg::Vec3d>& polygnPnts,
										 int nGroundType);

	/**
	 * 合并多边形
	 * @param [in] polygon1	多边形
	 * @param [in] polygon2	多边形
	 * @return
	 */
	static std::vector<osg::Vec3d> MergePolygons(const std::vector<osg::Vec3d>& polygon1,
												 const std::vector<osg::Vec3d>& polygon2);

	/**
	 * 多边形求差
	 * @param [in] polygon1	多边形
	 * @param [in] polygon2	多边形
	 * @return
	 */
	static std::vector<osg::Vec3d> DifferencePolygons(const std::vector<osg::Vec3d>& polygon1,
													  const std::vector<osg::Vec3d>& polygon2);

	/**
	 * 计算多边形面积
	 * @param [in] polygon	多边形
	 * @return
	 */
	static double CalPolygonArea(const std::vector<osg::Vec3d>& polygon);

	/**
	 * 多边形求交
	 * @param [in] polygon1	多边形
	 * @param [in] polygon2	多边形
	 * @return
	 */
	static bool IntersectsPolygons(const std::vector<osg::Vec3d>& polygon1,
								   const std::vector<osg::Vec3d>& polygon2);

	static bool PolygonIntersectWithBox(const std::vector<osg::Vec3d>& poly,
										const osg::BoundingBox& bbox);

	/**
	 * 计算点云的面积(三角片化)
	 * @param [in] pointList	点云
	 * @return 面积
	 */
	static double CalculatePCTinArea(const std::vector<osg::Vec3d>& pointList);

	/**
	 * 计算簇网格划分后的高度
	 * @param [in] pProject 工程对象
	 * @param [in] items 点云节点
	 * @param [in] clusterIDBoxs 簇ID
	 * @param [in] dGridWidth 网格宽度
	 * @param [out] heights 高度
	 * @return
	 */
	static void ComputeClusterHeightByGrid(
		const pc::data::CModelNodeVector& pcElements,
		const std::vector<std::pair<int, osg::BoundingBox>>& clusterIDBoxs,
		double dGridWidth,
		std::vector<double>& heights);

	/**
	 * 开启post服务接口（阻塞主线程，用于后台命令，目前未完善退出机制）
	 * @param [in] PostCallback 接口回调响应函数模板，返回响应结果
	 * @param [in] strBodyData 接口参数
	 * @param [in] nPort 端口号
	 * @param [in] CallbackFun 接口回调响应
	 * @return
	 */
	typedef bool (*PostCallback)(const CString& strBodyData, CString& strResult);
	static void StartPostApi(unsigned nPort, const CString& strApiName, PostCallback pCallbackFun);
};


#endif // PCALGORITHM_TOOLKIT_H_