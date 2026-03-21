/*---------------------------------------------------------------------
*文件名称：PCAutoSegmentToolkit.h
*功能描述：点云自动分类工具集
*创建标识：陶子龙2022.7.20.
*
*修改标识：
*修改描述：
----------------------------------------------------------------------*/
#ifndef PCAUTOSEGMENTTOOLKIT_H_
#define PCAUTOSEGMENTTOOLKIT_H_

#include <include/PointCloudManagerExport.h>
#include <include/PointCloudSegStruct.h>

#include <Segment/PointCloudManagerDefine.h>
#include <Segment/AutoSegmentFileLoadSaveThread.h>

#include <Tool/XmlDocument.h>

class POINTCLOUDMANAGER_EXPORT CPCAutoSegmentToolkit
{
public:
	/*
	 * 函数介绍：点云分类
	 * 输入参数：vPointCloudElements 即将分类的点云数据
	 * 输入参数：segmentParam 点云分类参数
	 * 输入参数：pProgressCallBack 分类进度条回调
	 * 输入参数： mapConvertType 类别转换
	 * 返回值：void
	 */
	static void Segment(const pc::data::CModelNodeVector& vPointCloudElements,
						pc::data::SegmentParam segmentParam,
						std::map<unsigned, osg::BoundingBox>& clusterBoxMap,
						const std::map<int, int>& mapConvertType = std::map<int, int>(),
						ESegmentType eSegment = eAutoSegment,
						const std::set<unsigned>& vegetationTypes = {},
						const unsigned& nGroundType = 0);

	/*
	 * 函数介绍：单木分割
	 * 输入参数：pcWrapperList		点云数据
	 * 输入参数：boundingBox		范围包围盒
	 * 输入参数：nVegetationType	植被类型
	 * 输入参数：nGroundType		地面类型
	 * 输入参数：clusterBoxMap		簇包围盒列表
	 * 返回值：void
	 */
	static bool TreeIndividual(const pc::data::CModelNodeVector& pcElementList,
							   const osg::BoundingBox& boundingBox,
							   const std::set<unsigned>& vegetationTypes,
							   const unsigned& nGroundType,
							   std::map<unsigned, osg::BoundingBox>& clusterBoxMap);

	/*
	 * 函数介绍：计算点云包围盒
	 *  输入参数    	vOldPointCloudElements	即将分类的点云数据
	 *  输入参数    	segmentParam	分类参数
	 *  输入参数    	nPointSize 一个包围盒点的数量
	 *  输入参数    	vBoundingBoxs 点云包围盒
	 *  输出参数     vPointCloudElements 在框选范围中的点云元素
	 * 返回值  ：void
	 */
	static void ComputeCloudBounds(const pc::data::CModelNodeVector& vOldPointCloudElements,
								   const pc::data::SegmentParam& segmentParam,
								   size_t nPointSize,
								   std::vector<osg::BoundingBox>& vBoundingBoxs,
								   pc::data::CModelNodeVector& vPointCloudElements);

	/*
	 * 函数介绍：计算点云包围盒
	 *  输入参数    	vOldPointCloudElements	即将分类的点云数据
	 *  输入参数    	segmentParam	分类参数
	 *  输入参数    	nPointSize 一个包围盒点的数量
	 *  输入参数    	vBoundingBoxs 点云包围盒
	 *  输出参数     vPointCloudElements 在框选范围中的点云元素
	 *  输出参数     eSegmentType 分类类型
	 * 返回值  ：void
	 */
	static void ComputeCloudBounds(const pc::data::CModelNodeVector& vOldPointCloudElements,
								   const pc::data::SegmentParam& segmentParam,
								   size_t nPointSize,
								   std::vector<osg::BoundingBox>& vBoundingBoxs,
								   pc::data::CModelNodeVector& vPointCloudElements,
								   ESegmentType eSegmentType);

	/*
	 * 函数介绍：杆塔偏移
	 *  输出参数    	segmentParam	分类参数
	 *  输入参数    	boxList 分块的包围盒列表
	 * 返回值  ：void
	 */
	static void TowerOffect(pc::data::SegmentParam& segmentParam, const osg::BoundingBox& boxList);

protected:
	/*
	 * 函数介绍：点云分割处理
	 * 输入参数：pThread	执行读写操作的线程
	 * 输入参数： segmentParam	点云分类参数
	 * 输入参数： nProgressStep	进度条最大进度
	 * 输入参数： pProgressCallBack	进度条回调
	 * 输入参数： mapConvertType	类别转换信息
	 * 返回值：void
	 */
	static void SegmentProgress(CAutoSegmentFileLoadSaveThread* pThread,
								const std::vector<osg::BoundingBox>& vDataBounds,
								pc::data::SegmentParam& segmentParam,
								const std::map<int, int>& mapConvertType,
								ESegmentType eSegment);

	/*
	 * 函数介绍：克隆点云模型
	 * 输入参数：const pc::data::CModelNodeVector &vPointCloudElements
	 * 待克隆的点云模型列表 输出参数：void 返回值  ：std::vector<pc::data::CModelNodePtr>
	 * 克隆的点云模型列表
	 */
	static pc::data::CModelNodeVector ClonePointCloudElements(
		const pc::data::CModelNodeVector& vPointCloudElements);

	/*
	 * 函数介绍：转化包围盒内的点云
	 * 输入参数：CAutoSegmentFileLoadSaveThread *pThread				执行读写操作的线程
	 *			const osg::BoundingBox &boundingBox2D				包围盒
	 * 输出参数：void
	 * 返回值  ：IPointCloudPtr										IPointCloud点云分类所需的点云
	 */
	static IPointCloudPtr ConvertPointCloud(CAutoSegmentFileLoadSaveThread* pThread,
											const pc::data::PointCloudBoundBox2D& boundingBox2D);


	/*
	 * 函数介绍：合并点云包围盒
	 *  输入参数 boundToPointNum 包围盒
	 *  输入参数 segmentParam 分类参数
	 *  输入/出参数  vBoundingBoxs 包围盒
	 *  输入参数   nPointSize 一个包围盒点的数量
	 *  输入参数   boundingBox 范围包围盒
	 *  输入参数   vecTowerPos 杆塔坐标
	 * 返回值：void
	 */
	static void MergeBoundingBoxs(pc::data::PointCloudBoundToPointNum& boundToPointNum,
								  std::vector<osg::BoundingBox>& vBoundingBoxs,
								  size_t nPointSize,
								  const osg::BoundingBox& boundingBox,
								  std::vector<osg::Vec3d> vecTowerPos);

	/*
	 * 函数介绍：点云自动分类实现
	 * 输入参数：segmentParam			分类参数
	 * 输入参数： pProgressCallBack		进度条回调
	 * 返回值  ：bool					点云自动分类实现结果
	 */
	static bool SegmentInternel(const pc::data::SegmentParam& segmentParam,
								IPointCloudPtr pCloud,
								ESegmentType eSegment);

	/*
	 * 函数介绍：点云地面自动分类实现
	 * 输入参数：toolkit::CXmlElement *pRoot					配置文件根节点
	 *			const CString &strCfgPath					配置文件路径
	 *			const CString &strTopographicFeatures		地形特征
	 *			IPointCloudPtr pCloud						待分类的点云
	 * 输入参数： pProgressCallBack		进度条回调
	 * 输出参数：void
	 * 返回值  ：bool										点云自动分类实现结果
	 */
	static bool TopographicSegmentInternel(toolkit::CXmlElement* pRoot,
										   const CString& strCfgPath,
										   const CString& strTopographicFeatures,
										   IPointCloudPtr pCloud);

	/*
	 * 函数介绍：点云电力线自动分类实现
	 * 输入参数：toolkit::CXmlElement *pRoot					配置文件根节点
	 *			const CString &strCfgPath					配置文件路径
	 *			const pc::data::SegmentParam &segmentParam 分类参数
	 *			IPointCloudPtr pCloud						待分类的点云
	 * 输入参数： pProgressCallBack		进度条回调
	 * 输出参数：void
	 * 返回值  ：bool										点云自动分类实现结果
	 */
	static bool PowerCorridorsSegmentInternel(toolkit::CXmlElement* pRoot,
											  const CString& strCfgPath,
											  const pc::data::SegmentParam& segmentParam,
											  IPointCloudPtr pCloud);

	/*
	 * 函数介绍：点云环境自动分类实现
	 * 输入参数：toolkit::CXmlElement *pRoot					配置文件根节点
	 *			const CString &strCfgPath					配置文件路径
	 *			IPointCloudPtr pCloud						待分类的点云
	 * 输出参数：void
	 * 返回值  ：bool										点云自动分类实现结果
	 */
	static bool EnvironmentsSegmentInternel(toolkit::CXmlElement* pRoot,
											const CString& strCfgPath,
											IPointCloudPtr pCloud);

	/*
	 * 函数介绍：点云单木分割分类实现
	 * 输入参数：toolkit::CXmlElement *pRoot					配置文件根节点
	 *			const CString &strCfgPath					配置文件路径
	 *			IPointCloudPtr pCloud						待分类的点云
	 *			ISegmentProgressPtr pProgress				点云自动分类进度条
	 * 输出参数：void
	 * 返回值  ：bool										点云自动分类实现结果
	 */
	static bool TreeIndividualSegmentInternel(toolkit::CXmlElement* pRoot,
											  const CString& strCfgPath,
											  IPointCloudPtr pCloud);

	/*
	 * 函数介绍：点云自动分类实现
	 * 输入参数：IPointCloudPtr pCloud						待分类的点云
	 *			d3s::pcs::SegmentationType eSegmentationType分类类型
	 *			const CString &strCfgPath					自动分类配置文件路径
	 *		    const std::vector<osg::Vec3d>& vecTowerPoints   杆塔坐标
	 * 输出参数：void
	 * 返回值  ：bool										点云自动分类实现结果
	 */
	static bool SegmentInternel(
		IPointCloudPtr pCloud,
		d3s::pcs::SegmentationType eSegmentationType,
		const CString& strCfgPath,
		const std::vector<osg::Vec3d>& vecTowerPoints = std::vector<osg::Vec3d>());

private:
	/**
	 *  函数介绍    	获取点云瓦片信息
	 *  输入参数		boundingBox 范围包围盒
	 *  输入参数		vecTowerPos 杆塔坐标
	 *  输入参数    	const pc::data::PointCloudBoundToPointNum & boundToPointNum
	 * 包围盒对应的点计数 输出参数    	osg::Vec3d & centerPt						中心点 输出参数
	 * size_t & nTotalSize							总得点数 输出参数    	std::vector<osg::Vec3d>
	 * & vTileCentroids	瓦片/包围盒中心列表 输出参数    	std::vector<osg::BoundingBox> &
	 * vTileBounds	瓦片/包围盒列表 输出参数    	std::vector<size_t> & vPointSizes
	 * 包围盒点大小集合 返回值   	void
	 */
	static void GetTileInfo(const osg::BoundingBox& boundingBox,
							std::vector<osg::Vec3d> vecTowerPos,
							const pc::data::PointCloudBoundToPointNum& boundToPointNum,
							osg::Vec3d& centerPt,
							size_t& nTotalSize,
							std::vector<osg::Vec3d>& vTileCentroids,
							std::vector<osg::BoundingBox>& vTileBounds,
							std::vector<size_t>& vPointSizes);

private:
	static IValueBufferPtr _roadVectorize; // 道路矢量加载文件，保存仅加载一次
};

#endif // PCAUTOSEGMENTTOOLKIT_H_