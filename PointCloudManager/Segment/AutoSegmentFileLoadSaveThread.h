/*---------------------------------------------------------------------
*文件名称：AutoSegmentFileLoadSaveThread.h
*功能描述：自动分类文件处理线程
*创建标识：陶子龙2022.7.27.
*
*修改标识：
*修改描述：
----------------------------------------------------------------------*/
#ifndef AUTO_SEGMENT_FILE_LOAD_SAVE_THREAD_H_
#define AUTO_SEGMENT_FILE_LOAD_SAVE_THREAD_H_

#include <Segment/PointCloudManagerDefine.h>
#include <Segment/PointCloudManagerDef.h>

#include <include/ICloudSegmentation.h>

#include <OpenThreads/Thread>

struct SSegmentThreadParam
{
	SSegmentThreadParam(std::map<unsigned, osg::BoundingBox>& clusterBoxMap)
		: _nDenoiseLevel(0),
		  _eSegment(eAutoSegment),
		  _clusterBoxMap(clusterBoxMap),
		  _vegetationTypes{}
	{
	}

	// std::vector<d3s::share_ptr<CPointCloudElement>> _vPointCloudElements;	// 点云图元
	std::vector<pc::data::CModelNodePtr> _vPointCloudElements;
	std::vector<osg::BoundingBox> _vBoundingBoxs;		  // 包围盒列表
	std::map<char, bool> _showTypeMap;					  // 分类使能
	std::map<int, int> _mapConvertType;					  // 产品——平台分类映射
	unsigned _nDenoiseLevel;							  // 降噪等级
	ESegmentType _eSegment;								  // 分类模式
	std::map<unsigned, osg::BoundingBox>& _clusterBoxMap; // 簇对应的包围盒
	std::set<int> _clearTypeCluster;					  // 清理对应类型下的簇
	std::set<unsigned> _vegetationTypes;				  // 单木分割所需的植被类型索引
	unsigned _nGroundType;								  // 单木分割所需的地面类型索引
};

///////////////////////////////////////////////////////////////////自动分类文件处理线程///////////////////////////////////////////////////////////////////
class CAutoSegmentFileLoadSaveThread : public OpenThreads::Thread
{
public:
	CAutoSegmentFileLoadSaveThread(SSegmentThreadParam& param);
	~CAutoSegmentFileLoadSaveThread(void);

public:
	/*-----------------------------------------------------------------------线程执行-----------------------------------------------------------------------*/
	/*
	 * 函数介绍：线程执行（override）
	 * 输入参数：void
	 * 输出参数：void
	 * 返回值  ：void
	 */
	virtual void run(void) override;

	/*
	 * 函数介绍：中断线程执行
	 * 输入参数：void
	 * 输出参数：void
	 * 返回值  ：void
	 */
	void Stop(void);

	/*-------------------------------------------------------------------------查询-------------------------------------------------------------------------*/
	/**
	 * 查询目标点
	 * @param [in] vPointCloudElements
	 * @param [in] boundBox
	 * @return
	 **/
	IPointCloudPtr QueryPoints(osg::BoundingBox boundBox);
	/*
	 * 函数介绍：获取查询的点
	 * 输入参数：const pc::data::PointCloudBoundBox2D &boundBox		查询的包围盒
	 * 返回值  ：IPointCloudPtr 点云数据
	 */
	IPointCloudPtr GetQueryPoints(const pc::data::PointCloudBoundBox2D& boundBox);

	/*-------------------------------------------------------------------------修改-------------------------------------------------------------------------*/
	/*
	 * 函数介绍：设置修改点集分类（包括第零层级的第一层级）
	 * 输入参数：const pc::data::PointCloudBoundBox2D &boundBox						修改的包围盒
	 *			const std::vector<pc::data::PointInfo> &vPointInfos					修改点云信息
	 * 输出参数：void
	 * 返回值  ：void
	 */
	void SetModifyPoints(const pc::data::PointCloudBoundBox2D& boundBox);


	/*
	 * 函数介绍：设置框选参数
	 * 输入参数：vecSelectPonits 框选点集
	 * 输入参数：boundingBox		范围包围盒（目前用于单木分割，通过包围盒确定范围）
	 * 输入参数：vpwMatrix vpw变换矩阵
	 * 返回值  ：void
	 */
	void SetPolygonParam(const std::vector<osg::Vec3d>& vecSelectPonits,
						 const osg::BoundingBox& boundingBox,
						 const osg::Matrix& vpwMatrix);

	/**
	 * 获取清理的簇（克隆节点内的簇属性已清理，但簇管理器内的处对象未删除，返回又外部删除，防止通过进度条中途取消操作）
	 * @return
	 */
	std::set<unsigned> GetClearCluster() { return _clearCluter; }

protected:
	/*-------------------------------------------------------------------------文件-------------------------------------------------------------------------*/
	/*
	 * 函数介绍：修改点集分类（包括第零层级的第一层级）
	 * 输入参数：void
	 * 输出参数：void
	 * 返回值  ：void
	 */
	virtual void ModifyPointsClassify(void);

protected:
	std::vector<pc::data::CModelNodePtr> _vReadValidLods;		// 合法的Lod节点
	std::vector<pc::data::CModelNodePtr> _vWriteValidLods;		// 合法的Lod节点
	std::vector<pc::data::CModelNodePtr> _vReadAllInValidLods;	// 合法的Lod节点（全在包围盒中）
	std::vector<pc::data::CModelNodePtr> _vWriteAllInValidLods; // 合法的Lod节点（全在包围盒中）
	IPointCloudPtr _pReadedPoints;								// 查询点结果
	IPointCloudPtr _pModifyPoints;								// 修改点信息
	pc::data::PointCloudBoundBox2D _queryBoundBox;				// 已查询的包围盒列表
	pc::data::PointCloudBoundBox2D _modifyBoundBox;				// 修改点云的包围框
	int _nCompleteQueryCount;									// 完成查询包围盒点的数量
	int _nCompleteModifyCount;									// 完成修改的数量
	bool _bCompleteQuery;										// 是否完成修改
	bool _bModifyPointCloud;									// 是否修改点云元素
	bool _bStop;												// 是否中断线程执行
	bool _bPolygon;												// 框选标记
	std::vector<osg::Vec3d> _vecSelectPoints;					// 框选点集
	osg::Matrix _vpwMatrix;										// vpw变换矩阵
	SSegmentThreadParam& _param;								// 分类线程参数
	std::set<unsigned> _clearCluter;							// 清理的簇
	osg::BoundingBox _boundingBox;								// 范围包围盒
	std::vector<pc::data::CModelNodePtr> _vTinyPagedLods;		// 所有层级的page
	bool _bReadTinyPagedLods;									// 是否已读取_vTinyPagedLods
};

#endif // AUTO_SEGMENT_FILE_LOAD_SAVE_THREAD_H_