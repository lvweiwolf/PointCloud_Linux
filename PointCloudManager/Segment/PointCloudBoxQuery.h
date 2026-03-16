//*****************************************************
//
//    @copyright      	三维技术部
//    @version        	v1.0
//    @file           	PointCloudBoxQuery.H
//    @author         	LC
//    @data           	2022/6/24 17:23
//    @brief          	点云包围盒查询工具
//*****************************************************
#ifndef POINT_CLOUD_BOX_QUERY_H_
#define POINT_CLOUD_BOX_QUERY_H_

#include <include/ICloudSegmentation.h>

#include <Segment/PointCloudManagerDefine.h>
#include <Segment/PointCloudManagerDef.h>

// 包围盒对应的点云节点列表
typedef std::map<pc::data::PointCloudBoundBox2D,
				 std::vector<pc::data::CModelNodePtr>,
				 pc::data::BoundBoxCompare>
	BoundToPointCloudPagedLodDic;

typedef std::map<pc::data::PointCloudBoundBox2D,
				 std::vector<pc::data::tagPagedLodFile>,
				 pc::data::BoundBoxCompare>
	BoundToFile; // 包围盒区域对应的文件列表

class CPointCloudBoxQuery
{
public:
	/**
	 *  函数介绍    	获取指定层级的pagedlod列表
	 *
	 *  输入参数    	d3s::share_ptr<CPointCloudElement> pointCloudElement		点云元素
	 *  输入参数    	int nLevel 指定层级(-1为叶子级节点) 输出参数
	 * std::vector<CPointCloudpagedLod> &pointCloudPagedLodList	指定层级的pagedlod列表 输出参数
	 *  返回值   	void
	 */
	static const int nAllLevel = -1; // 获取所有层级pagedlod子节点
	static void GetLevelPagedLodList(pc::data::CModelNodePtr pointCloudElement,
									 int nLevel,
									 std::vector<pc::data::CModelNodePtr>& pointCloudPagedLodList);

	/**
	 *  函数介绍    	获取指定层级的pagedlod列表
	 *
	 *  输入参数    	pc::data::CModelNodePtr pPagedLod					点云osg节点
	 *  输入参数    	int nLevel 指定层级(-1为叶子级节点) 输入参数
	 * std::vector<pc::data::CModelNodePtr> & pointCloudPagedLodList	指定层级的pagedlod列表
	 *  输出参数
	 *  返回值   	void
	 */
	static void GetSubLevelPagedLodList(
		pc::data::CModelNodePtr pPagedLod,
		int nLevel,
		std::vector<pc::data::CModelNodePtr>& pointCloudPagedLodList);

	/**
	 *  函数介绍    	获取叶子节点列表
	 *
	 *  输入参数    	pc::data::CModelNodePtr pPagedLod						当前pagedLod
	 *  输出参数    	std::vector<CPointCloudpagedLod> & pointCloudPagedLodList
	 * 叶子节点pagedLod 输出参数 返回值   	void
	 */
	static void GetLeafPagedLodList(pc::data::CModelNodePtr pPagedLod,
									std::vector<pc::data::CModelNodePtr>& pointCloudPagedLodList);

	/**
	 *  函数介绍    	初始化包围盒对应的文件列表
	 *
	 *  输入参数    	const std::vector<pc::data::CModelNodePtr> & pointCloudPagedLodList
	 * pagedLod列表 输出参数    	BoundToFile & boundBoxToFiles
	 * 包围盒对应的文件列表 输出参数 返回值   	void
	 */
	static void InitBoundBoxToFileDic(
		const std::vector<pc::data::CModelNodePtr>& pointCloudPagedLodList,
		BoundToFile& boundBoxToFiles);

	/**
	 * 计算包围盒内节点列表
	 * @param [in] pointCloudPagedLodList
	 * @param [in] boundToNodeList
	 * @param [in] boundBox
	 * @return
	 **/
	static void CalcValidNodeList(
		const std::vector<pc::data::CModelNodePtr>& pointCloudPagedLodList,
		std::vector<pc::data::CModelNodePtr>& validNodeList,
		std::vector<pc::data::CModelNodePtr>& validAllInNodeList,
		const pc::data::PointCloudBoundBox2D& boundBox);


	struct SMtReadParam
	{
		SMtReadParam() : eSegment(eAutoSegment), _vegetationTypes{}, nGroundType(0) {}
		IPointCloudPtr pPointCloud;								 // 点云数据
		std::vector<pc::data::CModelNodePtr> validNodeList;		 // 在范围边缘的节点
		std::vector<pc::data::CModelNodePtr> validAllInNodeList; // 范围内的节点
		std::map<int, int> mapConvertType;						 // 产品、平台分类类型映射
		std::map<char, bool> segShowTypeMap;					 // 分类使能
		pc::data::PointCloudBoundBox2D boundBox;				 // 待查询的包围盒
		ESegmentType eSegment;									 // 分类模式
		std::set<unsigned> _vegetationTypes;					 // 单木分割所需的植被类型索引
		unsigned nGroundType;									 // 单木分割所需的地面类型索引
		osg::BoundingBox boxList;								 // 当前划分的包围盒列表
	};
	/**
	 *  多线程筛选有效点集
	 *  @param [in] param	构建参数
	 *  @return
	 */
	static void MtReadPointsToPointCloud(const SMtReadParam& param);


	struct SMtSetParam
	{
		SMtSetParam(std::map<unsigned, osg::BoundingBox>& boxMap)
			: nDenoiseLevel(0),
			  eSegment(ESegmentType::eAutoSegment),
			  strPrjId(nullptr),
			  clusterBoxMap(boxMap)
		{
		}

		IPointCloudPtr pPointCloud;								 // 点云数据
		std::vector<pc::data::CModelNodePtr> validNodeList;		 // 在范围边缘的节点
		std::vector<pc::data::CModelNodePtr> validAllInNodeList; // 范围内的节点
		std::vector<osg::Vec3d> vecSelectPonits;				 // 多边形屏幕点
		osg::BoundingBox boundingBox;		 // 范围包围盒（目前用于单木分割，通过包围盒确定范围）
		osg::Matrix vpwMatrix;				 // 屏幕vpw矩阵
		std::map<int, int> mapConvertType;	 // 产品、平台分类类型映射
		std::map<char, bool> segShowTypeMap; // 分类使能
		pc::data::PointCloudBoundBox2D boundBox;			 // 包围盒对应的文件列表
		unsigned nDenoiseLevel;								 // 降噪等级
		ESegmentType eSegment;								 // 分类模式
		std::map<unsigned, osg::BoundingBox>& clusterBoxMap; // 待查询的包围盒
		LPCTSTR strPrjId;									 // 工程id
		std::set<int> clearTypeCluster;						 // 清理对应类型下的簇
		std::set<unsigned> clearCluster;					 // 节点中已清理的簇id
	};
	/**
	 * 同步PointCloud的分类信息到点云节点中
	 * @param [in] param	构建参数
	 * @return
	 **/
	static void MtSetPointsCloudToPoints(SMtSetParam& param);
};

#endif // POINT_CLOUD_BOX_QUERY_H_
