//*****************************************************
//    
//    @copyright      	三维技术部
//    @version        	v1.0
//    @file           	PagedLodPointsVisitor.H
//    @author         	LC
//    @data           	2022/6/23 16:30
//    @brief          	点云pagedlod点数据遍历器
//*****************************************************
#ifndef PAGEDLODPOINTSVISITOR_H_
#define PAGEDLODPOINTSVISITOR_H_

#include <Segment/PointCloudManagerDefine.h>
#include <Segment/PointCloudManagerDef.h>

#include <include/ICloudSegmentation.h>

class CPointReadWriter;
class CPointReadWriterVisitor : public osg::NodeVisitor
{
public:
	CPointReadWriterVisitor(CPointReadWriter *pWriter, bool bReadWrite);
	~CPointReadWriterVisitor();

	/**
	 * 设置多边形参数
	 * @param [in] vecSelectPonits
	 * @param [in] vpwMatrix
	 * @return
	 **/
	void SetPolygonParam();

protected:
	virtual void apply(osg::Geometry& geometry) override;

protected:
	/**
	* 设置单木分割分类类型
	* @param [in] pTexCoordArray	纹理数组
	* @param [in] texIndex			纹理索引
	* @param [in] pointCloudIndex	PointCloud数组索引
	* @return
	*/
	void SetPointCloudIndividualClass(osg::ref_ptr<osg::Vec2Array> pTexCoordArray, size_t texIndex, size_t pointCloudIndex);

	/**
	* 写入单木分割信息
	* @param [in] point	顶点
	* @param [in] pTexCoordArray 纹理
	* @param [in] texIndex 纹理索引
	* @param [in] pointCloudIndex PointCloud索引
	* @return
	*/
	void WriteTreeIndividual(const osg::Vec3& point, osg::ref_ptr<osg::Vec2Array> pTexCoordArray, size_t texIndex
		, size_t pointCloudIndex);
	
	/**
	* 写入分类信息
	* @param [in] point	顶点
	* @param [in] screenPoint 顶点投影屏幕点
	* @param [in] pTexCoordArray 纹理
	* @param [in] texIndex 纹理索引
	* @param [in] pointCloudIndex PointCloud索引
	* @return
	*/
	void WriteSetSegment(const osg::Vec3& screenPoint, osg::ref_ptr<osg::Vec2Array> pTexCoordArray, size_t texIndex, size_t pointCloudIndex);

	/**
	* 无包围盒的读写
	* @param [in] vertArray			顶点数组
	* @param [in] matrix			矩阵
	* @param [in] pTexCoordArray	纹理数组
	* @return
	**/
	void ReadWriteNoBound(osg::ref_ptr<osg::Vec3Array>& vertArray, osg::Matrix& matrix, const osg::Vec3d& boxCenter , osg::ref_ptr<osg::Vec2Array> pTexCoordArray);

	/**
	* 读取点
	* @param [in] vertArray			顶点数组
	* @param [in] matrix			矩阵
	* @param [in] pTexCoordArray	纹理数组
	* @return
	*/
	void WritePoint(osg::ref_ptr<osg::Vec3Array>& vertArray, osg::Matrix& matrix, osg::ref_ptr<osg::Vec2Array> pTexCoordArray);

	/**
	* 带包围盒的读写
	* @param [in] point		点
	* @param [in] geometry	pTexCoordArray
	* @param [in] i			索引
	* @return
	**/
	virtual void ReadWriteByBound(osg::Vec3& point, osg::ref_ptr<osg::Vec2Array> pTexCoordArray,const osg::Vec3d& boxCenter ,size_t i);

	/**
	* 根据treeId生成簇
	* @param [in] pTexCoordArray	纹理
	* @param [in] nVertexIndex		顶点索引
	* @param [in] treeId			树id
	* @param [in] point				顶点
	* @return
	*/
	void CreateCluster(osg::ref_ptr<osg::Vec2Array> pTexCoordArray, size_t nVertexIndex, unsigned treeId, const osg::Vec3& point);

protected:
	CPointReadWriter*					_pWriter;			// 读写器
	bool								_bReadWrite;		// 读取或写入（true标识读取，false表示写入）
	bool								_bPolygon;			// 框选标记
	//d3s::share_ptr<CClusterManager>		_pClusterManager;	// 簇管理器
};

//struct SClusterBoxMap : public d3s::ReferenceCountObj
//{
//	SClusterBoxMap(std::map<unsigned, osg::BoundingBox>& clusterBoxList) :_clusterBoxMap(clusterBoxList) {}
//	std::map<unsigned, osg::BoundingBox>&	_clusterBoxMap;
//	toolkit::CCriticalSectionHandle			_CSC;
//};
//
//struct STreeClusterMap : public d3s::ReferenceCountObj
//{
//	//std::map<int, d3s::share_ptr<CClusterItem>>	_treeClusterMap;
//};
//
//struct SClearCluster : public d3s::ReferenceCountObj
//{
//	SClearCluster(std::set<unsigned>& clearCluster) :_clearCluster(clearCluster) {}
//	std::set<unsigned>&	_clearCluster;
//	toolkit::CCriticalSectionHandle	_CSC;
//};

struct SPointReadWriterParam
{
	SPointReadWriterParam() :_nBeginIndex(0), _eSegment(ESegmentType::eAutoSegment)
		//, _pClusterBoxMap(nullptr)
		, _strPrjId(nullptr)
		, _pLodNode(nullptr)
		//, _pTreeClusterMap(nullptr)
		, _vegetationTypes{}
		, _nGroundType(0){}

	//osg::ref_ptr<CPointCloudpagedLod>	_pagedLod;			// PageLod节点
	pc::data::CModelNodePtr				_pLodNode;			// 点云业务节点
	IPointCloudPtr						_pPointCloud;		// 用于分类的点云数组
	int									_nBeginIndex;		// 起始索引
	std::map<int, int>					_mapConvertType;	// 类型转换
	std::map<char, bool>				_vecSegShowTypep;	// 框选点
	ESegmentType						_eSegment;			// 分类类型
	//pc::share_ptr<SClusterBoxMap>		_pClusterBoxMap;	// 单木分割生成的簇
	LPCTSTR								_strPrjId;			// 工程id
	//pc::share_ptr<STreeClusterMap>		_pTreeClusterMap;	// 用以记录树id对应的簇
	std::set<int>						_denoiseIndexSet;	// 噪点索引
	std::set<int>						_clearTypeCluster;	// 指定分类清理所属簇
	//pc::share_ptr<SClearCluster>		_pClearCluster;		// 需要删除簇对象的簇id
	std::set<unsigned>				_vegetationTypes;	// 单木分割所需的植被类型索引
	unsigned							_nGroundType;		// 单木分割所需的地面类型索引
	osg::BoundingBox					_boundBox;			// 当前划分区域的包围盒
};

class CPointReadWriter
{
	friend CPointReadWriterVisitor;
public:
	CPointReadWriter(const SPointReadWriterParam& param);
	CPointReadWriter(const SPointReadWriterParam& param,const pc::data::PointCloudBoundBox2D& boundBox);

public:
	// 线程执行
	virtual void Read();
	virtual void Write();

	/**
	* 设置多边形参数
	* @param [in] vecSelectPonits	多边形屏幕点
	* @param [in] boundingBox		范围包围盒（目前用于单木分割，通过包围盒确定范围）
	* @param [in] vpwMatrix			屏幕vpw矩阵
	* @return
	*/
	void SetPolygonParam(const std::vector<osg::Vec3d>& vecSelectPonits, const osg::BoundingBox& boundingBox, const osg::Matrix& vpwMatrix);

	/**
	* 获取点个数（有BoundBox才有效）
	* @param [in] 
	* @return
	**/
	size_t GetPointsNum(void) { return _nEndIndex - _param._nBeginIndex; }

protected:
	osg::ref_ptr<osg::Node>					_pNode;				// 点云子节点
	std::vector<osg::Vec3d>					_vecSelectPoints;	// 框选点
	osg::Matrix								_vpwMatrix;			// 框选查看方向
	const pc::data::PointCloudBoundBox2D&	_boundBox;			// 当前遍历节点包围盒大小
	bool									_bBound;			// 当前遍历节点是否存在包围盒
	int										_nEndIndex;			// pointCloud尾索引
	SPointReadWriterParam					_param;				// 遍历读写参数
	osg::BoundingBox						_boundingBox;		// 范围包围盒（目前用于单木分割，通过包围盒确定范围）
};

#endif // PAGEDLODPOINTSVISITOR_H_