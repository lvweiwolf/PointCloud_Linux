#ifndef POINT_CLOUD_PROPERTY_VISITOR_H_
#define POINT_CLOUD_PROPERTY_VISITOR_H_

#include <LasFile/PointCloudToolDefine.h>


class CPointCloudSetPropVisitor : public osg::NodeVisitor
{
public:
	CPointCloudSetPropVisitor(
		const pc::data::SVisitorInfos& visitorInfos,
		const CString& strPageNodeId,
		osg::NodeVisitor::TraversalMode traversalMode = osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
	
	virtual ~CPointCloudSetPropVisitor();
	
	void apply(osg::Geometry& geometry) override;
	
	void apply(osg::Geode& node) override;

public:
	/**
	 * 是否有效遍历
	 * @return
	 */
	bool EffectiveTraversal() { return _bEffectiveTraversal; }

protected:
	/**
	 * 初始化信息
	 * @param [out] info			遍历信息
	 * @return
	 */
	void InitInfo(pc::data::SVisitorInfo& info);

	/**
	 * 判断点是否在二维多边形内
	 * @param [in] point
	 * @param [in] polygon 多边形
	 * @return
	 */
	bool PointInPolygonXY(const osg::Vec3d& point, const std::vector<osg::Vec3d>& polygon);

	/**
	 * 获取点在拟合多边形内的索引
	 * @param [in] VertexPoint	点
	 * @param [in] matrixVPW		vpw矩阵
	 * @param [in] polygonLimit	多边形极值
	 * @param [out] nIndex		索引
	 * @return	是否在拟合多边形的包围盒内
	 */
	bool GetPointInPolygonIndex(const osg::Vec3d& VertexPoint,
								const osg::Matrix& matrixVPW,
								const pc::data::SLimitValue& polygonLimit,
								int& nIndex);

	/**
	 * 校验点以判断是否执行OperatorPoint
	 * @param [in] VertexPoint 顶点
	 * @param [in] pTexCoordArray
	 * @param [in] nIndex
	 * @return
	 */
	bool ValildData(const osg::Vec3d& VertexPoint,
					osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
					const size_t& nIndex);

	/**
	 * 校验点以判断是否执行OperatorPoint
	 * @param [in] VertexPoint	顶点
	 * @param [in] info			遍历信息
	 * @return
	 */
	bool ValildData(const osg::Vec3d& VertexPoint, pc::data::SVisitorInfo& info);

protected:
	pc::data::SVisitorInfos _visitorInfos; // 遍历信息
	bool _bExitTraversal;				   // 结束遍历
	bool _bEffectiveTraversal;			   // 有效多边形（多边形内是否存在点云）
	bool _bSliceSelect;					   // 是否是剖切视图选择
	CString _strPageNodeId;				   // pagenode id
};


#endif // POINT_CLOUD_PROPERTY_VISITOR_H_