#ifndef CLUSTER_ITEM_H_
#define CLUSTER_ITEM_H_

#include <include/cstring.h>
#include <include/share_ptr.h>
#include <include/ReferenceCountObj.h>
#include <include/PointCloudManagerExport.h>

#include <Tool/XmlDocument.h>

#include <osg/Vec3d>
#include <osg/BoundingBox>

#include <map>
#include <set>

class CClusterManager;

class POINTCLOUDMANAGER_EXPORT CClusterItem : public d3s::ReferenceCountObj
{
	friend class CClusterManager;

public:
	typedef std::vector<osg::Vec3d> Polygon;
	typedef std::vector<Polygon> PolygonList;

	CClusterItem(const CClusterItem& rhs);

	/**
	 * 加载保存
	 * @return
	 */
	static void Save(d3s::share_ptr<CClusterItem> pCluster, toolkit::CXmlElement* pClusterEle);
	static d3s::share_ptr<CClusterItem> Load(toolkit::CXmlElement* pClusterEle);

public:
	/**
	 * 获取簇ID
	 * @return
	 */
	int GetId();

	/**
	 * 设置/获取分类类型
	 * @return
	 */
	int GetClassifyType();
	void SetClassifyType(int nClassifyType);

	/**
	 * 设置获取属性
	 * @return
	 */
	CString GetProp(const CString& strKey);
	std::map<CString, CString>& GetProp();
	void SetProp(const CString& strKey, const CString& strValue);

	/**
	 * 添加关联pageLod唯一标识ID
	 * @return
	 */
	void AddPagedLodID(const CString& strPageLodId);
	void AddPagedLodID(const std::vector<CString>& pageLodIdList);

	/**
	 * 删除关联pageLod唯一标识ID
	 * @param [in] strPageLodId
	 * @return
	 */
	void RemovePagedLodID(const CString& strPageLodId);

	/**
	 * 获取关联pageLod唯一标识ID
	 * @return
	 */
	const std::set<CString>& GetPagedLodID();

	/**
	 * 获取包围盒
	 * @return
	 */
	osg::BoundingBox GetBoundingBox();
	void SetBoundingBox(const osg::BoundingBox& boundingBox);

	/**
	 *  @brief    获得/设置 多边形边界
	 *
	 *  @return   PolygonList
	 */
	PolygonList GetPolygons();
	void SetPolygons(const PolygonList& polygons);

protected:
	CClusterItem(int nClassifyType,
				 std::map<CString, CString> propMap = std::map<CString, CString>{});

	/**
	 * 设置获取簇ID
	 * @return
	 */
	void SetId(int nId);

private:
	std::map<CString, CString> _propMap; // 属性
	std::set<CString> _pageLodIdList;	 // 关联的pageLOD唯一标识ID
};


#endif // CLUSTER_ITEM_H_