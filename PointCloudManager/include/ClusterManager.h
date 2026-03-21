#ifndef CLUSTER_MANAGER_H_
#define CLUSTER_MANAGER_H_

#include <include/ClusterItem.h>

#define INVALID_CLUSTER_ID 0

class CClusterManagerSet;

class POINTCLOUDMANAGER_EXPORT CClusterManager : public d3s::ReferenceCountObj
{
	friend CClusterManagerSet;

public:
	/**
	 * 创建簇
	 * @param [in] bAdd	是否添加
	 * @return
	 */
	d3s::share_ptr<CClusterItem> CreateCluster(int nType, bool bAdd);

	/**
	 * 添加簇
	 * @param [in] pClusterItem
	 * @return
	 */
	void AddClusterItem(d3s::share_ptr<CClusterItem> pClusterItem);

	/**
	 * 移除簇
	 * @param [in] pClusterItem
	 * @param [in] nClusterID
	 * @return
	 */
	void RemoveClusterItem(d3s::share_ptr<CClusterItem> pClusterItem);
	void RemoveClusterItem(int nClusterID);

	/**
	 * 查找簇
	 * @param [in] nClusterID 簇ID
	 * @return
	 */
	d3s::share_ptr<CClusterItem> FindClusterByID(int nClusterID);

	/**
	 * 查找簇
	 * @param [in] nClassifyType 分类类型
	 * @return
	 */
	d3s::share_ptr<CClusterItem> FindClusterByType(int nClassifyType);

	/**
	 * 获取簇对象集合
	 * @return
	 */
	std::map<int, d3s::share_ptr<CClusterItem>> GetClusterMap();

	/**
	 * 格式化数据（清理所有簇，重置最大簇id）
	 * @return
	 */
	void FormatData();

protected:
	/**
	 * 初始化簇对象ID
	 * @param [in] pClusterItem
	 * @return
	 */
	void InitClusterItemID(d3s::share_ptr<CClusterItem> pClusterItem);

protected:
	CClusterManager();

private:
	int _nMaxClusterID;												 // 最大簇ID
	std::map<int, d3s::share_ptr<CClusterItem>> _clusterItemMap;	 // 簇集合 <簇ID,簇对象>
	std::map<int, d3s::share_ptr<CClusterItem>> _clusterClassifyMap; // 簇集合 <类型,簇对象>
};


#endif // CLUSTER_MANAGER_H_