#ifndef CLUSTER_MANAGER_SET_H_
#define CLUSTER_MANAGER_SET_H_

#include <include/Singleton.h>
#include <include/ClusterManager.h>

class POINTCLOUDMANAGER_EXPORT CClusterManagerSet : public Singleton<CClusterManagerSet>
{
	friend Singleton<CClusterManagerSet>;

public:
	/**
	 * 获取簇管理器
	 * @param [in] strPrjId			工程ID
	 * @param [in] bNotExistCreate	不存则创建添加
	 * @return
	 */
	d3s::share_ptr<CClusterManager> GetClusterManager(CString strPrjId,
													  bool bNotExistCreate = true);

	/**
	 * 删除簇管理器
	 * @param [in] strPrjId			工程ID
	 * @return
	 */
	void RemoveClusterManager(CString strPrjId);

protected:
	/**
	 * 添加簇管理器
	 * @param [in] strPrjId			工程ID
	 * @param [in] pClusterManager	簇管理器
	 * @return
	 */
	d3s::share_ptr<CClusterManager> AddClusterManager(CString strPrjId);

private:
	CClusterManagerSet() {}

private:
	std::map<CString, d3s::share_ptr<CClusterManager>> _clusterManagerMap; // 簇管理器集合
};





#endif // CLUSTER_MANAGER_SET_H_