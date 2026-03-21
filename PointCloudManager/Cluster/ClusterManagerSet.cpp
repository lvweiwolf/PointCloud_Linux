#include <include/ClusterManagerSet.h>

// CClusterManagerSet* Singleton<CClusterManagerSet>::_pInstance{ nullptr };

d3s::share_ptr<CClusterManager> CClusterManagerSet::AddClusterManager(CString strPrjId)
{
	if (CString(strPrjId).IsEmpty())
		return nullptr;

	d3s::share_ptr<CClusterManager> pClusterManager = new CClusterManager;
	_clusterManagerMap[strPrjId] = pClusterManager;
	
	return pClusterManager;
}

void CClusterManagerSet::RemoveClusterManager(CString strPrjId)
{
	_clusterManagerMap.erase(strPrjId);
}

d3s::share_ptr<CClusterManager> CClusterManagerSet::GetClusterManager(
	CString strPrjId,
	bool bNotExistCreate)
{
	auto pIter = _clusterManagerMap.find(strPrjId);
	if (_clusterManagerMap.end() == pIter)
	{
		if (bNotExistCreate)
			return AddClusterManager(strPrjId);
		else
			return nullptr;
	}
	
	return pIter->second;
}
