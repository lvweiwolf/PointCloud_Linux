#include <include/ClusterManager.h>


CClusterManager::CClusterManager() { _nMaxClusterID = INVALID_CLUSTER_ID; }

d3s::share_ptr<CClusterItem> CClusterManager::CreateCluster(int nType, bool bAdd)
{
	d3s::share_ptr<CClusterItem> pClusterItem = new CClusterItem(nType);
	InitClusterItemID(pClusterItem);

	if (bAdd)
		AddClusterItem(pClusterItem);

	return pClusterItem;
}

void CClusterManager::AddClusterItem(d3s::share_ptr<CClusterItem> pClusterItem)
{
	if (nullptr == pClusterItem)
		return;
	
	InitClusterItemID(pClusterItem);
	
	// 联动更新最大簇ID
	if (_nMaxClusterID < pClusterItem->GetId())
		_nMaxClusterID = pClusterItem->GetId();

	_clusterItemMap[pClusterItem->GetId()] = pClusterItem;
	_clusterClassifyMap[pClusterItem->GetClassifyType()] = pClusterItem;
}

void CClusterManager::RemoveClusterItem(d3s::share_ptr<CClusterItem> pClusterItem)
{
	if (nullptr == pClusterItem)
		return;

	_clusterItemMap.erase(pClusterItem->GetId());
	_clusterClassifyMap.erase(pClusterItem->GetClassifyType());
}

void CClusterManager::RemoveClusterItem(int nClusterID)
{
	RemoveClusterItem(FindClusterByID(nClusterID));
}

d3s::share_ptr<CClusterItem> CClusterManager::FindClusterByID(int nClusterID)
{
	auto pPairIter = _clusterItemMap.find(nClusterID);
	if (_clusterItemMap.end() == pPairIter)
		return nullptr;

	return pPairIter->second;
}

d3s::share_ptr<CClusterItem> CClusterManager::FindClusterByType(int nClassifyType)
{
	auto pPairIter = _clusterClassifyMap.find(nClassifyType);
	if (_clusterClassifyMap.end() == pPairIter)
		return nullptr;

	return pPairIter->second;
}

std::map<int, d3s::share_ptr<CClusterItem>> CClusterManager::GetClusterMap()
{
	return _clusterItemMap;
}

void CClusterManager::FormatData()
{
	_clusterClassifyMap.clear();
	_clusterItemMap.clear();
	_nMaxClusterID = INVALID_CLUSTER_ID;
}

void CClusterManager::InitClusterItemID(d3s::share_ptr<CClusterItem> pClusterItem)
{
	if (nullptr == pClusterItem || INVALID_CLUSTER_ID != pClusterItem->GetId())
		return;

	_nMaxClusterID += 1;
	pClusterItem->SetId(_nMaxClusterID);
}
