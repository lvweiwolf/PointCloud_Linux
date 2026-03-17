#include <include/ModelNode.h>
#include <Tool/LibToolkit.h>

namespace pc {
	namespace data {
		CModelNode::CModelNode(int nodeTypeId)
			: _nodeTypeId(nodeTypeId), _pParent(nullptr), _strID(CLibToolkit::CreateGuid())
		{
			_pDataMap = nullptr;
		}

		CModelNode::~CModelNode()
		{
			// 释放数据列表
			if (nullptr != _pDataMap)
			{
				_pDataMap->clear();
				SAFE_DELETE(_pDataMap);
			}
		}

		void CModelNode::setNodeTypeId(int nodeTypeId) { _nodeTypeId = nodeTypeId; }

		int CModelNode::getNodeTypeId(void) { return _nodeTypeId; }

		CString CModelNode::GetId() { return _strID; }

		void CModelNode::SetId(const CString& strId) { _strID = strId; }

		void CModelNode::SetNodeName(const CString& strName) { _strNodeName = strName; }

		CString CModelNode::GetNodeName() { return _strNodeName; }

		pc::data::CModelNode* CModelNode::GetParent(void) { return _pParent; }

		void CModelNode::SetParent(CModelNode* parent) { _pParent = parent; }

		size_t CModelNode::getChildCount(void) { return _childNodes.size(); }

		pc::data::CModelNodePtr CModelNode::GetChildNode(size_t index)
		{
			if (index >= _childNodes.size())
			{
				return NULL;
			}
			return _childNodes[index];
		}


		bool CModelNode::RemoveData(LPCWSTR lpKey) { return RemoveDataR(lpKey); }

		void CModelNode::RemoveAllData() { RemoveAllDataR(); }

		const pc::data::CAny& CModelNode::GetData(LPCWSTR lpKey, const pc::data::CAny& defData)
		{
			static pc::data::CAny stAny; // 用于返回临时引用
			if (nullptr == lpKey && defData.IsEmpty())
				return stAny;

			if (nullptr == _pDataMap)
				_pDataMap = new CModelNodeDataMap();

			CModelNodeDataMapCIter findIter = _pDataMap->find(lpKey);
			if (findIter != _pDataMap->end())
			{
				return findIter->second;
			}

			// 没有找到并且是默认值为空返回
			if (defData.IsEmpty())
			{
				return stAny;
			}

			// 添加默认值
			std::pair<CModelNodeDataMapIter, bool> ret =
				_pDataMap->insert(CModelNodeDataMap::value_type(lpKey, defData));

			// 返回新添加的默认值
			return ret.first->second;
		}

		void CModelNode::SetData(LPCWSTR lpKey, const pc::data::CAny& data, bool bAttach)
		{
			SetDataR(lpKey, data, bAttach);
		}

		void CModelNode::SetDataFromNode(CModelNode* pNode)
		{
			auto newNodeData = pNode->GetDataMap();
			for (auto var : newNodeData)
			{
				SetData(var.first, var.second);
			}
		}

		bool CModelNode::HasData(LPCWSTR lpKey) const
		{
			return (nullptr != lpKey) && (nullptr != _pDataMap) &&
				   (_pDataMap->find(lpKey) != _pDataMap->end());
		}

		CModelNode* CModelNode::GetTypeParent(int nType)
		{
			CModelNode* pParent = GetParent();
			while (pParent != nullptr && pParent->getNodeTypeId() != nType)
			{
				pParent = pParent->GetParent();
			}

			return pParent;
		}

		CModelNodeVector CModelNode::GetSubNodes(void)
		{
			CModelNodeVector subNodeVec;
			size_t nCnt = getChildCount();
			for (size_t i = 0; i < nCnt; ++i)
			{
				CModelNodePtr pSubNode = GetChildNode(i);
				if (NULL == pSubNode)
					continue;

				subNodeVec.emplace_back(pSubNode);
			}
			return subNodeVec;
		}

		CModelNodePtr CModelNode::FindSubNode(int nNodeType, bool bCreate)
		{
			int nCount = getChildCount();
			for (int i = 0; i < nCount; i++)
			{
				CModelNodePtr pNode = GetChildNode(i);
				if (pNode->getNodeTypeId() == nNodeType)
					return pNode;
			}

			// 不存在则创建
			if (bCreate)
			{
				return InsertNode(getChildCount(), nNodeType);
			}

			return nullptr;
		}

		CModelNodePtr CModelNode::InsertNode(size_t nIndex, int nTag)
		{
			// 检查输入合法性
			size_t nSubCount = getChildCount();
			if (nIndex == -1)
			{
				nIndex = nSubCount;
			}

			if ((size_t(-1) != nIndex) && (nIndex > nSubCount))
				return nullptr;

			CModelNodePtr pNode = new CModelNode(nTag);

			// 插入子节点
			insertChild(nIndex, pNode);

			return pNode;
		}

		CModelNodePtr CModelNode::InsertNode(CModelNodePtr pInsertNode, size_t nIndex)
		{
			// 检查输入合法性
			if (NULL == pInsertNode)
				return NULL;

			// 检查输入合法性
			size_t nSubCount = getChildCount();
			if (nIndex == -1)
			{
				nIndex = nSubCount;
			}

			if ((size_t(-1) != nIndex) && (nIndex > nSubCount))
				return nullptr;

			// 插入子节点
			insertChild(nIndex, pInsertNode);
			return pInsertNode;
		}

		size_t CModelNode::RemoveNode(CModelNodePtr pRemNode)
		{
			size_t nIndex = MODELNODE_ERROR;
			size_t nCount = getChildCount();
			for (size_t i = 0; i < nCount; i++)
			{
				if (GetChildNode(i) == pRemNode)
				{
					nIndex = i;
					break;
				}
			}
			if (nIndex != MODELNODE_ERROR)
			{
				removeChild(nIndex);
			}

			return nIndex;
		}

		CModelNodePtr CModelNode::RemoveNode(size_t nIndex)
		{
			size_t nCount = getChildCount();
			if (nIndex > nCount - 1)
				return NULL;
			CModelNodePtr pRetNode = GetChildNode(nIndex);

			auto pRemoveNode = removeChild(nIndex);
			return pRetNode;
		}

		void CModelNode::RemoveAllNode(bool deleteNode) { removeAllChild(deleteNode); }

		void CModelNode::CopyFrom(CModelNodePtr pSrcNode, bool bRecursion)
		{
			if (nullptr == pSrcNode || pSrcNode == this)
				return;

			// 清除源所有属性
			RemoveAllData();
			setNodeTypeId(pSrcNode->getNodeTypeId());

			// 拷贝属性,触发撤销记录
			const CModelNodeDataMap& dataMap = pSrcNode->GetDataMap();
			for (auto it : dataMap)
			{
				SetData(it.first, it.second);
			}

			if (!bRecursion)
				return;

			// 删除当前所有子节点
			removeAllChild(true);

			// 开始拷贝子节点
			int nCount = pSrcNode->getChildCount();
			for (int i = 0; i < nCount; i++)
			{
				CModelNodePtr pSrcChildNode = pSrcNode->GetChildNode(i);
				CModelNodePtr pChildNode = new CModelNode(0);
				insertChild(i, pChildNode);
				pChildNode->CopyFrom(pSrcChildNode, bRecursion);
			}
		}

		CModelNodePtr CModelNode::Clone(bool bRecursion)
		{
			CModelNodePtr pCloneNode = new CModelNode(0);
			pCloneNode->CopyFrom(this, bRecursion);
			return pCloneNode;
		}

		bool CModelNode::SwapNode(CModelNodePtr pNode)
		{
			if (pNode == nullptr || pNode == this)
				return false;

			SwapNodeR(pNode);

			return true;
		}

		size_t CModelNode::GetNodeIndex()
		{
			CModelNodePtr pParent = GetParent();
			if (pParent == nullptr)
				return MODELNODE_ERROR;

			int nCount = pParent->getChildCount();
			for (int i = 0; i < nCount; i++)
			{
				if (pParent->GetChildNode(i) == this)
				{
					return i;
				}
			}
			return MODELNODE_ERROR;
		}

		const CModelNodeDataMap& CModelNode::GetDataMap() const
		{
			static CModelNodeDataMap tmpDataMap;
			if (nullptr == _pDataMap)
				return tmpDataMap;

			return (*_pDataMap);
		}

		void CModelNode::SetDataR(LPCWSTR lpKey,
								  const pc::data::CAny& data,
								  bool bAttach /*= false*/)
		{
			if (nullptr == lpKey)
				return;

			if (nullptr == _pDataMap)
				_pDataMap = new CModelNodeDataMap();

			pc::data::CAny& oldData = (*_pDataMap)[lpKey];
			if (oldData == data)
				return;

			// 修改
			if (bAttach)
				oldData.Attach(data);
			else
				oldData = data;
		}

		bool CModelNode::RemoveDataR(LPCWSTR lpKey)
		{
			if (nullptr == lpKey || nullptr == _pDataMap)
				return false;

			CModelNodeDataMapIter findIter = _pDataMap->find(lpKey);
			if (findIter != _pDataMap->end())
			{
				_pDataMap->erase(findIter);

				return true;
			}

			return false;
		}

		void CModelNode::RemoveAllDataR()
		{
			if (nullptr != _pDataMap)
			{
				_pDataMap->clear();
				delete _pDataMap;
				_pDataMap = NULL;
			}
		}

		void CModelNode::insertChild(size_t index, CModelNodePtr node)
		{
			_childNodes.insert(_childNodes.begin() + index, node);
			node->SetParent(this);
		}

		CModelNodePtr CModelNode::removeChild(size_t index)
		{
			CModelNodePtr node = _childNodes[index];
			for (auto it = _childNodes.begin(); it != _childNodes.end();)
			{
				if (*it == node)
				{
					it = _childNodes.erase(it); // 注意：erase返回指向下一个元素的迭代器
				}
				else
				{
					++it; // 如果不需要删除，则移动到下一个元素
				}
			}
		}

		void CModelNode::removeAllChild(bool deleteNode) { _childNodes.clear(); }
		bool CModelNode::SwapNodeR(CModelNodePtr pNode)
		{
			if (pNode == nullptr || pNode == this)
				return false;
			CModelNode* pSrcParent = GetParent();
			CModelNode* pDstParent = pNode->GetParent();

			size_t nDstIndex = pNode->GetNodeIndex();
			size_t nSrcIndex = GetNodeIndex();
			if (pSrcParent != NULL)
			{
				pSrcParent->removeChild(nSrcIndex);
				pSrcParent->insertChild(nSrcIndex, pNode);
			}
			pNode->SetParent(pSrcParent);
			if (pDstParent != NULL)
			{
				pDstParent->removeChild(nDstIndex);
				pDstParent->insertChild(nDstIndex, this);
			}
			SetParent(pDstParent);
			return true;
		}
	}
}
