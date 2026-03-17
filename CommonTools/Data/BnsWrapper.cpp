
#include <include/BnsWrapper.h>
#include <include/ModelNode.h>

namespace pc {
	namespace data {
		BnsWrapper::BnsWrapper(CModelNodePtr pNode) { _pNode = pNode; }

		BnsWrapper::BnsWrapper() : _pNode(nullptr) {}

		BnsWrapper::~BnsWrapper() {}

		bool BnsWrapper::IsNull() const { return nullptr == _pNode; }

		BnsWrapper::operator CModelNode*() const { return _pNode.get(); }

		BnsWrapper::operator CModelNodePtr() const { return _pNode; }

		bool BnsWrapper::HasNode(CModelNodePtr pNode) { return pNode == _pNode; }

		bool BnsWrapper::operator==(const BnsWrapper& wrapper) { return _pNode == wrapper._pNode; }

		CModelNode* BnsWrapper::operator->() { return _pNode.get(); }

		int BnsWrapper::GetTag() const { return _pNode->getNodeTypeId(); }

		void BnsWrapper::SetTag(int nTag)
		{
			if (_pNode)
				_pNode->setNodeTypeId(nTag);
		}

		const CAny& BnsWrapper::GetData(LPCWSTR lpKey, const CAny& defData)
		{
			return _pNode ? _pNode->GetData(lpKey, defData) : defData;
		}

		void BnsWrapper::SetData(LPCWSTR lpKey, const CAny& data)
		{
			if (_pNode)
				_pNode->SetData(lpKey, data);
		}

		void BnsWrapper::SetData(const CString& strTag, LPCWSTR lpKey, const CAny& data)
		{
			if (!_pNode)
				return;

			SetData(lpKey, data);
		}

		CModelNodePtr BnsWrapper::GetParentByTag(int nTag)
		{
			if (_pNode == nullptr)
				return nullptr;

			CModelNodePtr pParentNode = _pNode->GetParent();
			while (pParentNode != nullptr)
			{
				if (pParentNode->getNodeTypeId() == nTag)
					return pParentNode;

				pParentNode = pParentNode->GetParent();
			}
			return nullptr;
		}

		bool BnsWrapper::RemoveData(LPCWSTR lpKey) { return _pNode->RemoveData(lpKey); }

		bool BnsWrapper::HasData(LPCWSTR lpKey) const { return _pNode->HasData(lpKey); }
	}
}