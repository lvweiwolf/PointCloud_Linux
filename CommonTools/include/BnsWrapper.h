//*****************************************************
//
//    @copyright      配网设计平台组
//    @version        v4.0
//    @file           BnsWrapper
//    @author         WXL
//    @data           2018/02/07 14:45
//    @brief          业务包装类
//
//*****************************************************

#ifndef BNSWRAPPER_H_
#define BNSWRAPPER_H_

#include <include/CommonToolsExport.h>
#include <include/ModelNode.h>

namespace pc {
	namespace data {
		class COMMONTOOLS_EXPORT BnsWrapper
		{
		public:
			BnsWrapper();
			BnsWrapper(CModelNodePtr pNode);
			virtual ~BnsWrapper();

			/**
			 *  @brief    判断是否包含CModelNode指针相同
			 */
			bool operator==(const BnsWrapper& wrapper);

			CModelNode* operator->();

			/**
			 *  @brief    判断是否为空
			 */
			bool IsNull() const;

			/**
			 *  @brief    转换成CModelNode*
			 */
			operator CModelNode*() const;

			/**
			 *  @brief    转换成CModelNode*
			 */
			operator CModelNodePtr() const;

			/**
			 *  @brief    判断是否含有CModelNode,如果是BnsNode的包装类,则判断内部的CModelNode
			 *
			 *  @param    CModelNode * pNode
			 *  @return   bool
			 */
			bool HasNode(CModelNodePtr pNode);

		public:
			LPCTSTR GetID() const;
			int GetTag() const;
			void SetTag(int nTag);
			const CAny& GetData(LPCTSTR lpKey, const CAny& defData = CAny());
			void SetData(LPCTSTR lpKey, const CAny& data);
			bool RemoveData(LPCTSTR lpKey);
			bool HasData(LPCTSTR lpKey) const;
			void SetData(const CString& strTag, LPCTSTR lpKey, const CAny& data);
			CModelNodePtr GetParentByTag(int nTag);

		public:
			/**
			 *  @brief    转换成具体包装类
			 *
			 */
			template <class T>
			T Cast() const
			{
				T entity;
				entity._pNode = _pNode;
				return entity;
			}

		protected:
			CModelNodePtr _pNode; // 内部节点
		};

		using BnsWrapperList = std::vector<BnsWrapper>;


#define DefineProperty(proName, valType)                                                           \
	virtual void Set##proName(const valType& valType##inst)                                        \
	{                                                                                              \
		_pNode->SetData(L#proName, valType##inst);                                                 \
	}                                                                                              \
	virtual valType Get##proName() { return _pNode->GetData(L#proName); }

#define DefinePropertyD(proName, valType, defalutValue)                                            \
	virtual void Set##proName(const valType& valType##inst)                                        \
	{                                                                                              \
		_pNode->SetData(L#proName, valType##inst);                                                 \
	}                                                                                              \
	virtual valType Get##proName() { return _pNode->GetData(L#proName, #defalutValue); }

#define DefinePropertyEx(funName, keyName, valType)                                                \
	virtual void Set##funName(const valType& valType##inst)                                        \
	{                                                                                              \
		_pNode->SetData(L#keyName, valType##inst);                                                 \
	}                                                                                              \
	virtual valType Get##funName() { return _pNode->GetData(L#keyName); }

#define DefinePropertyT(funName, keyName, valType)                                                 \
	virtual void Set##funName(const valType& valType##inst)                                        \
	{                                                                                              \
		_pNode->SetData(keyName, valType##inst);                                                   \
	}                                                                                              \
	virtual valType Get##funName() const { return _pNode->GetData(keyName); }

#define DefinePropertyTD(funName, keyName, valType, defalutValue)                                  \
	virtual void Set##funName(const valType& valType##inst)                                        \
	{                                                                                              \
		_pNode->SetData(keyName, valType##inst);                                                   \
	}                                                                                              \
	virtual valType Get##funName() const { return _pNode->GetData(keyName, defalutValue); }
	}
}

#endif // BNSWRAPPER_H_