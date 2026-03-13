//////////////////////////////////////////////////////////////////////
// 文件名称：ModelNode.h
// 功能描述：模型树对象
// 创建标识：吴建峰 2025/01/17
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef MODELNODE_H_
#define MODELNODE_H_

#include <include/Any.h>
#include <include/share_ptr.h>
#include <include/ReferenceCountObj.h>

#include <map>
#include <vector>
#include <list>

namespace pc {
	namespace data {
		class CModelNode;
		class CModelNodeXmlHelper;
		class CModelNodeXmdxHelper;
		typedef d3s::share_ptr<CModelNode> CModelNodePtr;
		typedef std::vector<CModelNodePtr> CModelNodeVector;
		typedef std::list<CModelNodePtr> CModelNodeList;

// 节点返回错误标记
#define MODELNODE_POS_TAIL (size_t(-1)) // 队列最后面位置
#define MODELNODE_POS_HEAD (size_t(-2)) // 队列最前面位置
#define MODELNODE_ERROR (size_t(-3))	// MODELNODE操作错误
#define MODELNODE_NONE_TAG (-1)			// ModelNODE无标签

		// 节点数据MAP
		typedef std::map<CString, CAny> CModelNodeDataMap;
		typedef CModelNodeDataMap::iterator CModelNodeDataMapIter;
		typedef CModelNodeDataMap::const_iterator CModelNodeDataMapCIter;

		struct CStringHash
		{
			size_t operator()(const CString& _Keyval) const
			{
				// 获取字符串指针和长度
				const wchar_t* str = _Keyval;
				int len = _Keyval.GetLength();

				if (len == 0)
					return 0;

				// 使用djb2哈希算法（简单高效）
				size_t hash = 5381;

				for (int i = 0; i < len; ++i)
				{
					hash = ((hash << 5) + hash) + static_cast<size_t>(str[i]);
				}

				return hash;
			}
		};

		class COMMONTOOLS_EXPORT CModelNode : public d3s::ReferenceCountObj
		{
			friend CModelNodeXmlHelper;
			friend CModelNodeXmdxHelper;

		public:
			CModelNode(int nodeTypeId);

		public:
			virtual ~CModelNode();

			/**
			 * 设置节点类型
			 * @param [in] nodeTypeId
			 * @return
			 **/
			void setNodeTypeId(int nodeTypeId);
			/**
			 * 获取节点类型
			 * @param [in]
			 * @return
			 **/
			int getNodeTypeId(void);

			/**
			 *  函数介绍    	获取id
			 *
			 *  输出参数
			 *  返回值   	CString
			 */
			CString GetId();

			/**
			 *  函数介绍    	设置id
			 *
			 *  输入参数    	const CString & strId
			 *  输出参数
			 *  返回值   	void
			 */
			void SetId(const CString& strId);

			/**
			 * 设置节点名称
			 * @param [in] strName
			 * @return
			 **/
			void SetNodeName(const CString& strName);

			/**
			 * 获取节点名称
			 * @return
			 **/
			CString GetNodeName();

			/**
			 * 获取父节点
			 * @param [in]
			 * @return
			 **/
			CModelNode* GetParent(void);
			/**
			 * 设置父节点
			 * @param [in] parent
			 * @return
			 **/
			void SetParent(CModelNode* parent);

			/**
			 * 获取子节点数量
			 * @param [in]
			 * @return
			 **/
			size_t getChildCount(void);
			/**
			 * 获取子节点
			 * @param [in] index
			 * @return
			 **/
			CModelNodePtr GetChildNode(size_t index);

		public:
			/**
			 *  @brief    删除属性
			 *
			 *  @param    LPCWSTR lpKey 关键字
			 *  @return   bool 存在返回true
			 */
			virtual bool RemoveData(LPCWSTR lpKey);

			/**
			 *  @brief    删除所有属性
			 *
			 */
			virtual void RemoveAllData();

			/**
			 *  @brief    获取属性
			 *
			 *  @param    LPCWSTR lpKey 关键字
			 *  @param    const CAny& defData 没有找到的默认值
			 *								 (设置此参数后将默认添加到属性中)
			 *  @return   const CAny& 返回属性对象,如果没有则为空对象
			 */
			virtual const CAny& GetData(LPCWSTR lpKey, const CAny& defData = CAny());

			/**
			 *  @brief    设置属性
			 *
			 *  @param    LPCWSTR lpKey 关键字
			 *  @param    cosnt CAny& data 数据
			 *  @param    bool bAttach 是否把data的值直接转移(提高赋值性能)
			 *  @remark	 如果不存关键字则创建对应属性
			 */
			virtual void SetData(LPCWSTR lpKey, const CAny& data, bool bAttach = false);

			/**
			 *  @brief    设置属性
			 *
			 *  @param    LPCWSTR lpKey 关键字
			 *  @param    cosnt CAny& data 数据
			 *  @param    bool bAttach 是否把data的值直接转移(提高赋值性能)
			 *  @remark	 如果不存关键字则创建对应属性
			 */
			virtual void SetDataFromNode(CModelNode* pNode);

			/**
			 *  @brief    是否存指定属性
			 *
			 *  @param    LPCWSTR lpKey 关键字
			 *  @return   bool 存在返回true
			 */
			virtual bool HasData(LPCWSTR lpKey) const;

		public:
			/**
			 *  @函数介绍  获取
			 *
			 *  @函数参数  int nType
			 *  @返回值    CModelNode*
			 */
			virtual CModelNode* GetTypeParent(int nType);

			/**
			 *  函数介绍    	获取子节点(非递归)
			 *
			 *  输入参数    	void
			 *  输出参数
			 *  返回值   	CModelNodeVector
			 */
			virtual CModelNodeVector GetSubNodes(void);

			/*
			 *  函数介绍：   查找对应索引位置的子节点
			 *  输入参数：	nIndex		子节点索引
			 *  返回值  ：	CModelNode*	返回查找的节点，索引越界则返回null
			 */
			virtual CModelNodePtr FindSubNode(int nNodeType, bool bCreate);
			/**
			 *  @brief    删除指定的节点(删除的节点内存自行释放)
			 *
			 *  @param   CModelNode* pRemNode 删除的节点
			 *  @param    bool bDelete 是否删除节点内存
			 *  @return  size_t 返回删除的节点的索引，没找到返回CModelNode_ERROR
			 */
			virtual size_t RemoveNode(CModelNodePtr pRemNode);
			virtual CModelNodePtr RemoveNode(size_t nIndex);

			/**
			 * 移除所有子节点
			 * @return
			 */
			virtual void RemoveAllNode(bool deleteNode);

			/**
			 *  @brief    插入一个子节点(通过基于0索引位置)
			 *
			 *  @param    size_t nIndex 新插入节点所在索引,传入size_t(-1)则为插入到最后
			 *  @param    int nTag 数据类型
			 *  @return   CModelNode* 失败返回NULL
			 */
			virtual CModelNodePtr InsertNode(size_t nIndex, int nTag);

			/**
			 *  函数介绍    	插入一个子节点
			 *
			 *  输入参数    	CModelNode * pInsertNode	待插入的节点
			 *  输入参数    	size_t nIndex 新插入节点所在索引,传入size_t(-1)则为插入到最后
			 *  输出参数
			 *  返回值   	CModelNode*
			 */
			virtual CModelNodePtr InsertNode(CModelNodePtr pInsertNode, size_t nIndex);

			/**
			 *  @brief    复制节点数据以及子节点
			 *			 (如果拷贝子节点则当前节点下的子节点内存会全部释放)
			 *
			 *  @param    CModelNode* pSrcNode 原始数据的节点
			 *  @param    bool bRecursion 是否复制子节点与数据
			 */
			virtual void CopyFrom(CModelNodePtr pSrcNode, bool bRecursion = true);

			/**
			 *  函数介绍    	克隆节点
			 *
			 *  输入参数    	bool bRecursion		true为深拷贝(递归拷贝)，false为浅拷贝
			 *  输出参数
			 *  返回值   	CModelNode*
			 */
			virtual CModelNodePtr Clone(bool bRecursion);

			/**
			 * 交换节点
			 * @param [in] pNode	节点
			 * @return
			 **/
			virtual bool SwapNode(CModelNodePtr pNode);
			/**
			 * 获取节点索引
			 * @param [in] pNode	节点
			 * @return
			 **/
			virtual size_t GetNodeIndex();

		public:
			/**
			 *  @brief    获取所有属性
			 *
			 *  @return   const CModelNodeDataMap& 返回所有属性(没有属性返回NULL)
			 */
			virtual const CModelNodeDataMap& GetDataMap() const;

		protected:
			/**
			 * 实际的修改数据
			 * @param [in] lpKey
			 * @param [in] data
			 * @param [in] bAttach
			 * @return
			 **/
			void SetDataR(LPCWSTR lpKey, const pc::data::CAny& data, bool bAttach = false);
			/**
			 *  @brief    删除属性
			 *
			 *  @param    LPCWSTR lpKey 关键字
			 *  @return   bool 存在返回true
			 */
			virtual bool RemoveDataR(LPCWSTR lpKey);

			/**
			 *  @brief    删除所有属性
			 *
			 */
			virtual void RemoveAllDataR();

			/**
			 * 插入子结点
			 * @param [in] node	结点
			 * @param [in] index	索引
			 * - 如果插入第1个位置则为0，最后则为 getChildCount()
			 */
			virtual void insertChild(size_t index, CModelNodePtr node);

			/**
			 * 移除了结点
			 * @param [in] index	结点索引
			 * @return 返回要移除的结点
			 */
			virtual CModelNodePtr removeChild(size_t index);

			/**
			 * 移除所有子结点
			 * @param [in] deleteNode	是否要删除移除的子结点
			 */
			virtual void removeAllChild(bool deleteNode);
			/**
			 * 实际的交换节点
			 * @param [in] pNode
			 * @return
			 **/
			bool SwapNodeR(CModelNodePtr pNode);

		protected:
			CModelNodeDataMap* _pDataMap; ///< 属性集合
			int _nodeTypeId;
			CModelNode* _pParent;
			std::vector<CModelNodePtr> _childNodes;
			CString _strID;		  // 节点唯一id
			CString _strNodeName; // 节点名称
		};
	}
}

typedef pc::data::CModelNodePtr ModelNodePtr;


#endif // MODELNODE_H_