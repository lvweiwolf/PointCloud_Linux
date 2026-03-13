//////////////////////////////////////////////////////////////////////
// 文件名称：XmlDocument.h
// 文件用途：ThXmlDoc,对XML文档的操作
// 创建时间：2008.3.17, 星期一
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////

#ifndef XMLDOCUMENT_H_
#define XMLDOCUMENT_H_

#include <include/CommonToolsExport.h>
#include <include/CommonToolsDef.h>
#include <include/StringToolkit.h>

#include <cstddef>
#include <cassert>

#include <vector>


namespace toolkit {

	/** @addtogroup Toolkit
	 * @{
	 */

	// XML树节点类型
	enum xml_node_type
	{
		node_null,		  // 空 (null) 节点句柄
		node_document,	  // XML文档树的根节点
		node_element,	  // 元素节点, 如 '<node/>'
		node_cdata,		  // Character data, i.e. '<![CDATA[text]]>'
		node_comment,	  // XML注释, 如 '<!-- text -->'
		node_pi,		  // Processing instruction, i.e. '<?name?>'
		node_declaration, // 文档声明, 如 '<?xml version="1.0"?>'
		node_doctype	  // Document type declaration, i.e. '<!DOCTYPE doc>'
	};


	class CXmlAttribute;
	class CXmlElement;
	class CXmlDocument;

	//! 结点/属性容器
	template <class T, class CONTENT>
	class COMMONTOOLS_EXPORT CXmlContainer
	{
	public:
		virtual ~CXmlContainer(void);

		/**
		 * 结点/属性个数
		 * @return 返回结点/属性的个数
		 */
		size_t GetCount(void);

		/**
		 * 获取结点/属性对象
		 * @param [in] index	结点/属性的索引
		 * @return 返回结点/属性对象
		 */
		CONTENT* GetAt(size_t index);

		/**
		 * 增加结点/属性
		 * @return 返回增加后的类对象
		 */
		CONTENT* Add(void);

		/**
		 * 移除结点/属性
		 * @param [in] index	结点/属性的索引
		 */
		void RemoveAt(size_t index);

		/**
		 * 移除所有结点/属性
		 */
		void RemoveAll(void);

		/**
		 * 清空重置结点/属性容器
		 */
		void reset(void);
		CONTENT* InsertAt(size_t index);

	protected:
		/**
		 * 插入结点/属性
		 * @param [in] index		插入的位置
		 * @param [in] content	结点/属性对象
		 * @return 结点/属性对象
		 */
		CONTENT* InsertAt(size_t index, CONTENT* content);

	protected:
		/** 结点/属性容器对象 */
		std::vector<CONTENT*> _array;
		CXmlElement* _parent;
	};

	template <class T, class CONTENT>
	CXmlContainer<T, CONTENT>::~CXmlContainer(void)
	{
		T* pT = static_cast<T*>(this);
		pT->RemoveAll();
	}
	template <class T, class CONTENT>
	size_t CXmlContainer<T, CONTENT>::GetCount(void)
	{
		return _array.size();
	}
	template <class T, class CONTENT>
	CONTENT* CXmlContainer<T, CONTENT>::GetAt(size_t index)
	{
		return _array.at(index);
	}
	template <class T, class CONTENT>
	CONTENT* CXmlContainer<T, CONTENT>::Add(void)
	{
		T* pT = static_cast<T*>(this);
		CONTENT* content = pT->InsertAt(_array.size());
		return content;
	}
	template <class T, class CONTENT>
	void CXmlContainer<T, CONTENT>::RemoveAt(size_t index)
	{
		assert(false);
	}
	template <class T, class CONTENT>
	void CXmlContainer<T, CONTENT>::RemoveAll(void)
	{
		T* pT = static_cast<T*>(this);
		for (int i = (int)_array.size() - 1; i >= 0; i--)
			pT->RemoveAt(i);
	}
	template <class T, class CONTENT>
	void CXmlContainer<T, CONTENT>::reset(void)
	{
		RemoveAll();
	}
	template <class T, class CONTENT>
	CONTENT* CXmlContainer<T, CONTENT>::InsertAt(size_t index)
	{
		assert(false);
		return NULL;
	}
	template <class T, class CONTENT>
	CONTENT* CXmlContainer<T, CONTENT>::InsertAt(size_t index, CONTENT* content)
	{
		_array.insert(_array.begin() + index, content);
		content->_parent = _parent;
		return content;
	}

	//! CXmlElements
	class COMMONTOOLS_EXPORT CXmlElements : public CXmlContainer<CXmlElements, CXmlElement>
	{
		friend class CXmlElement;

	public:
		/**
		 * 插入结点
		 * @param [in] index		插入的位置
		 * @return 结点对象
		 */
		CXmlElement* InsertAt(size_t index);

		/**
		 * 移除结点
		 * @param [in] index		结点索引
		 */
		void RemoveAt(size_t index);
	};

	//! CXmlAttributes
	class COMMONTOOLS_EXPORT CXmlAttributes : public CXmlContainer<CXmlAttributes, CXmlAttribute>
	{
		friend class CXmlElement;

	public:
		/**
		 * 插入属性
		 * @param [in] index		插入的位置
		 * @return 属性对象
		 */
		CXmlAttribute* InsertAt(size_t index);

		/**
		 * 移除属性
		 * @param [in] index		属性索引
		 */
		void RemoveAt(size_t index);
	};

	//! 结点属性
	class COMMONTOOLS_EXPORT CXmlAttribute
	{
		friend class CXmlContainer<CXmlAttributes, CXmlAttribute>;

	public:
		CXmlAttribute(void);
		/**
		 * 设置属性值
		 * @param [in] value	int类型属性值
		 */
		void SetAttrValue(int value);

		/**
		 * 设置属性值
		 * @param [in] value	long类型属性值
		 */
		void SetAttrValue(long value);

		/**
		 * 设置属性值
		 * @param [in] value	double类型属性值
		 */
		void SetAttrValue(double value);

		/**
		 * 设置属性值
		 * @param [in] value	LPCWSTR类型属性值
		 */
		void SetAttrValue(LPCWSTR value);

		/**
		 * 设置属性名称
		 * @param [in] name	属性名称
		 */
		void SetAttrName(LPCWSTR name);

		/**
		 * 获取属性值
		 * @return 返回int类型属性值
		 */
		int GetIntValue(void);

		/**
		 * 获取属性值
		 * @return 返回long类型属性值
		 */
		long GetLngValue(void);

		/**
		 * 获取属性值
		 * @return 返回double类型属性值
		 */
		double GetDblValue(void);

		/**
		 * 获取属性值
		 * @return 返回LPCWSTR类型属性值
		 */
		LPCWSTR GetStrValue(void);

		/**
		 * 获取属性值
		 * @return 返回LPCWSTR类型名称
		 */
		LPCWSTR GetAttrName(void);

		/**
		 * 重置属性信息
		 */
		void reset(void);

	protected:
		/** 属性名称 */
		LPWSTR _name;
		/** 属性值 */
		LPWSTR _value;

		CXmlElement* _parent;
	};

	//! CXmlElement
	class COMMONTOOLS_EXPORT CXmlElement
	{
		friend class CXmlDocument;
		friend class CXmlContainer<CXmlElements, CXmlElement>;
		friend class CXmlElements;
		friend struct xml_parser;

	public:
		CXmlElement(void);
		virtual ~CXmlElement(void);

		/**
		 * 取得子结点容器数量
		 * @return	返回子结点容器数量
		 */
		int GetChildElementCount(void);

		/**
		 * 取得子结点容器
		 * @param [in] NotExistCreate	不存在的结点是否创建 （默认为true）
		 * @return	返回子结点容器
		 */
		CXmlElements* GetChildElements(bool NotExistCreate = true);

		/**
		 * 取得子结点
		 * @param [in] name				结点名称
		 * @param [in] NotExistCreate	不存在的结点是否创建（默认为true）
		 * @return	返回子结点
		 */
		CXmlElement* GetChildElementAt(LPCWSTR name, bool NotExistCreate = true);

		/**
		 * 取得属性容器数量
		 * @return	返回属性容器数量
		 */
		int GetAttributeCount(void);

		/**
		 * 取得属性容器
		 * @param [in] NotExistCreate	不存在的属性是否创建（默认为true）
		 * @return	返回属性容器
		 */
		CXmlAttributes* GetAttributes(bool NotExistCreate = true);

		/**
		 * 取得属性
		 * @param [in] name				属性名称
		 * @param [in] NotExistCreate	不存在的属性是否创建（默认为true）
		 * @return	返回属性
		 */
		CXmlAttribute* GetAttributeAt(LPCWSTR name, bool NotExistCreate = true);

		/**
		 * 取得属性值
		 * @return	返回属性值
		 */
		LPCWSTR GetElementText(void);

		/**
		 * 设置属性值
		 * @param [in] text				属性值
		 */
		void SetElementText(LPCWSTR text);

		/**
		 * 取得属性名称
		 * @return	返回属性名称
		 */
		LPCWSTR GetElementName(void);

		/**
		 * 设置属性名称
		 * @param [in] name				属性名称
		 */
		void SetElementName(LPCWSTR name);

		/**
		 * 设置属性
		 * @param [in] AttrName			属性名称
		 * @param [in] AttrValue			属性值
		 */
		void SetAttrValue(LPCWSTR AttrName, LPCWSTR AttrValue);

		/**
		 * 添加属性
		 * @param [in] AttrName			属性名称
		 * @param [in] AttrValue			属性默认值
		 */
		void AddAttrValue(LPCWSTR AttrName, LPCWSTR AttrValue);

		/**
		 * 读取属性
		 * @param [in] AttrName			属性名称
		 * @param [in] DefaultValue		属性默认值（默认为空）
		 */
		LPCWSTR GetAttrValue(LPCWSTR AttrName, LPCWSTR DefaultValue = L"");

		/**
		 * 查找属性
		 * @param [in] AttrName			属性名称
		 * @param [out] index			属性索引
		 * @return	返回是否找到属性
		 */
		bool FindAttribute(LPCWSTR AttrName, int& index);

		/**
		 * 复制结点
		 * @param [in] root			复制的结点
		 */
		void CopyFrom(CXmlElement* root);

		/**
		 * 重置所有数据
		 */
		void reset(void);


		/**
		 * 获取当前节点是否为声明结点
		 * @return	返回当前节点是否为声明结点
		 * - true 是， false 否
		 */
		bool GetIsDeclaration() { return _Type == node_declaration; };

		/**
		 * 设置当前节点是否为声明结点
		 * @param [in] bDeclaration		是否为声明结点（默认为false）
		 */
		void SetIsDeclaration(bool bDeclaration = false);



		/**
		 * 获取文本两端是否增加双引号
		 * @return	返回文本两端是否增加双引号
		 * - true 是， false 否
		 */
		bool GetIsAddDoubleQuot() { return _IsAddDoubleQuot; }

		/**
		 * 设置文本两端是否增加双引号
		 * @param [in] bAdd		是否增加双引号（默认为true）
		 */
		void SetIsAddDoubleQuot(bool bAdd = true) { _IsAddDoubleQuot = bAdd; }


		/**
		 * 取得 XML 字符串
		 * @param [out] xml		XML字符串
		 * @param [in] error
		 * @return	返回取得 XML 字符串是否成功
		 */
		bool GetXmlString(CString& xml, CString* error = NULL);

		xml_node_type GetType() { return _Type; }
		void SetType(xml_node_type type) { _Type = type; }

		CXmlElement* GetParent() { return _parent; }

		void* GetAllocator() { return _alloc; }

	protected:
		/**
		 * 为字符串增加双引号
		 * @param [in] str	字符串
		 * @return	返回增加双引号后的字符串
		 */
		CString AddDoubleQuot(LPCWSTR str);

		/**
		 * 删除字符串双引号
		 * @param [in] str	字符串
		 * @return	返回删除双引号后的字符串
		 */
		CString DelDoubleQuot(LPCWSTR str);

	protected:
		/** 结点操作对象 */
		CXmlElements* _ElementContainer;
		/** 属性操作对象 */
		CXmlAttributes* _AttributeContainer;

		/** 结点内容 */
		LPWSTR _ElementText;
		/** 结点名称 */
		LPWSTR _ElementName;

		/** _ElementText保存时是否自动添加双引号 */
		bool _IsAddDoubleQuot;

		CXmlElement* _parent;

		xml_node_type _Type;

		void* _alloc;
	};

	//--------------------------------------------------------
	// CXmlDocument
	//--------------------------------------------------------

	/** 定义文档格式枚举类型　*/
	enum DocFormatEnum
	{
		fmtXML,		/**< XML 文本格式 */
		fmtXMD,		/**< XML 二进制格式 */
		fmtXMLUTF8, /**< UTF-8类型的文本格式(目前只支持读取UTF-8类型，不支持保存成此格式 */
		fmtXMLANSI, /**< 将XML文件保存成ANSI格式*/
		fmtXMDANSI, /**< XML 二进制格式(字符用ANSI格式) */
	};

	// 保存成UNICODE格式（<XMD 1.0>默认格式）
#define XMD_FLAGS_SAVEFORMAT_UNICODE 0x00
	// 保存成ANSI格式
#define XMD_FLAGS_SAVEFORMAT_ANSI 0x01

	//! 文档处理
	class COMMONTOOLS_EXPORT CXmlDocument
	{
		friend struct xml_parser;

	private:
		CXmlDocument(CXmlDocument&);

	public:
		CXmlDocument(void);
		~CXmlDocument(void);

		/**
		 * 加载 XML 文件
		 * @param [in] XmlFile			XML文件
		 * @param [in] format			XML格式枚举
		 * @return	返回加载 XML 文件是否成功
		 * - true 加载成功， false 加载失败
		 */
		bool LoadFile(LPCWSTR XmlFile, DocFormatEnum format);

		/**
		 * 加载 XML 文件
		 * @param [in] FileHandle		XML文件句柄
		 * @return	返回加载 XML 文件是否成功
		 * - true 加载成功， false 加载失败
		 */
		bool LoadXmlFile(std::istream& fin);


		/**
		 * 加载 UTF-8码文件
		 * @param [in] FileHandle		UTF-8码文件句柄
		 * @return	返回加载 XML 文件是否成功
		 * - true 加载成功， false 加载失败
		 */
		bool LoadXmlUtf8File(std::istream& fin);

		/**
		 * 保存 XML 文件
		 * @param [in] XmlFile			XML文件
		 * @param [in] format			XML格式枚举
		 * @return	返回保存 XML 文件是否成功
		 * - true 保存成功， false 保存失败
		 */
		bool SaveFile(LPCWSTR XmlFile, DocFormatEnum format);

		/**
		 * 保存 XML 文件
		 * @param [in] FileHandle		XML文件句柄
		 * @return	返回保存 XML 文件是否成功
		 * - true 保存成功， false 保存失败
		 */
		bool SaveXmlFile(std::ostream& fout);

		/**
		 * 保存 XML ANSI码文件
		 * @param [in] FileHandle		XML文件句柄
		 * @return	返回保存 XML 文件是否成功
		 */
		bool SaveXmlANSIFile(std::ostream& fout);

		/**
		 * 保存 XML UTF-8码文件
		 * @param [in] FileHandle		XML文件句柄
		 * @return	返回保存 XML 文件是否成功
		 * - true 保存成功， false 保存失败
		 */
		bool SaveXmlUtf8File(std::ostream& fout);

		/**
		 * 加载 XML 字符串
		 * @param [in] xml		XML字符串Load
		 * @param [in] error		错误对象结构（默认为NULL）
		 * @return	返回加载 XML 字符串是否成功
		 * - true 加载成功， false 加载失败
		 */
		bool SetXmlString(LPCWSTR xml);

		/**
		 * 取得 XML 字符串
		 * @param [out] xml		XML字符串
		 * @param [in] error		错误对象结构（默认为NULL）
		 * @return	返回取得 XML 字符串是否成功
		 * - true 获取成功， false 获取失败
		 */
		bool GetXmlString(CString& xml, CString* error = NULL);

		/**
		 * 取得 XML 根结点
		 * @return	返回XML 根结点
		 */
		CXmlElement* GetElementRoot(void);

		CXmlElement* GetDocumentElement(void);

		/**
		 * 删除数据
		 */
		void Clear();

		/**
		 * 取得最近一次错误信息
		 * @return	返回最近一次错误信息
		 */
		LPCWSTR GetLastError(void);

		/**
		 * 保存时是否格式化文本
		 * @return	返回是否格式化文本
		 */
		bool GetIsFormatSave(void);

		/**
		 * 设置是否格式化保存文本
		 * @param [in] bFormat	是否格式化保存文本（默认为true）
		 */
		void SetIsFormatSave(bool bFormat = true);

		void* alloc;

	protected:
		bool load_buffer_impl(void* contents,
							  size_t size,
							  unsigned int options,
							  int encode,
							  bool is_mutable,
							  bool own);

		CXmlElement* _Doc_node;

		/** 根节点 */
		CXmlElement* _ElementRoot;

		/** 最后一次错误文本 */
		CString _LastError;
		/** 保存时是否格式化文本 (能提供比较好的可读性) */
		unsigned long _Option;
	};

	/** * @} */
	// end namespace
}

#endif // XMLDOCUMENT_H_