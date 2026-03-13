//////////////////////////////////////////////////////////////////////
// 文件名称：ModelNodeXmlHelper.h
// 功能描述：模型节点XML/XMDX文件帮助类
// 创建标识：wjf 2025/01/20
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef MODELNODEHELPER_H_
#define MODELNODEHELPER_H_

// 使用跨平台的XML解析库
#if defined(_WIN32)
#include <tinyxml2.h>
using namespace tinyxml2;
#else
// Linux下使用pugixml
#include <pugixml.hpp>
using namespace pugi;
#endif


#include <include/CommonToolsExport.h>
#include <include/StringToolkit.h>
#include <include/ModelNode.h>

namespace pc {
	namespace data {
		class COMMONTOOLS_EXPORT CModelNodeXmlHelper
		{
		public:
			/**
			 * @brief 从XMDX文件加载模型节点树
			 * @param[in] strFilePath 文件路径
			 * @param[out] pRootNode 根节点指针
			 * @return bool 成功返回true，失败返回false
			 */
			static bool LoadFromFile(const CString& strFilePath, CModelNodePtr& pRootNode);

			/**
			 * @brief 保存模型节点树到XMDX文件
			 * @param[in] strFilePath 文件路径
			 * @param[in] pRootNode 根节点指针
			 * @return bool 成功返回true，失败返回false
			 */
			static bool SaveToFile(const CString& strFilePath, CModelNodePtr pRootNode);

			/**
			 * @brief 从XML字符串加载模型节点树
			 * @param[in] strXml XML字符串
			 * @param[out] pRootNode 根节点指针
			 * @return bool 成功返回true，失败返回false
			 */
			static bool LoadFromString(const CString& strXml, CModelNodePtr& pRootNode);

			/**
			 * @brief 将模型节点树保存为XML字符串
			 * @param[in] pRootNode 根节点指针
			 * @param[out] strXml 输出的XML字符串
			 * @return bool 成功返回true，失败返回false
			 */
			static bool SaveToString(CModelNodePtr pRootNode, CString& strXml);

		private:
			/**
			 * @brief 递归加载XML节点为模型节点
			 * @param[in] xmlNode XML节点
			 * @param[out] pModelNode 模型节点指针
			 * @return bool 成功返回true，失败返回false
			 */
			static bool LoadXmlNode(const xml_node* pXmlNode, CModelNodePtr pModelNode);

			/**
			 * @brief 递归保存模型节点为XML节点
			 * @param[in] pModelNode 模型节点指针
			 * @param[out] xmlNode XML节点
			 * @return bool 成功返回true，失败返回false
			 */
			static bool SaveXmlNode(CModelNodePtr pModelNode, xml_node* pXmlNode);

			/**
			 * @brief 转换CString到UTF8字符串（跨平台处理）
			 * @param[in] str CString字符串
			 * @return std::string UTF8字符串
			 */
			static std::string CStringToUTF8(const CString& str);

			/**
			 * @brief 转换UTF8字符串到CString（跨平台处理）
			 * @param[in] utf8Str UTF8字符串
			 * @return CString CString字符串
			 */
			static CString UTF8ToCString(const std::string& utf8Str);

			/**
			 * @brief 获取XML节点的属性值
			 * @param[in] pXmlNode XML节点
			 * @param[in] attrName 属性名
			 * @return CString 属性值
			 */
			static CString GetXmlAttribute(const xml_node* pXmlNode, const char* attrName);

			/**
			 * @brief 设置XML节点的属性值
			 * @param[in] pXmlNode XML节点
			 * @param[in] attrName 属性名
			 * @param[in] value 属性值
			 */
			static void SetXmlAttribute(xml_node* pXmlNode,
										const char* attrName,
										const CString& value);

			/**
			 * @brief 创建XML子节点
			 * @param[in] pParentXmlNode 父XML节点
			 * @param[in] nodeName 节点名
			 * @return xml_node* 创建的XML节点指针
			 */
			static xml_node* CreateXmlChildNode(xml_node* pParentXmlNode, const char* nodeName);

			/**
			 * @brief 将CAny值转换为字符串
			 * @param[in] anyValue CAny值
			 * @return CString 字符串表示
			 */
			static CString AnyToString(const CAny& anyValue);

			/**
			 * @brief 将字符串转换为CAny值
			 * @param[in] strValue 字符串值
			 * @param[in] valueType 值类型
			 * @return CAny CAny值
			 */
			static CAny StringToAny(const CString& strValue, EAnyType valueType);

			/**
			 * @brief 获取属性类型字符串
			 * @param[in] valueType 值类型枚举
			 * @return CString 类型字符串
			 */
			static CString GetTypeString(EAnyType valueType);

			/**
			 * @brief 从字符串获取属性类型
			 * @param[in] typeStr 类型字符串
			 * @return EAnyType 值类型枚举
			 */
			static EAnyType GetTypeFromString(const CString& typeStr);
		};
	}
}

#endif // MODELNODEHELPER_H_
