#include <Tool/ModelNodeHelper.h>
#include <Tool/LibToolkit.h>

#include <sstream>
#include <fstream>

#if defined(__GNUC__)
#include <locale>
#include <codecvt>
#endif

namespace pc {
	namespace data {
		bool CModelNodeXmlHelper::LoadFromFile(const CString& strFilePath, CModelNodePtr& pRootNode)
		{
			// 检查文件路径是否有效
			if (strFilePath.IsEmpty())
				return false;

			// 清空现有根节点
			pRootNode = nullptr;

			try
			{
				// 转换文件路径为UTF8（跨平台处理）
				std::string utf8Path = CStringToUTF8(strFilePath);

				// 检查文件是否存在
				std::ifstream fileTest(utf8Path);
				if (!fileTest.good())
				{
					return false;
				}
				fileTest.close();

#if defined(_WIN32)
				// Windows下使用TinyXML2
				XMLDocument xmlDoc;
				XMLError error = xmlDoc.LoadFile(utf8Path.c_str());
				if (error != XML_SUCCESS)
				{
					return false;
				}

				// 获取根节点
				XMLElement* pRootElement = xmlDoc.RootElement();
				if (!pRootElement)
				{
					return false;
				}

				// 创建根模型节点
				int nodeType = 0;
				const char* typeStr = pRootElement->Attribute("NodeType");
				if (typeStr)
				{
					nodeType = atoi(typeStr);
				}

				pRootNode = new CModelNode(nodeType);

				// 递归加载XML节点
				return LoadXmlNode(pRootElement, pRootNode);
#else
				// Linux下使用pugixml
				xml_document xmlDoc;
				xml_parse_result result = xmlDoc.load_file(utf8Path.c_str());
				if (!result)
				{
					return false;
				}

				// 获取根节点
				xml_node rootNode = xmlDoc.child("ModelNode");
				if (!rootNode)
				{
					return false;
				}

				// 创建根模型节点
				int nodeType = 0;
				const char* typeStr = rootNode.attribute("NodeType").value();
				if (typeStr && strlen(typeStr) > 0)
				{
					nodeType = atoi(typeStr);
				}

				pRootNode = new CModelNode(nodeType);

				// 递归加载XML节点
				return LoadXmlNode(&rootNode, pRootNode);
#endif
			}
			catch (...)
			{
				return false;
			}
		}

		bool CModelNodeXmlHelper::SaveToFile(const CString& strFilePath, CModelNodePtr pRootNode)
		{
			if (strFilePath.IsEmpty() || !pRootNode)
				return false;

			try
			{
				// 转换文件路径为UTF8
				std::string utf8Path = CStringToUTF8(strFilePath);

#if defined(_WIN32)
				// Windows下使用TinyXML2
				XMLDocument xmlDoc;

				// 创建XML声明
				XMLDeclaration* pDecl = xmlDoc.NewDeclaration();
				xmlDoc.InsertFirstChild(pDecl);

				// 创建根元素
				XMLElement* pRootElement = xmlDoc.NewElement("ModelNode");
				xmlDoc.InsertEndChild(pRootElement);

				// 递归保存模型节点
				if (!SaveXmlNode(pRootNode, pRootElement))
				{
					return false;
				}

				// 保存到文件
				XMLError error = xmlDoc.SaveFile(utf8Path.c_str());
				return (error == XML_SUCCESS);
#else
				// Linux下使用pugixml
				xml_document xmlDoc;

				// 添加XML声明
				xml_node decl = xmlDoc.prepend_child(node_declaration);
				decl.append_attribute("version") = "1.0";
				decl.append_attribute("encoding") = "UTF-8";

				// 创建根元素
				xml_node rootNode = xmlDoc.append_child("ModelNode");

				// 递归保存模型节点
				if (!SaveXmlNode(pRootNode, &rootNode))
				{
					return false;
				}

				// 保存到文件
				return xmlDoc.save_file(utf8Path.c_str());
#endif
			}
			catch (...)
			{
				return false;
			}
		}

		bool CModelNodeXmlHelper::LoadFromString(const CString& strXml, CModelNodePtr& pRootNode)
		{
			if (strXml.IsEmpty())
				return false;

			// 清空现有根节点
			pRootNode = nullptr;

			try
			{
				std::string utf8Xml = CStringToUTF8(strXml);

#if defined(_WIN32)
				// Windows下使用TinyXML2
				XMLDocument xmlDoc;
				XMLError error = xmlDoc.Parse(utf8Xml.c_str());
				if (error != XML_SUCCESS)
				{
					return false;
				}

				// 获取根节点
				XMLElement* pRootElement = xmlDoc.RootElement();
				if (!pRootElement)
				{
					return false;
				}

				// 创建根模型节点
				int nodeType = 0;
				const char* typeStr = pRootElement->Attribute("NodeType");
				if (typeStr)
				{
					nodeType = atoi(typeStr);
				}

				pRootNode = new CModelNode(nodeType);

				// 递归加载XML节点
				return LoadXmlNode(pRootElement, pRootNode);
#else
				// Linux下使用pugixml
				xml_document xmlDoc;
				xml_parse_result result = xmlDoc.load_string(utf8Xml.c_str());
				if (!result)
				{
					return false;
				}

				// 获取根节点
				xml_node rootNode = xmlDoc.child("ModelNode");
				if (!rootNode)
				{
					return false;
				}

				// 创建根模型节点
				int nodeType = 0;
				const char* typeStr = rootNode.attribute("NodeType").value();
				if (typeStr && strlen(typeStr) > 0)
				{
					nodeType = atoi(typeStr);
				}

				pRootNode = new CModelNode(nodeType);

				// 递归加载XML节点
				return LoadXmlNode(&rootNode, pRootNode);
#endif
			}
			catch (...)
			{
				return false;
			}
		}

		bool CModelNodeXmlHelper::SaveToString(CModelNodePtr pRootNode, CString& strXml)
		{
			if (!pRootNode)
				return false;

			try
			{
#if defined(_WIN32)
				// Windows下使用TinyXML2
				XMLDocument xmlDoc;

				// 创建XML声明
				XMLDeclaration* pDecl =
					xmlDoc.NewDeclaration("xml version=\"1.0\" encoding=\"UTF-8\"");
				xmlDoc.InsertFirstChild(pDecl);

				// 创建根元素
				XMLElement* pRootElement = xmlDoc.NewElement("ModelNode");
				xmlDoc.InsertEndChild(pRootElement);

				// 递归保存模型节点
				if (!SaveXmlNode(pRootNode, pRootElement))
				{
					return false;
				}

				// 获取XML字符串
				XMLPrinter printer;
				xmlDoc.Print(&printer);
				strXml = UTF8ToCString(printer.CStr());

				return true;
#else
				// Linux下使用pugixml
				xml_document xmlDoc;

				// 添加XML声明
				xml_node decl = xmlDoc.prepend_child(node_declaration);
				decl.append_attribute("version") = "1.0";
				decl.append_attribute("encoding") = "UTF-8";

				// 创建根元素
				xml_node rootNode = xmlDoc.append_child("ModelNode");

				// 递归保存模型节点
				if (!SaveXmlNode(pRootNode, &rootNode))
				{
					return false;
				}

				// 获取XML字符串
				std::stringstream ss;
				xmlDoc.save(ss);
				strXml = UTF8ToCString(ss.str());

				return true;
#endif
			}
			catch (...)
			{
				return false;
			}
		}

		bool CModelNodeXmlHelper::LoadXmlNode(const xml_node* pXmlNode, CModelNodePtr pModelNode)
		{
			if (!pXmlNode || !pModelNode)
				return false;

			try
			{
				// 获取节点ID
				CString strId = GetXmlAttribute(pXmlNode, "ID");
				if (!strId.IsEmpty())
				{
					pModelNode->SetId(strId);
				}

				// 获取节点名称
				CString strName = GetXmlAttribute(pXmlNode, "NodeName");
				if (!strName.IsEmpty())
				{
					pModelNode->SetNodeName(strName);
				}

				// 加载属性数据
#if defined(_WIN32)
				const XMLElement* pElement = static_cast<const XMLElement*>(pXmlNode);
				const XMLAttribute* pAttr = pElement->FirstAttribute();
				while (pAttr)
				{
					CString attrName = UTF8ToCString(pAttr->Name());
					CString attrValue = UTF8ToCString(pAttr->Value());

					// 跳过系统属性
					if (attrName != "NodeType" && attrName != "ID" && attrName != "NodeName")
					{
						// 检查是否有对应的类型属性
						CString typeAttrName = attrName + L"_Type";
						CString typeValue =
							GetXmlAttribute(pXmlNode, CStringToUTF8(typeAttrName).c_str());

						if (!typeValue.IsEmpty())
						{
							// 有类型信息，使用类型转换
							EAnyType valueType = GetTypeFromString(typeValue);
							CAny anyValue = StringToAny(attrValue, valueType);
							pModelNode->SetData(attrName, anyValue);
						}
						else
						{
							// 无类型信息，当作字符串处理
							pModelNode->SetData(attrName, attrValue);
						}
					}

					pAttr = pAttr->Next();
				}
#else
				const xml_node* pNode = static_cast<const xml_node*>(pXmlNode);
				for (xml_attribute attr = pNode->first_attribute(); attr;
					 attr = attr.next_attribute())
				{
					CString attrName = UTF8ToCString(attr.name());
					CString attrValue = UTF8ToCString(attr.value());

					// 跳过系统属性
					if (attrName != L"NodeType" && attrName != L"ID" && attrName != L"NodeName")
					{
						// 检查是否有对应的类型属性
						CString typeAttrName = attrName + L"_Type";
						CString typeValue =
							GetXmlAttribute(pXmlNode, CStringToUTF8(typeAttrName).c_str());

						if (!typeValue.IsEmpty())
						{
							// 有类型信息，使用类型转换
							EAnyType valueType = GetTypeFromString(typeValue);
							CAny anyValue = StringToAny(attrValue, valueType);
							pModelNode->SetData(attrName, anyValue);
						}
						else
						{
							// 无类型信息，当作字符串处理
							pModelNode->SetData(attrName, attrValue);
						}
					}
				}
#endif

				// 递归加载子节点
#if defined(_WIN32)
				const XMLElement* pChildElement = pElement->FirstChildElement("ModelNode");
				while (pChildElement)
				{
					// 获取子节点类型
					int childType = 0;
					const char* typeStr = pChildElement->Attribute("NodeType");
					if (typeStr)
					{
						childType = atoi(typeStr);
					}

					// 创建子模型节点
					CModelNodePtr pChildNode =
						pModelNode->InsertNode(pModelNode->getChildCount(), childType);
					if (pChildNode)
					{
						// 递归加载子节点
						LoadXmlNode(pChildElement, pChildNode);
					}

					pChildElement = pChildElement->NextSiblingElement("ModelNode");
				}
#else
				xml_node childNode = pNode->child("ModelNode");
				while (childNode)
				{
					// 获取子节点类型
					int childType = 0;
					const char* typeStr = childNode.attribute("NodeType").value();
					if (typeStr && strlen(typeStr) > 0)
					{
						childType = atoi(typeStr);
					}

					// 创建子模型节点
					CModelNodePtr pChildNode =
						pModelNode->InsertNode(pModelNode->getChildCount(), childType);
					if (pChildNode)
					{
						// 递归加载子节点
						LoadXmlNode(&childNode, pChildNode);
					}

					childNode = childNode.next_sibling("ModelNode");
				}
#endif

				return true;
			}
			catch (...)
			{
				return false;
			}
		}

		bool CModelNodeXmlHelper::SaveXmlNode(CModelNodePtr pModelNode, xml_node* pXmlNode)
		{
			if (!pModelNode || !pXmlNode)
				return false;

			try
			{
				// 设置节点基本属性
				SetXmlAttribute(pXmlNode,
								"NodeType",
								CStringToolkit::IntToStr(pModelNode->getNodeTypeId()));
				SetXmlAttribute(pXmlNode, "ID", pModelNode->GetId());
				SetXmlAttribute(pXmlNode, "NodeName", pModelNode->GetNodeName());

				// 保存节点数据属性
				const CModelNodeDataMap& dataMap = pModelNode->GetDataMap();
				for (CModelNodeDataMapCIter it = dataMap.begin(); it != dataMap.end(); ++it)
				{
					CString key = it->first;
					const CAny& value = it->second;

					// 将CAny转换为字符串
					CString strValue = AnyToString(value);

					// 保存值
					SetXmlAttribute(pXmlNode, CStringToUTF8(key).c_str(), strValue);

					// 保存类型信息
					CString typeStr = GetTypeString(value.GetType());
					CString typeKey = key + L"_Type";
					SetXmlAttribute(pXmlNode, CStringToUTF8(typeKey).c_str(), typeStr);
				}

				// 递归保存子节点
				size_t childCount = pModelNode->getChildCount();
				for (size_t i = 0; i < childCount; ++i)
				{
					CModelNodePtr pChildNode = pModelNode->GetChildNode(i);
					if (pChildNode)
					{
						// 创建XML子节点
						// xml_node* pChildXmlNode = CreateXmlChildNode(pXmlNode, "ModelNode");
						xml_node pChildXmlNode = pXmlNode->append_child("ModelNode");
						if (pChildXmlNode)
						{
							// 递归保存子节点
							SaveXmlNode(pChildNode, &pChildXmlNode);
						}
					}
				}

				return true;
			}
			catch (...)
			{
				return false;
			}
		}

		std::string CModelNodeXmlHelper::CStringToUTF8(const CString& str)
		{
			if (str.IsEmpty())
				return "";

#if defined(_WIN32)
			// Windows: CString是wchar_t，转换为UTF8
			int utf8Length = WideCharToMultiByte(CP_UTF8, 0, str, -1, NULL, 0, NULL, NULL);
			if (utf8Length == 0)
				return "";

			std::string utf8Str(utf8Length, 0);
			WideCharToMultiByte(CP_UTF8, 0, str, -1, &utf8Str[0], utf8Length, NULL, NULL);
			utf8Str.resize(utf8Length - 1); // 去除末尾的\0

			return utf8Str;
#else
			// Linux: CString是wchar_t，使用codecvt转换为UTF8
			LPCWSTR wstr = str;
			std::wstring wstrS(wstr);
			std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
			return converter.to_bytes(wstrS);
#endif
		}

		CString CModelNodeXmlHelper::UTF8ToCString(const std::string& utf8Str)
		{
			if (utf8Str.empty())
				return CString();

#if defined(_WIN32)
			// Windows: UTF8转换为wchar_t
			int wcharLength = MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, NULL, 0);
			if (wcharLength == 0)
				return CString();

			std::wstring wstr(wcharLength, 0);
			MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, &wstr[0], wcharLength);
			wstr.resize(wcharLength - 1); // 去除末尾的\0

			return CString(wstr.c_str());
#else
			// Linux: UTF8转换为wchar_t
			std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
			std::wstring wstr = converter.from_bytes(utf8Str);
			return CString(wstr.c_str());
#endif
		}

		CString CModelNodeXmlHelper::GetXmlAttribute(const xml_node* pXmlNode, const char* attrName)
		{
			if (!pXmlNode || !attrName)
				return CString();

#if defined(_WIN32)
			const XMLElement* pElement = static_cast<const XMLElement*>(pXmlNode);
			const char* value = pElement->Attribute(attrName);
			if (value)
			{
				return UTF8ToCString(value);
			}
#else
			const xml_node* pNode = static_cast<const xml_node*>(pXmlNode);
			const char* value = pNode->attribute(attrName).value();
			if (value && strlen(value) > 0)
			{
				return UTF8ToCString(value);
			}
#endif

			return CString();
		}

		void CModelNodeXmlHelper::SetXmlAttribute(xml_node* pXmlNode,
												  const char* attrName,
												  const CString& value)
		{
			if (!pXmlNode || !attrName)
				return;

			std::string utf8Value = CStringToUTF8(value);
			std::string utf8Name = attrName;

#if defined(_WIN32)
			XMLElement* pElement = static_cast<XMLElement*>(pXmlNode);
			pElement->SetAttribute(utf8Name.c_str(), utf8Value.c_str());
#else
			xml_node* pNode = static_cast<xml_node*>(pXmlNode);
			pNode->append_attribute(utf8Name.c_str()) = utf8Value.c_str();
#endif
		}

		xml_node* CModelNodeXmlHelper::CreateXmlChildNode(xml_node* pParentXmlNode,
														  const char* nodeName)
		{
			if (!pParentXmlNode || !nodeName)
				return nullptr;

#if defined(_WIN32)
			XMLElement* pParentElement = static_cast<XMLElement*>(pParentXmlNode);
			XMLDocument* pDoc = pParentElement->GetDocument();
			if (!pDoc)
				return nullptr;

			XMLElement* pChildElement = pDoc->NewElement(nodeName);
			pParentElement->InsertEndChild(pChildElement);
			return pChildElement;
#else
			xml_node* pParentNode = static_cast<xml_node*>(pParentXmlNode);
			xml_node childNode = pParentNode->append_child(nodeName);
			return &childNode;
#endif
		}

		CString CModelNodeXmlHelper::AnyToString(const CAny& anyValue)
		{
			if (anyValue.IsEmpty())
				return CString();

			// 使用CAny的转换运算符
			return anyValue;
		}

		CAny CModelNodeXmlHelper::StringToAny(const CString& strValue, EAnyType valueType)
		{
			if (strValue.IsEmpty())
				return CAny();

			CAny result;
			result.SetStringValue(strValue, valueType);
			return result;
		}

		CString CModelNodeXmlHelper::GetTypeString(EAnyType valueType)
		{
			switch (valueType)
			{
			case ANY_EMPTY:
				return L"EMPTY";
			case ANY_BOOL:
				return L"BOOL";
			case ANY_INT:
				return L"INT";
			case ANY_INT64:
				return L"INT64";
			case ANY_DOUBLE:
				return L"DOUBLE";
			case ANY_STRING:
				return L"STRING";
			case ANY_POINT:
				return L"POINT";
			case ANY_GUID:
				return L"GUID";
			case ANY_STREAM:
				return L"STREAM";
			case ANY_PTARRAY:
				return L"PTARRAY";
			case ANY_MATRIX:
				return L"MATRIX";
			case ANY_POINTDS:
				return L"POINTDS";
			default:
				return L"UNKNOWN";
			}
		}

		EAnyType CModelNodeXmlHelper::GetTypeFromString(const CString& typeStr)
		{
			if (typeStr == L"BOOL")
				return ANY_BOOL;
			if (typeStr == L"INT")
				return ANY_INT;
			if (typeStr == L"INT64")
				return ANY_INT64;
			if (typeStr == L"DOUBLE")
				return ANY_DOUBLE;
			if (typeStr == L"STRING")
				return ANY_STRING;
			if (typeStr == L"POINT")
				return ANY_POINT;
			if (typeStr == L"GUID")
				return ANY_GUID;
			if (typeStr == L"STREAM")
				return ANY_STREAM;
			if (typeStr == L"PTARRAY")
				return ANY_PTARRAY;
			if (typeStr == L"MATRIX")
				return ANY_MATRIX;
			if (typeStr == L"POINTDS")
				return ANY_POINTDS;

			return ANY_EMPTY;
		}
	}
}