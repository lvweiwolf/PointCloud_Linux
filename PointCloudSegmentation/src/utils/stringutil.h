//////////////////////////////////////////////////////////////////////
// 文件名称：stringutil.h
// 功能描述：字符串工具集
// 创建标识：吕伟	2021/2/4
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef STRINGUTIL_H_
#define STRINGUTIL_H_

#include <vector>
#include <string>

namespace d3s {
	namespace pcs {

		/**
		 *  @brief     可变参数格式化字符串
		 *
		 *  @param    std::string * dst
		 *  @param    const char * format
		 *  @param    va_list ap
		 *
		 *  @return   void
		 */
		void StringAppendV(std::string* dst, const char* format, va_list ap);

		/**
		 *  @brief    字符格式化
		 *
		 *  @param    const char * format
		 *  @param    ...
		 *  @return   std::string
		 */
		std::string StringPrintf(const char* format, ...);

		/**
		 *  @brief    字符串替换
		 *
		 *  @param    const std::string & str			原始字符串
		 *  @param    const std::string & old_str		待替换字符串
		 *  @param    const std::string & new_str		替换字符串
		 *  @return   std::string						新字符串
		 */
		std::string StringReplace(const std::string& str,
								  const std::string& old_str,
								  const std::string& new_str);

		/**
		 *  @brief    获得指定子串后的部分
		 *
		 *  @param    const std::string & str	原始字符串
		 *  @param    const std::string & key	指定子串
		 *  @return   std::string				指定子串后的部分
		 */
		std::string StringGetAfter(const std::string& str, const std::string& key);

		/**
		 *  @brief    字符串拆分
		 *
		 *  @param    const std::string & str	原始字符串
		 *  @param    const std::string & delim	拆分字符/子串
		 *  @return   std::vector<std::string>   拆分后单词列表
		 */
		std::vector<std::string> StringSplit(const std::string& str, const std::string& delim);

		/**
		 *  @brief    判断字符串是否有指定的前缀
		 *
		 *  @param    const std::string & str	原始字符串
		 *  @param    const std::string & prefix 前缀
		 *  @return   bool
		 */
		bool StringStartsWith(const std::string& str, const std::string& prefix);

		void StringTrim(std::string* str);
		void StringLeftTrim(std::string* str);
		void StringRightTrim(std::string* str);


		/**
		 *  @brief    大小写转换
		 *
		 *  @param    std::string * str 原始字符串
		 *  @return   void
		 */
		void StringToLower(std::string* str);
		void StringToUpper(std::string* str);

		/**
		 *  @brief    判断是否包含子串
		 *
		 *  @param    const std::string & str	原始字符串
		 *  @param    const std::string & sub_str	子串
		 *  @return   bool
		 */
		bool StringContains(const std::string& str, const std::string& sub_str);

		/**
		 *  @brief    字符集转换 UTF-8 <-> ANSI
		 *
		 *  @prarm	 const std::string & str
		 *
		 *  @return   std::string
		 */
		std::string FromUTF8(const std::string& str);

		std::string ToUTF8(const std::string& str);
	}
}

#endif // STRINGUTIL_H_