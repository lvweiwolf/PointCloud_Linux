//stdafx.h
#include "stringutil.h"
#include <stdarg.h>
#include <codecvt>
#include <stdexcept>
#include <locale>
#include <string>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#ifndef LPCWSTR
typedef const wchar_t* LPCWSTR;
#endif

namespace d3s {
	namespace pcs {
		namespace {

			bool IsNotWhiteSpace(const int character)
			{
				return character != ' ' && character != '\n' && character != '\r' &&
					   character != '\t';
			}

			std::string UnicodeToANSI(const std::wstring& str)
			{
				#ifdef WIN32
				char* pElementText;
				int iTextLen;
				// 宽字节转多字节
				iTextLen =
					WideCharToMultiByte(CP_ACP, 0, str.c_str(), -1, nullptr, 0, nullptr, nullptr);

				pElementText = new char[iTextLen + 1];
				memset((void*)pElementText, 0, sizeof(char) * (iTextLen + 1));
				::WideCharToMultiByte(CP_ACP,
									  0,
									  str.c_str(),
									  -1,
									  pElementText,
									  iTextLen,
									  nullptr,
									  nullptr);

				std::string strText;
				strText = pElementText;
				delete[] pElementText;
				pElementText = nullptr;
				return strText;

				#else


				if (str.empty()) {
					return "";
				}

				// 保存当前locale
				std::unique_ptr<char, void(*)(char*)> old_locale(
					std::setlocale(LC_ALL, nullptr),
					[](char* p) { if (p) free(p); }
				);

				// 设置locale为当前系统默认（模拟Windows的CP_ACP）
				if (std::setlocale(LC_ALL, "") == nullptr) {
					throw std::runtime_error("Failed to set locale");
				}

				// 计算所需多字节字符数量
				std::size_t mblen = std::wcstombs(nullptr, str.c_str(), 0);
				if (mblen == static_cast<std::size_t>(-1)) {
					// 恢复原来的locale
					if (old_locale) {
						std::setlocale(LC_ALL, old_locale.get());
					}
					return "";
				}

				// 分配缓冲区
				std::vector<char> buffer(mblen + 1, 0);

				// 执行转换
				if (std::wcstombs(buffer.data(), str.c_str(), buffer.size()) ==
					static_cast<std::size_t>(-1)) {
					// 恢复原来的locale
					if (old_locale) {
						std::setlocale(LC_ALL, old_locale.get());
					}
					return "";
				}

				// 恢复原来的locale
				if (old_locale) {
					std::setlocale(LC_ALL, old_locale.get());
				}

				return std::string(buffer.data());
				#endif
			}

			std::wstring AnsiToUNICODE(const std::string& str)
			{
				#ifdef WIN32
				wchar_t* pElementText;
				int iTextLen;
				// 宽字节转多字节
				iTextLen = MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, nullptr, 0);

				pElementText = new wchar_t[iTextLen + 1];
				memset((void*)pElementText, 0, sizeof(char) * (iTextLen + 1));
				::MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, pElementText, iTextLen);

				std::wstring strText;
				strText = pElementText;
				delete[] pElementText;
				pElementText = nullptr;
				return strText;
				#else
				// 保存当前locale
				std::unique_ptr<char, void(*)(char*)> old_locale(
					std::setlocale(LC_ALL, nullptr),
					[](char* p) { if (p) free(p); }
				);

				// 设置locale为当前系统默认（模拟Windows的CP_ACP）
				if (std::setlocale(LC_ALL, "") == nullptr) {
					throw std::runtime_error("Failed to set locale");
				}

				// 计算所需宽字符数量
				std::size_t wlen = std::mbstowcs(nullptr, str.c_str(), 0);
				if (wlen == static_cast<std::size_t>(-1)) {
					// 恢复原来的locale
					if (old_locale) {
						std::setlocale(LC_ALL, old_locale.get());
					}
					return L"";
				}

				// 分配缓冲区
				std::vector<wchar_t> buffer(wlen + 1, 0);

				// 执行转换
				if (std::mbstowcs(buffer.data(), str.c_str(), buffer.size()) ==
					static_cast<std::size_t>(-1)) {
					// 恢复原来的locale
					if (old_locale) {
						std::setlocale(LC_ALL, old_locale.get());
					}
					return L"";
				}

				// 恢复原来的locale
				if (old_locale) {
					std::setlocale(LC_ALL, old_locale.get());
				}

				return std::wstring(buffer.data());
				#endif // WIN32
			}

			std::string UnicodeToUTF8(LPCWSTR lpszWideStr)
			{
				#ifdef WIN32
				int nLen = ::WideCharToMultiByte(CP_UTF8,
												 0,
												 lpszWideStr,
												 -1,
												 nullptr,
												 0,
												 nullptr,
												 nullptr);

				char* buffer = new char[nLen + 1];
				::ZeroMemory(buffer, nLen + 1);

				::WideCharToMultiByte(CP_UTF8, 0, lpszWideStr, -1, buffer, nLen, nullptr, nullptr);

				std::string multStr = buffer;
				delete[] buffer;
				buffer = nullptr;
				return multStr;
				#else
				if (lpszWideStr == nullptr) {
					return "";
				}
				try {
					std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
					return converter.to_bytes(lpszWideStr);
				}
				catch (const std::range_error& e) {
					// 转换失败时返回空字符串
					return "";
				}
				#endif
			}

			std::wstring Utf8ToUnicode(const std::string& str)
			{
				#ifdef WIN32
				int nLen = ::MultiByteToWideChar(CP_UTF8, 0, str.c_str(), str.length(), nullptr, 0);

				WCHAR* buffer = new WCHAR[nLen + 1];
				::ZeroMemory(buffer, sizeof(WCHAR) * (nLen + 1));

				::MultiByteToWideChar(CP_UTF8, 0, str.c_str(), str.length(), buffer, nLen);

				std::wstring wideStr = buffer;
				delete[] buffer;
				buffer = nullptr;
				return wideStr;
				#else
				std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
				return converter.from_bytes(str);
				#endif
				
			}

		} // namespace

		void StringAppendV(std::string* dst, const char* format, va_list ap)
		{
			// 先尝试固定缓冲区大小
			static const int kFixedBufferSize = 1024;
			char fixed_buffer[kFixedBufferSize];

			// 使用va_list的方法有可能在使用时使其中的数据无效。
			// 解决方法是在使用该结构之前，先制作一份副本，然后使用该副本
			va_list backup_ap;
			va_copy(backup_ap, ap);
			int result = vsnprintf(fixed_buffer, kFixedBufferSize, format, backup_ap);
			va_end(backup_ap);

			if (result < kFixedBufferSize)
			{
				if (result >= 0)
				{
					dst->append(fixed_buffer, result);
					return;
				}

#ifdef _MSC_VER
				va_copy(backup_ap, ap);
				result = vsnprintf(nullptr, 0, format, backup_ap);
				va_end(backup_ap);
#endif

				if (result < 0)
				{
					return;
				}
			}

			// 将缓冲区的大小增加到vsnprintf所要求的大小，再加上一个结束符 0
			const int variable_buffer_size = result + 1;
			std::unique_ptr<char[]> variable_buffer(new char[variable_buffer_size]);

			// 再次使用之前，请还原va_list
			va_copy(backup_ap, ap);
			result = vsnprintf(variable_buffer.get(), variable_buffer_size, format, backup_ap);
			va_end(backup_ap);

			if (result >= 0 && result < variable_buffer_size)
			{
				dst->append(variable_buffer.get(), result);
			}
		}

		std::string StringPrintf(const char* format, ...)
		{
			va_list ap;
			va_start(ap, format);
			std::string result;
			StringAppendV(&result, format, ap);
			va_end(ap);
			return result;
		}

		std::string StringReplace(const std::string& str,
								  const std::string& old_str,
								  const std::string& new_str)
		{
			if (old_str.empty())
			{
				return str;
			}
			size_t position = 0;
			std::string mod_str = str;
			while ((position = mod_str.find(old_str, position)) != std::string::npos)
			{
				mod_str.replace(position, old_str.size(), new_str);
				position += new_str.size();
			}
			return mod_str;
		}

		std::string StringGetAfter(const std::string& str, const std::string& key)
		{
			if (key.empty())
			{
				return str;
			}
			std::size_t found = str.rfind(key);
			if (found != std::string::npos)
			{
				return str.substr(found + key.length(), str.length() - (found + key.length()));
			}
			return "";
		}

		std::vector<std::string> StringSplit(const std::string& str, const std::string& delim)
		{
			std::vector<std::string> elems;
			boost::split(elems, str, boost::is_any_of(delim), boost::token_compress_on);
			return elems;
		}

		bool StringStartsWith(const std::string& str, const std::string& prefix)
		{
			return !prefix.empty() && prefix.size() <= str.size() &&
				   str.substr(0, prefix.size()) == prefix;
		}

		void StringLeftTrim(std::string* str)
		{
			str->erase(str->begin(), std::find_if(str->begin(), str->end(), IsNotWhiteSpace));
		}

		void StringRightTrim(std::string* str)
		{
			str->erase(std::find_if(str->rbegin(), str->rend(), IsNotWhiteSpace).base(),
					   str->end());
		}

		void StringTrim(std::string* str)
		{
			StringLeftTrim(str);
			StringRightTrim(str);
		}

		void StringToLower(std::string* str)
		{
			std::transform(str->begin(), str->end(), str->begin(), ::tolower);
		}

		void StringToUpper(std::string* str)
		{
			std::transform(str->begin(), str->end(), str->begin(), ::toupper);
		}

		bool StringContains(const std::string& str, const std::string& sub_str)
		{
			return str.find(sub_str) != std::string::npos;
		}

		std::string FromUTF8(const std::string& str) { return UnicodeToANSI(Utf8ToUnicode(str)); }

		std::string ToUTF8(const std::string& str)
		{
			return UnicodeToUTF8(AnsiToUNICODE(str).c_str());
		}

	}
}
