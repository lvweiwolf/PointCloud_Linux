#ifndef PLATFORM_H_
#define PLATFORM_H_

#ifdef WIN32
#else

#include <string>

namespace CLibToolkit {

	std::string CreateGuid();

	std::string GetAppModulePath();

	std::string GetSoftTempPath();

}

// Linux 下的 CA2T 实现
inline std::wstring CA2T_Compat(const char* asciiStr)
{
	if (!asciiStr)
		return L"";
	std::size_t wlen = std::mbstowcs(nullptr, asciiStr, 0);
	if (wlen == static_cast<std::size_t>(-1))
	{
		return L"";
	}
	std::wstring wstr(wlen + 1, L'\0');
	std::mbstowcs(&wstr[0], asciiStr, wstr.size());
	wstr.pop_back();
	return wstr;
};


// 为保持原有调用方式定义的宏（注意：返回临时对象，确保使用安全）
#define CA2T(str) CA2T_Compat(str).c_str()
#endif

#endif // PLATFORM_H_