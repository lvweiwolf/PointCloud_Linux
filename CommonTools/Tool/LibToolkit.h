//////////////////////////////////////////////////////////////////////
// 文件名称：LibToolkit.h
// 功能描述：对象工具
// 创建标识：吴建峰 2025/01/17
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////

#ifndef LIBTOOLKIT_H_
#define LIBTOOLKIT_H_

#include <include/CommonToolsExport.h>
#include <include/StringToolkit.h>


#ifdef _WIN32
#include <windows.h>
#include <objbase.h>
#else
// Linux下的类型定义
typedef void* HINSTANCE;
typedef void* HMODULE;
#endif


//! 应用程序辅助工具类（跨平台版本）
class COMMONTOOLS_EXPORT CLibToolkit
{
public:
	/**
	 * 生成 GUID/UUID
	 * @return  返回生成的GUID字符串（Windows下使用CoCreateGuid，Linux下使用libuuid或随机数生成）
	 */
	static CString CreateGuid(void);

	/**
	 * 取得当前可执行文件路径
	 * @param [in] hInst    模块的句柄（Windows用，Linux下可忽略）
	 * @return  返回模块路径，如（c:\a.exe 或 /usr/bin/app）
	 */
	static CString GetAppModuleFilename(HINSTANCE hInst = nullptr);

	/**
	 * 取得当前可执行文件所在目录
	 * @param [in] hInst    模块的句柄（Windows用，Linux下可忽略）
	 * @return  返回模块目录路径
	 */
	static CString GetAppModuleDir(HINSTANCE hInst = nullptr);

	/**
	 * 取得临时路径
	 * @return  返回临时路径字符串，如（c:\temp 或 /tmp）
	 */
	static CString GetTempPath(void);

	/**
	 * 创建临时文件
	 * @param [in] prefix   文件名前缀
	 * @return  返回临时文件完整路径
	 */
	static CString CreateTempFile(const CString& prefix = L"tmp");

	/**
	 * 取得系统错误信息
	 * @return  返回系统错误信息字符串
	 */
	static CString GetSystemLastError(void);

	/**
	 * 取得特定错误码的错误信息
	 * @param [in] errorCode 错误码
	 * @return  返回错误信息字符串
	 */
	static CString GetErrorString(int errorCode);

	// 文件系统相关函数
	static bool FileExists(const CString& filePath);
	static bool DirectoryExists(const CString& dirPath);
	static bool CreateDirectoryRecursive(const CString& dirPath);
	static CString GetCurrentDirectory();
	static bool SetCurrentDirectory(const CString& dirPath);

	// 路径处理函数
	static CString GetFileName(const CString& filePath);
	static CString GetFileNameWithoutExtension(const CString& filePath);
	static CString GetFileExtension(const CString& filePath);
	static CString GetDirectoryName(const CString& filePath);
	static CString CombinePath(const CString& path1, const CString& path2);

	// 环境变量函数
	static CString GetEnvironmentVariable(const CString& name);
	static bool SetEnvironmentVariable(const CString& name, const CString& value);

	// 进程相关函数
	static int GetCurrentProcessId();
	static CString GetUserName();
	static CString GetComputerName();

	// 时间相关函数
	static CString GetCurrentTimeString();
	static CString GetCurrentDateString();
	static long long GetFileLastModifiedTime(const CString& filePath);

	// 字符串转换函数
	static std::string WStringToString(const std::wstring& wstr);
	static std::wstring StringToWString(const std::string& str);
	static std::string CStringToString(const CString& cstr);
	static CString StringToCString(const std::string& str);

private:
	// 内部辅助函数
	static bool InitGuidGenerator();
	static std::string GenerateRandomHex(size_t length);
	static void EnsureTrailingSeparator(CString& path);

	// 跨平台路径分隔符
#ifdef _WIN32
	static const wchar_t PATH_SEPARATOR = L'\\';
#else
	static const wchar_t PATH_SEPARATOR = L'/';
#endif
};

#endif // LIBTOOLKIT_H_