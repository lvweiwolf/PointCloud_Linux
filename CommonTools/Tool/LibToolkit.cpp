#include <Tool/LibToolkit.h>

// GNU
#include <unistd.h>
#include <dlfcn.h>
#include <pwd.h>
#include <sys/stat.h>

// STL
#include <random>

// 全局变量：GUID生成器是否已初始化
static bool g_bGuidGeneratorInitialized = false;

/** 生成GUID/UUID */
CString CLibToolkit::CreateGuid(void)
{
	InitGuidGenerator();

	CString result;

#ifdef _WIN32
	// Windows实现
	GUID guid;
	CoCreateGuid(&guid);
	wchar_t buf[64] = { 0 };

	_snwprintf(buf,
			   sizeof(buf) / sizeof(wchar_t),
			   L"%08X%04X%04X%02X%02X%02X%02X%02X%02X%02X%02X",
			   guid.Data1,
			   guid.Data2,
			   guid.Data3,
			   guid.Data4[0],
			   guid.Data4[1],
			   guid.Data4[2],
			   guid.Data4[3],
			   guid.Data4[4],
			   guid.Data4[5],
			   guid.Data4[6],
			   guid.Data4[7]);

	result = buf;
#else
	// Linux实现：使用libuuid或随机数生成
#ifdef HAVE_UUID_UUID_H
	// 使用libuuid库
	uuid_t uuid;
	char uuid_str[37]; // UUID字符串长度（36个字符 + 终止符）

	uuid_generate(uuid);
	uuid_unparse(uuid, uuid_str);

	// 转换为大写并移除连字符
	std::string uuid_string = uuid_str;
	std::transform(uuid_string.begin(), uuid_string.end(), uuid_string.begin(), ::toupper);
	uuid_string.erase(std::remove(uuid_string.begin(), uuid_string.end(), '-'), uuid_string.end());

	result = StringToCString(uuid_string);
#else
	// 如果没有libuuid，使用随机数生成模拟GUID
	std::string guid_str;

	// 第一部分：8位十六进制
	guid_str += GenerateRandomHex(8);

	// 第二、三部分：各4位十六进制
	guid_str += GenerateRandomHex(4);
	guid_str += GenerateRandomHex(4);

	// 第四部分：2位十六进制
	guid_str += GenerateRandomHex(2);

	// 第五部分：6位十六进制
	guid_str += GenerateRandomHex(6);

	result = StringToCString(guid_str);
#endif
#endif

	return result;
}

/** 取得当前可执行文件路径 */
CString CLibToolkit::GetAppModuleFilename(HINSTANCE hInst)
{
#ifdef _WIN32
	// Windows实现
	wchar_t m_pstr[MAX_PATH];
	::GetModuleFileName(hInst, m_pstr, MAX_PATH);
	return m_pstr;
#else
	// Linux实现
	char exePath[PATH_MAX] = { 0 };
	ssize_t count = readlink("/proc/self/exe", exePath, PATH_MAX - 1);

	if (count != -1)
	{
		exePath[count] = '\0';
		return StringToCString(exePath);
	}
	else
	{
		// 备用方法：使用dladdr
		Dl_info dl_info;
		if (dladdr((void*)GetAppModuleFilename, &dl_info))
		{
			return StringToCString(dl_info.dli_fname);
		}
	}

	// 如果都失败，返回当前目录
	return GetCurrentDirectory();
#endif
}

/** 取得当前可执行文件所在目录 */
CString CLibToolkit::GetAppModuleDir(HINSTANCE hInst)
{
	CString exePath = GetAppModuleFilename(hInst);
	return GetDirectoryName(exePath);
}

/** 取得临时路径 */
CString CLibToolkit::GetTempPath(void)
{
#ifdef _WIN32
	// Windows实现
	wchar_t m_pstr[MAX_PATH];
	DWORD dwLength = ::GetTempPathW(MAX_PATH, m_pstr);
	m_pstr[dwLength] = L'\0';
	return m_pstr;
#else
	// Linux实现
	const char* tempDir = nullptr;

	// 按优先级检查环境变量
	tempDir = getenv("TMPDIR");
	if (!tempDir || strlen(tempDir) == 0)
		tempDir = getenv("TEMP");
	if (!tempDir || strlen(tempDir) == 0)
		tempDir = getenv("TMP");
	if (!tempDir || strlen(tempDir) == 0)
		tempDir = "/tmp"; // 默认值

	CString result = StringToCString(tempDir);
	EnsureTrailingSeparator(result);
	return result;
#endif
}

/** 创建临时文件 */
CString CLibToolkit::CreateTempFile(const CString& prefix)
{
	CString tempDir = GetTempPath();
	CString fileName;

#ifdef _WIN32
	// Windows实现
	wchar_t tempFileName[MAX_PATH];
	wchar_t prefixBuf[MAX_PATH];

	wcsncpy(prefixBuf, prefix, MAX_PATH - 1);
	prefixBuf[MAX_PATH - 1] = L'\0';

	if (GetTempFileNameW(tempDir, prefixBuf, 0, tempFileName) == 0)
	{
		return L"";
	}

	fileName = tempFileName;
#else
	// Linux实现
	std::string prefixStr = CStringToString(prefix);
	std::string tempDirStr = CStringToString(tempDir);

	// 生成随机文件名
	char templateStr[PATH_MAX];
	snprintf(templateStr, sizeof(templateStr), "%s%sXXXXXX", tempDirStr.c_str(), prefixStr.c_str());

	// 创建临时文件
	int fd = mkstemp(templateStr);
	if (fd != -1)
	{
		close(fd);
		fileName = StringToCString(templateStr);
	}
#endif

	return fileName;
}

/** 取得系统错误信息 */
CString CLibToolkit::GetSystemLastError(void)
{
	return GetErrorString(
#ifdef _WIN32
		GetLastError()
#else
		errno
#endif
	);
}

/** 取得特定错误码的错误信息 */
CString CLibToolkit::GetErrorString(int errorCode)
{
#ifdef _WIN32
	// Windows实现
	CString str;
	LPVOID lpMsgBuf = nullptr;

	DWORD flags =
		FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS;

	DWORD result = FormatMessageW(flags,
								  NULL,
								  errorCode,
								  MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
								  (LPWSTR)&lpMsgBuf,
								  0,
								  NULL);

	if (result > 0 && lpMsgBuf)
	{
		str = (LPWSTR)lpMsgBuf;
		LocalFree(lpMsgBuf);
	}

	// 移除末尾的换行符
	str.TrimRight();

	return str;
#else
	// Linux实现
	char buffer[256];
	const char* errorMsg = nullptr;

#if (_POSIX_C_SOURCE >= 200112L) && !_GNU_SOURCE
	// XSI兼容版本
	strerror_r(errorCode, buffer, sizeof(buffer));
	errorMsg = buffer;
#else
	// GNU特定版本
	errorMsg = strerror_r(errorCode, buffer, sizeof(buffer));
#endif

	if (errorMsg)
	{
		return StringToCString(errorMsg);
	}

	return L"Unknown error";
#endif
}

/** 检查文件是否存在 */
bool CLibToolkit::FileExists(const CString& filePath)
{
#ifdef _WIN32
	return (_waccess(filePath, 0) == 0);
#else
	std::string path = CStringToString(filePath);
	return (access(path.c_str(), F_OK) == 0);
#endif
}

/** 检查目录是否存在 */
bool CLibToolkit::DirectoryExists(const CString& dirPath)
{
#ifdef _WIN32
	DWORD attrib = GetFileAttributesW(dirPath);
	return (attrib != INVALID_FILE_ATTRIBUTES && (attrib & FILE_ATTRIBUTE_DIRECTORY));
#else
	std::string path = CStringToString(dirPath);
	struct stat statbuf;
	if (stat(path.c_str(), &statbuf) != 0)
		return false;
	return S_ISDIR(statbuf.st_mode);
#endif
}

/** 递归创建目录 */
bool CLibToolkit::CreateDirectoryRecursive(const CString& dirPath)
{
	if (dirPath.IsEmpty())
		return false;

	if (DirectoryExists(dirPath))
		return true;

	// 创建父目录
	CString parentDir = GetDirectoryName(dirPath);
	if (!parentDir.IsEmpty() && parentDir != dirPath)
	{
		if (!CreateDirectoryRecursive(parentDir))
			return false;
	}

#ifdef _WIN32
	return (CreateDirectoryW(dirPath, NULL) != 0 || GetLastError() == ERROR_ALREADY_EXISTS);
#else
	std::string path = CStringToString(dirPath);
	return (mkdir(path.c_str(), 0755) == 0 || errno == EEXIST);
#endif
}

/** 获取当前工作目录 */
CString CLibToolkit::GetCurrentDirectory()
{
#ifdef _WIN32
	wchar_t buffer[MAX_PATH];
	DWORD len = ::GetCurrentDirectoryW(MAX_PATH, buffer);
	if (len > 0 && len < MAX_PATH)
	{
		return CString(buffer);
	}
	return L"";
#else
	char buffer[PATH_MAX];
	if (getcwd(buffer, sizeof(buffer)) != NULL)
	{
		return StringToCString(buffer);
	}
	return L"";
#endif
}

/** 设置当前工作目录 */
bool CLibToolkit::SetCurrentDirectory(const CString& dirPath)
{
#ifdef _WIN32
	return (::SetCurrentDirectoryW(dirPath) != 0);
#else
	std::string path = CStringToString(dirPath);
	return (chdir(path.c_str()) == 0);
#endif
}

/** 获取文件名（包含扩展名） */
CString CLibToolkit::GetFileName(const CString& filePath)
{
	if (filePath.IsEmpty())
		return L"";

	int nPos = filePath.ReverseFind(PATH_SEPARATOR);
	if (nPos == -1)
		return filePath;

	return filePath.Mid(nPos + 1);
}

/** 获取文件名（不包含扩展名） */
CString CLibToolkit::GetFileNameWithoutExtension(const CString& filePath)
{
	CString fileName = GetFileName(filePath);
	int nPos = fileName.ReverseFind(L'.');
	if (nPos != -1)
	{
		return fileName.Left(nPos);
	}
	return fileName;
}

/** 获取文件扩展名 */
CString CLibToolkit::GetFileExtension(const CString& filePath)
{
	CString fileName = GetFileName(filePath);
	int nPos = fileName.ReverseFind(L'.');
	if (nPos != -1 && nPos < fileName.GetLength() - 1)
	{
		return fileName.Mid(nPos + 1);
	}
	return L"";
}

/** 获取目录名 */
CString CLibToolkit::GetDirectoryName(const CString& filePath)
{
	if (filePath.IsEmpty())
		return L"";

	int nPos = filePath.ReverseFind(PATH_SEPARATOR);
	if (nPos == -1)
		return L"";

	return filePath.Left(nPos);
}

/** 合并路径 */
CString CLibToolkit::CombinePath(const CString& path1, const CString& path2)
{
	if (path1.IsEmpty())
		return path2;

	if (path2.IsEmpty())
		return path1;

	CString result = path1;
	EnsureTrailingSeparator(result);
	result += path2;

	return result;
}

/** 获取环境变量 */
CString CLibToolkit::GetEnvironmentVariable(const CString& name)
{
	if (name.IsEmpty())
		return L"";

#ifdef _WIN32
	wchar_t buffer[4096];
	DWORD len = ::GetEnvironmentVariableW(name, buffer, sizeof(buffer) / sizeof(wchar_t));
	if (len > 0 && len < sizeof(buffer) / sizeof(wchar_t))
	{
		return CString(buffer);
	}
#else
	std::string nameStr = CStringToString(name);
	const char* value = getenv(nameStr.c_str());
	if (value != nullptr)
	{
		return StringToCString(value);
	}
#endif

	return L"";
}

/** 设置环境变量 */
bool CLibToolkit::SetEnvironmentVariable(const CString& name, const CString& value)
{
	if (name.IsEmpty())
		return false;

#ifdef _WIN32
	return (::SetEnvironmentVariableW(name, value) != 0);
#else
	std::string nameStr = CStringToString(name);
	std::string valueStr = CStringToString(value);
	return (setenv(nameStr.c_str(), valueStr.c_str(), 1) == 0);
#endif
}

/** 获取当前进程ID */
int CLibToolkit::GetCurrentProcessId()
{
#ifdef _WIN32
	return ::GetCurrentProcessId();
#else
	return getpid();
#endif
}

/** 获取当前用户名 */
CString CLibToolkit::GetUserName()
{
#ifdef _WIN32
	wchar_t buffer[256];
	DWORD len = 256;
	if (::GetUserNameW(buffer, &len))
	{
		return CString(buffer);
	}
#else
	struct passwd* pwd = getpwuid(getuid());
	if (pwd != nullptr && pwd->pw_name != nullptr)
	{
		return StringToCString(pwd->pw_name);
	}
#endif

	return L"";
}

/** 获取计算机名 */
CString CLibToolkit::GetComputerName()
{
#ifdef _WIN32
	wchar_t buffer[MAX_COMPUTERNAME_LENGTH + 1];
	DWORD len = MAX_COMPUTERNAME_LENGTH + 1;
	if (::GetComputerNameW(buffer, &len))
	{
		return CString(buffer);
	}
#else
	char buffer[HOST_NAME_MAX + 1];
	if (gethostname(buffer, sizeof(buffer)) == 0)
	{
		return StringToCString(buffer);
	}
#endif

	return L"";
}

/** 获取当前时间字符串 */
CString CLibToolkit::GetCurrentTimeString()
{
	time_t now = time(nullptr);
	struct tm* timeinfo = localtime(&now);

	wchar_t buffer[64];
	wcsftime(buffer, sizeof(buffer) / sizeof(wchar_t), L"%Y-%m-%d %H:%M:%S", timeinfo);

	return CString(buffer);
}

/** 获取当前日期字符串 */
CString CLibToolkit::GetCurrentDateString()
{
	time_t now = time(nullptr);
	struct tm* timeinfo = localtime(&now);

	wchar_t buffer[64];
	wcsftime(buffer, sizeof(buffer) / sizeof(wchar_t), L"%Y-%m-%d", timeinfo);

	return CString(buffer);
}

/** 获取文件最后修改时间 */
long long CLibToolkit::GetFileLastModifiedTime(const CString& filePath)
{
#ifdef _WIN32
	WIN32_FILE_ATTRIBUTE_DATA fileAttr;
	if (GetFileAttributesExW(filePath, GetFileExInfoStandard, &fileAttr))
	{
		ULARGE_INTEGER ui;
		ui.LowPart = fileAttr.ftLastWriteTime.dwLowDateTime;
		ui.HighPart = fileAttr.ftLastWriteTime.dwHighDateTime;
		return ui.QuadPart;
	}
#else
	std::string path = CStringToString(filePath);
	struct stat statbuf;
	if (stat(path.c_str(), &statbuf) == 0)
	{
		return static_cast<long long>(statbuf.st_mtime);
	}
#endif

	return 0;
}

/** 初始化GUID生成器 */
bool CLibToolkit::InitGuidGenerator()
{
	if (!g_bGuidGeneratorInitialized)
	{
#ifdef _WIN32
		// Windows: CoInitialize如果失败也没关系
		CoInitialize(nullptr);
#else
		// Linux: 使用当前时间作为随机数种子
		srand(static_cast<unsigned int>(time(nullptr)));
#endif
		g_bGuidGeneratorInitialized = true;
	}
	return true;
}

/** 生成随机十六进制字符串 */
std::string CLibToolkit::GenerateRandomHex(size_t length)
{
	static const char hex_chars[] = "0123456789ABCDEF";
	std::string result;

	// 使用C++11随机数生成器（如果可用）
#if __cplusplus >= 201103L
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_int_distribution<> dis(0, 15);

	for (size_t i = 0; i < length; ++i)
	{
		result += hex_chars[dis(gen)];
	}
#else
	// 传统方法
	for (size_t i = 0; i < length; ++i)
	{
		result += hex_chars[rand() % 16];
	}
#endif

	return result;
}

/** 确保路径末尾有分隔符 */
void CLibToolkit::EnsureTrailingSeparator(CString& path)
{
	if (path.IsEmpty())
		return;

	if (path[path.GetLength() - 1] != PATH_SEPARATOR)
	{
		path += PATH_SEPARATOR;
	}
}

/** 宽字符串转换为窄字符串 */
std::string CLibToolkit::WStringToString(const std::wstring& wstr)
{
#ifdef _WIN32
	// Windows: 使用WideCharToMultiByte
	int size_needed =
		WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), (int)wstr.size(), NULL, 0, NULL, NULL);
	std::string str(size_needed, 0);
	WideCharToMultiByte(CP_UTF8,
						0,
						wstr.c_str(),
						(int)wstr.size(),
						&str[0],
						size_needed,
						NULL,
						NULL);
	return str;
#else
	// Linux: 使用wcstombs
	size_t size = wcstombs(NULL, wstr.c_str(), 0) + 1;
	if (size == (size_t)-1)
		return "";

	std::string str(size, 0);
	wcstombs(&str[0], wstr.c_str(), size);
	str.resize(size - 1); // 移除终止符

	return str;
#endif
}

/** 窄字符串转换为宽字符串 */
std::wstring CLibToolkit::StringToWString(const std::string& str)
{
#ifdef _WIN32
	// Windows: 使用MultiByteToWideChar
	int size_needed = MultiByteToWideChar(CP_UTF8, 0, str.c_str(), (int)str.size(), NULL, 0);
	std::wstring wstr(size_needed, 0);
	MultiByteToWideChar(CP_UTF8, 0, str.c_str(), (int)str.size(), &wstr[0], size_needed);
	return wstr;
#else
	// Linux: 使用mbstowcs
	size_t size = mbstowcs(NULL, str.c_str(), 0) + 1;
	if (size == (size_t)-1)
		return L"";

	std::wstring wstr(size, 0);
	mbstowcs(&wstr[0], str.c_str(), size);
	wstr.resize(size - 1); // 移除终止符

	return wstr;
#endif
}

/** CString转换为std::string */
std::string CLibToolkit::CStringToString(const CString& cstr)
{
	return WStringToString(std::wstring(cstr));
}

/** std::string转换为CString */
CString CLibToolkit::StringToCString(const std::string& str)
{
	std::wstring wstr = StringToWString(str);
	return CString(wstr.c_str());
}