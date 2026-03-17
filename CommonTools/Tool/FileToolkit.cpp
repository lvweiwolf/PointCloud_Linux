
#include <Tool/FileToolkit.h>

#include <unistd.h>
#include <dirent.h>
#include <ftw.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <linux/magic.h>

#include <regex>
#include <fstream>

#ifndef _UNICODE
#define _UNICODE
#endif


// 在 FileToolkit.cpp 中添加辅助函数
static std::string CStringToStdString(const CString& str)
{
	std::string result;

#ifdef _UNICODE
	const wchar_t* wstr = (const wchar_t*)str;
	if (wstr && *wstr)
	{
		size_t size = wcstombs(NULL, wstr, 0);
		if (size != (size_t)-1)
		{
			char* buffer = new char[size + 1];
			wcstombs(buffer, wstr, size);
			buffer[size] = '\0';
			result = buffer;
			delete[] buffer;
		}
	}
#else
	const char* cstr = (const char*)str;
	if (cstr && *cstr)
	{
		result = cstr;
	}
#endif

	return result;
}

// Linux下路径验证辅助函数
bool VaildPath(CString strPath)
{
	try
	{
		strPath.Trim();
		if (strPath.IsEmpty())
			return false;

		// Linux下检查特殊路径
		std::string strPathStd;
#ifdef _UNICODE
		// 宽字符转多字节
		size_t size = wcstombs(NULL, strPath, 0);
		char* buffer = new char[size + 1];
		wcstombs(buffer, strPath, size);
		buffer[size] = '\0';
		strPathStd = buffer;
		delete[] buffer;
#else
		strPathStd = strPath;
#endif

		// 检查是否为根目录
		if (strPathStd == "/" || strPathStd == "//")
			return false;

		// 检查是否为系统重要目录
		if (strPathStd == "/bin" || strPathStd == "/sbin" || strPathStd == "/usr" ||
			strPathStd == "/etc" || strPathStd == "/root" || strPathStd == "/boot" ||
			strPathStd == "/dev" || strPathStd == "/proc" || strPathStd == "/sys" ||
			strPathStd == "/lib" || strPathStd == "/lib64")
			return false;
	}
	catch (...)
	{
		return false;
	}
	return true;
}

int CFileToolkit::DecDir(CString Filename, CSimpleArray<CString>* arr)
{
	int i, Cnt;
	arr->RemoveAll();
	Cnt = CStringToolkit::CountWord(Filename, _T('/'));
	for (i = 1; i <= Cnt; i++)
		arr->Add(CStringToolkit::ReadWord(Filename, i, _T('/')));
	return (int)arr->GetCount();
}

CString CFileToolkit::GetDrive(CString Path)
{
	// Linux下没有Windows的盘符概念
	// 返回路径的根目录部分
	if (Path.GetLength() < 1)
		return _T("");

	// 如果是绝对路径，返回 "/"
	if (Path.GetAt(0) == _T('/'))
		return _T("/");

	// 相对路径返回空
	return _T("");
}

CString CFileToolkit::FormatDirectory(CString Directory)
{
	CSimpleArray<CString> list;
	CString RetDir, SubDir;

	if (Directory.IsEmpty())
		return _T("");

	DecDir(Directory, &list);
	if (list.GetCount() == 0)
		return _T("");

	// Linux使用 '/' 而不是 '\'
	// 处理相对路径和绝对路径
	if (Directory.GetAt(0) == _T('/'))
	{
		// 绝对路径
		RetDir = _T("/");
	}
	else
	{
		// 相对路径，从当前目录开始
		RetDir = GetCurrentDirectory();
	}

	for (int i = 0; i < (int)list.GetCount(); i++)
	{
		SubDir = list.GetAt(i);

		if (SubDir == _T(".."))
		{
			// 上级目录
			int pos = RetDir.ReverseFind(_T('/'));
			if (pos > 0)
			{
				RetDir = RetDir.Left(pos);
			}
		}
		else if (SubDir == _T("."))
		{
			// 当前目录，不做处理
			continue;
		}
		else if (!SubDir.IsEmpty())
		{
			// 正常目录名
			if (!RetDir.IsEmpty() && RetDir.GetAt(RetDir.GetLength() - 1) != _T('/'))
			{
				RetDir += _T("/");
			}
			RetDir += SubDir;
		}
	}

	// 确保以 '/' 结尾
	if (!RetDir.IsEmpty() && RetDir.GetAt(RetDir.GetLength() - 1) != _T('/'))
	{
		RetDir += _T("/");
	}

	return RetDir;
}

CString CFileToolkit::FormatFilename(CString Filename, CString ExtendName)
{
	CString name = FormatDirectory(Filename);
	if (name.IsEmpty())
		return _T("");

	// 移除末尾的 '/'
	if (name.GetAt(name.GetLength() - 1) == _T('/'))
	{
		name = name.Left(name.GetLength() - 1);
	}

	// 添加扩展名
	if (GetFileExtendName(name).IsEmpty() && !ExtendName.IsEmpty())
	{
		name += _T(".") + ExtendName;
	}

	return name;
}

CString CFileToolkit::GetFileLogicName(CString Filename)
{
	int EndPos = CStringToolkit::FindLastChar(Filename, _T('/'));
	if (EndPos >= 0)
		return Filename.Mid(EndPos + 1);
	else
		return Filename;
}

CString CFileToolkit::GetFileExtendName(CString Filename)
{
	CString LogicName = GetFileLogicName(Filename);
	int i = CStringToolkit::FindLastChar(LogicName, _T('.'));
	int j = CStringToolkit::FindLastChar(LogicName, _T('/'));

	if (i < j)
		return _T("");
	else if (i >= 0)
		return LogicName.Mid(i);
	else
		return _T("");
}

CString CFileToolkit::GetFileBaseName(CString Filename)
{
	CString LogName = GetFileLogicName(Filename);
	int i = CStringToolkit::FindLastChar(LogName, _T('.'));

	if (i >= 0)
	{
		LogName = LogName.Left(i);
	}

	return LogName.Trim();
}

CString CFileToolkit::GetFileDirectory(CString Filename)
{
	int i = CStringToolkit::FindLastChar(Filename, _T('/'));
	if (i >= 0)
		return Filename.Left(i + 1);
	else
		return _T("");
}

CString CFileToolkit::SetFileExtendName(CString Filename, CString ExtendName)
{
	CString baseName = GetFileBaseName(Filename);
	if (baseName.IsEmpty())
		return _T("");

	ExtendName.TrimW(_T('.'));
	if (!ExtendName.IsEmpty())
		ExtendName = _T(".") + ExtendName;

	CString ExtName = GetFileExtendName(Filename);
	CString RetName;
	if (ExtName.IsEmpty())
		RetName = Filename + ExtendName;
	else
		RetName = Filename.Left(Filename.GetLength() - ExtName.GetLength()) + ExtendName;

	return RetName;
}

CString CFileToolkit::SetFileLogicName(CString Filename, CString LogicName)
{
	return GetFileDirectory(Filename) + LogicName;
}

CString CFileToolkit::SetFileBaseName(CString Filename, CString BaseName)
{
	if (BaseName.IsEmpty())
		return _T("");

	CString Ext = GetFileExtendName(Filename);
	if (!Ext.IsEmpty())
		return GetFileDirectory(Filename) + BaseName + Ext;
	else
		return GetFileDirectory(Filename) + BaseName;
}

CString CFileToolkit::SetFileDirectory(CString Filename, CString Directory)
{
	if (!Directory.IsEmpty() && Directory.GetAt(Directory.GetLength() - 1) != _T('/'))
		return Directory + _T("/") + GetFileLogicName(Filename);
	else
		return Directory + GetFileLogicName(Filename);
}

bool CFileToolkit::CreateDirectory(CString Directory)
{
	// 转换CString到std::string
	std::string dir;
#ifdef _UNICODE
	size_t size = wcstombs(NULL, Directory, 0);
	char* buffer = new char[size + 1];
	wcstombs(buffer, Directory, size);
	buffer[size] = '\0';
	dir = buffer;
	delete[] buffer;
#else
	dir = CStringToolkit::convertUTF16toUTF8(Directory);
#endif

	// 递归创建目录
	mode_t mode = 0755; // rwxr-xr-x
	size_t pre = 0, pos;
	std::string subdir;
	int mdret;

	if (dir[dir.size() - 1] != '/')
	{
		dir += '/';
	}

	while ((pos = dir.find_first_of('/', pre)) != std::string::npos)
	{
		subdir = dir.substr(0, pos++);
		pre = pos;
		if (subdir.empty())
			continue; // 开头的斜杠

		if ((mdret = mkdir(subdir.c_str(), mode)) && errno != EEXIST)
		{
			return false;
		}
	}

	return true;
}



bool CFileToolkit::FileExist(CString Filename)
{
	std::string filePath = CStringToStdString(Filename);
	if (filePath.empty())
		return false;

	struct stat fileStat;
	return (stat(filePath.c_str(), &fileStat) == 0);
}

bool CFileToolkit::DirectoryExist(CString Directory)
{
	std::string dir;
#ifdef _UNICODE
	// 宽字符版本
	size_t size = wcstombs(NULL, (const wchar_t*)Directory, 0);
	if (size == (size_t)-1)
		return false; // 转换失败

	char* buffer = new char[size + 1];
	wcstombs(buffer, (const wchar_t*)Directory, size);
	buffer[size] = '\0';
	dir = buffer;
	delete[] buffer;
#else
	// 窄字符版本 - 直接转换
	dir = (const char*)Directory;
#endif

	struct stat fileStat; // 使用不同的变量名，避免与buffer冲突
	if (stat(dir.c_str(), &fileStat) != 0)
		return false;

	return S_ISDIR(fileStat.st_mode);
}

CString CFileToolkit::GetCurrentDirectory()
{
	char buffer[MAX_PATH];
	if (getcwd(buffer, MAX_PATH) != NULL)
	{
		CString dir;
#ifdef _UNICODE
		size_t len = strlen(buffer);
		wchar_t* wbuffer = new wchar_t[len + 1];
		mbstowcs(wbuffer, buffer, len);
		wbuffer[len] = L'\0';
		dir = wbuffer;
		delete[] wbuffer;
#else
		dir = buffer;
#endif

		if (!dir.IsEmpty() && dir.GetAt(dir.GetLength() - 1) != _T('/'))
			dir += _T("/");
		return dir;
	}
	return _T("");
}

bool CFileToolkit::ReadDirectory(CString Directory,
								 std::vector<LINUX_FIND_DATA>& arr,
								 CString fileExt)
{
	std::string dir;
#ifdef _UNICODE
	size_t size = wcstombs(NULL, Directory, 0);
	char* buffer = new char[size + 1];
	wcstombs(buffer, Directory, size);
	buffer[size] = '\0';
	dir = buffer;
	delete[] buffer;
#else
	dir = Directory;
#endif

	// 确保目录以 '/' 结尾
	if (!dir.empty() && dir[dir.length() - 1] != '/')
		dir += '/';

	arr.clear();

	DIR* dp = opendir(dir.c_str());
	if (dp == NULL)
		return false;

	std::string pattern;
#ifdef _UNICODE
	size = wcstombs(NULL, fileExt, 0);
	char* patternBuffer = new char[size + 1];
	wcstombs(patternBuffer, fileExt, size);
	patternBuffer[size] = '\0';
	pattern = patternBuffer;
	delete[] patternBuffer;
#else
	pattern = fileExt;
#endif

	// 转换通配符为正则表达式
	std::string regexPattern = "^";
	for (char c : pattern)
	{
		if (c == '*')
			regexPattern += ".*";
		else if (c == '?')
			regexPattern += ".";
		else if (c == '.' || c == '(' || c == ')' || c == '[' || c == ']' || c == '{' || c == '}' ||
				 c == '\\' || c == '|' || c == '+' || c == '^' || c == '$')
			regexPattern += "\\" + std::string(1, c);
		else
			regexPattern += c;
	}
	regexPattern += "$";

	std::regex fileRegex(regexPattern, std::regex::icase);

	struct dirent* entry;
	while ((entry = readdir(dp)) != NULL)
	{
		if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
			continue;

		// 检查文件扩展名匹配
		std::string filename = entry->d_name;
		if (pattern != "*.*" && !std::regex_search(filename, fileRegex))
			continue;

		std::string fullpath = dir + entry->d_name;
		struct stat fileStat;
		if (stat(fullpath.c_str(), &fileStat) == 0)
		{
			LINUX_FIND_DATA findData;
			memset(&findData, 0, sizeof(LINUX_FIND_DATA));

			// 复制文件名
			strncpy(findData.cFileName, entry->d_name, MAX_PATH - 1);

#ifdef _UNICODE
			// 转换为宽字符
			mbstowcs(findData.wcFileName, entry->d_name, MAX_PATH - 1);
#endif

			// 设置文件属性
			if (S_ISDIR(fileStat.st_mode))
				findData.dwFileAttributes = 0x10; // FILE_ATTRIBUTE_DIRECTORY
			else
				findData.dwFileAttributes = 0x80; // FILE_ATTRIBUTE_NORMAL

			findData.nFileSizeLow = fileStat.st_size & 0xFFFFFFFF;
			findData.nFileSizeHigh = (fileStat.st_size >> 32) & 0xFFFFFFFF;
			findData.ftCreationTime = fileStat.st_ctime;
			findData.ftLastAccessTime = fileStat.st_atime;
			findData.ftLastWriteTime = fileStat.st_mtime;

			arr.push_back(findData);
		}
	}

	closedir(dp);
	return true;
}

bool CFileToolkit::ReadDirectory(CString Directory,
								 CSimpleArray<CString>& arr,
								 bool Recursion,
								 CString fileExt)
{
	std::string dir;
#ifdef _UNICODE
	size_t size = wcstombs(NULL, Directory, 0);
	char* buffer = new char[size + 1];
	wcstombs(buffer, Directory, size);
	buffer[size] = '\0';
	dir = buffer;
	delete[] buffer;
#else
	dir = Directory;
#endif

	// 递归读取目录的函数
	std::function<void(const std::string&)> readDirRecursive = [&](const std::string& currentDir) {
		DIR* dp = opendir(currentDir.c_str());
		if (dp == NULL)
			return;

		struct dirent* entry;
		while ((entry = readdir(dp)) != NULL)
		{
			if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
				continue;

			std::string fullpath = currentDir + "/" + entry->d_name;
			struct stat fileStat;
			if (stat(fullpath.c_str(), &fileStat) == 0)
			{
				if (S_ISDIR(fileStat.st_mode))
				{
					if (Recursion)
					{
						readDirRecursive(fullpath);
					}
				}
				else
				{
					// 检查文件扩展名
					std::string filename = entry->d_name;
					std::string ext = filename.substr(filename.find_last_of("."));

					if (fileExt == L"*.*" || (!ext.empty() && ext == std::string(".txt")))
					{ // 这里需要根据实际扩展名检查
						CString cstrFullpath;
#ifdef _UNICODE
						size_t len = fullpath.length();
						wchar_t* wbuffer = new wchar_t[len + 1];
						mbstowcs(wbuffer, fullpath.c_str(), len);
						wbuffer[len] = L'\0';
						cstrFullpath = wbuffer;
						delete[] wbuffer;
#else
						cstrFullpath = fullpath.c_str();
#endif
						arr.Add(cstrFullpath);
					}
				}
			}
		}

		closedir(dp);
	};

	arr.RemoveAll();
	readDirRecursive(dir);
	return true;
}

// Linux下删除文件（不支持回收站）
bool CFileToolkit::DeleteToRecycle(CString strFile, int fFlag)
{
	if (strFile.IsEmpty())
		return false;

	if (!VaildPath(strFile))
		return false;

	std::string filename;
#ifdef _UNICODE
	size_t size = wcstombs(NULL, strFile, 0);
	char* buffer = new char[size + 1];
	wcstombs(buffer, strFile, size);
	buffer[size] = '\0';
	filename = buffer;
	delete[] buffer;
#else
	filename = strFile;
#endif

	// 判断是文件还是目录
	struct stat fileStat;
	if (stat(filename.c_str(), &fileStat) != 0)
		return false;

	if (S_ISDIR(fileStat.st_mode))
	{
		// 递归删除目录
		auto deleteDir =
			[](const char* fpath, const struct stat* sb, int typeflag, struct FTW* ftwbuf) -> int {
			int ret = remove(fpath);
			if (ret)
				perror(fpath);
			return ret;
		};

		return nftw(filename.c_str(), deleteDir, 64, FTW_DEPTH | FTW_PHYS) == 0;
	}
	else
	{
		// 删除文件
		return remove(filename.c_str()) == 0;
	}
}

// Linux下获取文件图标（通过xdg-mime）
bool CFileToolkit::GetIconByExtName(CString strExt, void*& hIcon, int& nIndex, unsigned long attrib)
{
	hIcon = NULL;
	nIndex = 0;

	// Linux下通常通过xdg-mime查询文件类型关联的图标
	// 这里简化处理，返回NULL
	return false;
}

// 检查文件名是否有效
bool CFileToolkit::CheckFileNameValidate(CString strFile, TCHAR* pChar)
{
	// Linux文件名不能包含的字符
	TCHAR InvalidChar[] = { _T('/'), _T('\0'), _T('\n'), _T('\r'), _T('\t'), _T('\v'), _T('\f') };

	// 检查是否包含无效字符
	int nCount = sizeof(InvalidChar) / sizeof(TCHAR);
	for (int i = 0; i < nCount; ++i)
	{
		if (strFile.Find(InvalidChar[i]) != -1)
		{
			if (pChar != NULL)
				*pChar = InvalidChar[i];
			return false;
		}
	}

	// 检查保留文件名
	CString baseName = GetFileBaseName(strFile);
	if (baseName == _T(".") || baseName == _T(".."))
		return false;

	// Linux下文件名长度限制
	if (strFile.GetLength() > NAME_MAX)
		return false;

	return true;
}

// 获取符号链接目标
std::string CFileToolkit::GetSymlinkTarget(const std::string& symlink)
{
	char buffer[MAX_PATH];
	ssize_t len = readlink(symlink.c_str(), buffer, sizeof(buffer) - 1);
	if (len != -1)
	{
		buffer[len] = '\0';
		return buffer;
	}
	return "";
}

// 获取/proc/[pid]/fd/[fd]路径
std::string CFileToolkit::GetProcFdPath(int fd)
{
	char path[PATH_MAX];
	snprintf(path, sizeof(path), "/proc/self/fd/%d", fd);
	return GetSymlinkTarget(path);
}

// 检查是否为ext文件系统
bool CFileToolkit::IsExtFilesystem(int fd)
{
	struct statfs fs;
	if (fstatfs(fd, &fs) == 0)
	{
		return fs.f_type == EXT4_SUPER_MAGIC || fs.f_type == EXT3_SUPER_MAGIC ||
			   fs.f_type == EXT2_SUPER_MAGIC;
	}
	return false;
}

// 根据文件描述符获取文件路径（Linux实现）
bool CFileToolkit::GetFilePathFromHandle(int hFile, TCHAR* lpszPath, unsigned int cchMax)
{
	if (hFile < 0)
		return false;

	std::string procPath = GetProcFdPath(hFile);
	if (procPath.empty())
		return false;

#ifdef _UNICODE
	size_t len = procPath.length();
	if (len >= cchMax)
		len = cchMax - 1;
	mbstowcs(lpszPath, procPath.c_str(), len);
	lpszPath[len] = L'\0';
#else
	strncpy(lpszPath, procPath.c_str(), cchMax - 1);
	lpszPath[cchMax - 1] = '\0';
#endif

	return true;
}

// 目录拷贝（Linux实现）
bool CFileToolkit::CopyDirectory(const CString& srcDir, const CString& dstDir, void* hwnd)
{
	std::string src, dst;

#ifdef _UNICODE
	size_t size = wcstombs(NULL, srcDir, 0);
	char* srcBuffer = new char[size + 1];
	wcstombs(srcBuffer, srcDir, size);
	srcBuffer[size] = '\0';
	src = srcBuffer;
	delete[] srcBuffer;

	size = wcstombs(NULL, dstDir, 0);
	char* dstBuffer = new char[size + 1];
	wcstombs(dstBuffer, dstDir, size);
	dstBuffer[size] = '\0';
	dst = dstBuffer;
	delete[] dstBuffer;
#else
	src = srcDir;
	dst = dstDir;
#endif

	// 确保目标目录存在
	if (!DirectoryExist(dstDir))
	{
		if (!CreateDirectory(dstDir))
			return false;
	}

	// 使用rsync或cp命令（这里使用系统调用）
	std::string command = "cp -r \"" + src + "\" \"" + dst + "\" 2>/dev/null";
	return system(command.c_str()) == 0;
}

// 文件加密
bool CFileToolkit::EncryptFile(const CString& srcFile,
							   const CString& dstFile,
							   int nHeadLen,
							   int nKey)
{
	std::string src, dst;

#ifdef _UNICODE
	size_t size = wcstombs(NULL, srcFile, 0);
	char* srcBuffer = new char[size + 1];
	wcstombs(srcBuffer, srcFile, size);
	srcBuffer[size] = '\0';
	src = srcBuffer;
	delete[] srcBuffer;

	size = wcstombs(NULL, dstFile, 0);
	char* dstBuffer = new char[size + 1];
	wcstombs(dstBuffer, dstFile, size);
	dstBuffer[size] = '\0';
	dst = dstBuffer;
	delete[] dstBuffer;
#else
	src = srcFile;
	dst = dstFile;
#endif

	// 打开源文件
	std::ifstream in(src, std::ios::binary);
	if (!in)
		return false;

	// 获取文件大小
	in.seekg(0, std::ios::end);
	size_t fileSize = in.tellg();
	in.seekg(0, std::ios::beg);

	if (fileSize <= (size_t)nHeadLen)
		return false;

	// 读取文件数据
	char* pDataBuffer = new char[fileSize];
	in.read(pDataBuffer, fileSize);
	in.close();

	// 打开目标文件
	std::ofstream out(dst, std::ios::binary);
	if (!out)
	{
		delete[] pDataBuffer;
		return false;
	}

	// 加密文件头
	char* lpKey = (char*)&nKey;
	for (size_t i = 0; i < (size_t)nHeadLen; ++i)
	{
		pDataBuffer[i] ^= lpKey[i % 4];
	}

	// 加密文件主体
	nKey = (nKey & 0xB3641259);
	for (size_t i = 0; i < fileSize; ++i)
	{
		pDataBuffer[i] ^= lpKey[i % 4];
	}

	// 写入加密数据
	out.write(pDataBuffer, fileSize);

	// 写入加密标识
	const char* tagChar = "PWSJFILE";
	out.write(tagChar, strlen(tagChar));

	delete[] pDataBuffer;
	out.close();

	return true;
}

// 文件解密
bool CFileToolkit::DeEncryptFile(const CString& srcFile,
								 const CString& dstFile,
								 int nHeadLen,
								 int nKey)
{
	std::string src, dst;

#ifdef _UNICODE
	size_t size = wcstombs(NULL, srcFile, 0);
	char* srcBuffer = new char[size + 1];
	wcstombs(srcBuffer, srcFile, size);
	srcBuffer[size] = '\0';
	src = srcBuffer;
	delete[] srcBuffer;

	size = wcstombs(NULL, dstFile, 0);
	char* dstBuffer = new char[size + 1];
	wcstombs(dstBuffer, dstFile, size);
	dstBuffer[size] = '\0';
	dst = dstBuffer;
	delete[] dstBuffer;
#else
	src = srcFile;
	dst = dstFile;
#endif

	// 检查文件是否加密
	if (!IsFileEncrypt(srcFile))
		return false;

	// 打开源文件
	std::ifstream in(src, std::ios::binary);
	if (!in)
		return false;

	// 获取文件大小（减去标识长度）
	in.seekg(0, std::ios::end);
	size_t fileSize = in.tellg();
	const char* tagChar = "PWSJFILE";
	size_t tagLen = strlen(tagChar);
	fileSize -= tagLen;

	if (fileSize <= (size_t)nHeadLen)
		return false;

	// 读取文件数据
	char* pDataBuffer = new char[fileSize];
	in.seekg(0, std::ios::beg);
	in.read(pDataBuffer, fileSize);
	in.close();

	// 打开目标文件
	std::ofstream out(dst, std::ios::binary);
	if (!out)
	{
		delete[] pDataBuffer;
		return false;
	}

	// 解密文件头
	char* lpKey = (char*)&nKey;
	for (size_t i = 0; i < (size_t)nHeadLen; ++i)
	{
		pDataBuffer[i] ^= lpKey[i % 4];
	}

	// 解密文件主体
	nKey = (nKey & 0xB3641259);
	for (size_t i = 0; i < fileSize; ++i)
	{
		pDataBuffer[i] ^= lpKey[i % 4];
	}

	// 写入解密数据
	out.write(pDataBuffer, fileSize);

	delete[] pDataBuffer;
	out.close();

	return true;
}

// 判断文件是否加密
bool CFileToolkit::IsFileEncrypt(const CString& strFile)
{
	std::string filename;

#ifdef _UNICODE
	size_t size = wcstombs(NULL, strFile, 0);
	char* buffer = new char[size + 1];
	wcstombs(buffer, strFile, size);
	buffer[size] = '\0';
	filename = buffer;
	delete[] buffer;
#else
	filename = strFile;
#endif

	// 打开文件
	std::ifstream in(filename, std::ios::binary);
	if (!in)
		return false;

	// 检查文件大小
	in.seekg(0, std::ios::end);
	size_t fileSize = in.tellg();
	const char* tagChar = "PWSJFILE";
	size_t tagLen = strlen(tagChar);

	if (fileSize <= tagLen)
		return false;

	// 读取文件末尾的标识
	char* pReadTagChar = new char[tagLen];
	in.seekg(-tagLen, std::ios::end);
	in.read(pReadTagChar, tagLen);
	in.close();

	// 比较标识
	bool bSame = (memcmp(tagChar, pReadTagChar, tagLen) == 0);
	delete[] pReadTagChar;

	return bSame;
}

// 计算相对路径
CString CFileToolkit::GetRelatePath(const CString& strSrcPath, const CString& strRelateDir)
{
	std::string src, dir;

#ifdef _UNICODE
	size_t size = wcstombs(NULL, strSrcPath, 0);
	char* srcBuffer = new char[size + 1];
	wcstombs(srcBuffer, strSrcPath, size);
	srcBuffer[size] = '\0';
	src = srcBuffer;
	delete[] srcBuffer;

	size = wcstombs(NULL, strRelateDir, 0);
	char* dirBuffer = new char[size + 1];
	wcstombs(dirBuffer, strRelateDir, size);
	dirBuffer[size] = '\0';
	dir = dirBuffer;
	delete[] dirBuffer;
#else
	src = strSrcPath;
	dir = strRelateDir;
#endif

	// 简化路径
	char srcCanonical[PATH_MAX], dirCanonical[PATH_MAX];
	if (realpath(src.c_str(), srcCanonical) == NULL || realpath(dir.c_str(), dirCanonical) == NULL)
	{
		return strSrcPath;
	}

	// 计算相对路径
	std::string srcStr = srcCanonical;
	std::string dirStr = dirCanonical;

	// 确保目录以 '/' 结尾
	if (!dirStr.empty() && dirStr[dirStr.length() - 1] != '/')
		dirStr += '/';

	// 查找共同前缀
	size_t commonPrefix = 0;
	size_t minLen = std::min(srcStr.length(), dirStr.length());
	while (commonPrefix < minLen && srcStr[commonPrefix] == dirStr[commonPrefix])
		commonPrefix++;

	// 如果没有共同前缀，返回绝对路径
	if (commonPrefix == 0)
	{
		CString result;
#ifdef _UNICODE
		size_t len = srcStr.length();
		wchar_t* wbuffer = new wchar_t[len + 1];
		mbstowcs(wbuffer, srcStr.c_str(), len);
		wbuffer[len] = L'\0';
		result = wbuffer;
		delete[] wbuffer;
#else
		result = srcStr.c_str();
#endif
		return result;
	}

	// 构建相对路径
	std::string relativePath;

	// 回到共同祖先目录
	size_t pos = dirStr.find('/', commonPrefix);
	while (pos != std::string::npos)
	{
		relativePath += "../";
		pos = dirStr.find('/', pos + 1);
	}

	// 添加剩余部分
	if (commonPrefix < srcStr.length())
		relativePath += srcStr.substr(commonPrefix);

	CString result;
#ifdef _UNICODE
	size_t len = relativePath.length();
	wchar_t* wbuffer = new wchar_t[len + 1];
	mbstowcs(wbuffer, relativePath.c_str(), len);
	wbuffer[len] = L'\0';
	result = wbuffer;
	delete[] wbuffer;
#else
	result = relativePath.c_str();
#endif

	return result;
}

// 获取临时目录
CString CFileToolkit::GetTempPath(void)
{
	const char* tmpDir = getenv("TMPDIR");
	if (tmpDir == NULL)
		tmpDir = "/tmp";

	CString result;
#ifdef _UNICODE
	size_t len = strlen(tmpDir);
	wchar_t* wbuffer = new wchar_t[len + 1];
	mbstowcs(wbuffer, tmpDir, len);
	wbuffer[len] = L'\0';
	result = wbuffer;
	delete[] wbuffer;
#else
	result = tmpDir;
#endif

	if (!result.IsEmpty() && result.GetAt(result.GetLength() - 1) != _T('/'))
		result += _T("/");

	return result;
}

// Shell API 替代函数实现
bool SHGetSpecialFolderPath(void* hwnd, TCHAR* path, int csidl, bool fCreate)
{
	const char* folder = NULL;

	switch (csidl)
	{
	case CSIDL_DESKTOP:
		folder = getenv("HOME");
		if (folder)
		{
			std::string desktop = std::string(folder) + "/Desktop";
#ifdef _UNICODE
			mbstowcs(path, desktop.c_str(), desktop.length());
#else
			strcpy(path, desktop.c_str());
#endif
			return true;
		}
		break;

	case CSIDL_WINDOWS:
		// Linux下没有Windows目录，返回根目录
#ifdef _UNICODE
		wcscpy(path, L"/");
#else
		strcpy(path, "/");
#endif
		return true;

	case CSIDL_SYSTEM:
		// 返回/bin目录
#ifdef _UNICODE
		wcscpy(path, L"/bin");
#else
		strcpy(path, "/bin");
#endif
		return true;
	}

	return false;
}

bool PathFileExists(const TCHAR* path)
{
	std::string strPath = CStringToStdString(path);
	struct stat buffer;
	return (stat(strPath.c_str(), &buffer) == 0);
}

bool PathIsDirectory(const TCHAR* path)
{
	std::string strPath = CStringToStdString(path);
	struct stat buffer;
	if (stat(strPath.c_str(), &buffer) != 0)
		return false;

	return S_ISDIR(buffer.st_mode);
}

int SHCreateDirectoryEx(void* hwnd, const TCHAR* path, void* security)
{
	std::string dir;
#ifdef _UNICODE
	size_t size = wcslen(path);
	char* buffer = new char[size + 1];
	wcstombs(buffer, path, size);
	buffer[size] = '\0';
	dir = buffer;
	delete[] buffer;
#else
	dir = path;
#endif

	return CFileToolkit::CreateDirectory(path) ? 0 : -1;
}

// 简化版的SHFileOperation结构
struct SHFILEOPSTRUCT
{
	void* hwnd;
	unsigned int wFunc;
	const TCHAR* pFrom;
	const TCHAR* pTo;
	unsigned int fFlags;
	bool fAnyOperationsAborted;
};

int SHFileOperation(void* opStruct)
{
	SHFILEOPSTRUCT* op = (SHFILEOPSTRUCT*)opStruct;

	if (op->wFunc == 2)
	{ // FO_DELETE
		return CFileToolkit::DeleteToRecycle(op->pFrom, op->fFlags) ? 0 : 1;
	}
	else if (op->wFunc == 3)
	{ // FO_COPY
		return CFileToolkit::CopyDirectory(op->pFrom, op->pTo, op->hwnd) ? 0 : 1;
	}

	return 1;
}