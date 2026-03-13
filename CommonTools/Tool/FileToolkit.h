#ifndef FILETOOLKIT_H_
#define FILETOOLKIT_H_

#include <include/CommonToolsExport.h>
#include <include/CommonToolsDef.h>
#include <include/StringToolkit.h>

#include <ctime>

// Linux下没有这些Windows类型，需要定义
#ifndef MAX_PATH
#define MAX_PATH 260
#endif


// CSimpleArray 模拟 (如果原项目中使用了MFC的CSimpleArray)
template <class TYPE, class ARG_TYPE = const TYPE&>
class COMMONTOOLS_EXPORT CSimpleArray
{
public:
	TYPE* m_pData;
	int m_nSize;
	int m_nMaxSize;
	int m_nGrowBy;

	CSimpleArray() : m_pData(NULL), m_nSize(0), m_nMaxSize(0), m_nGrowBy(10) {}
	~CSimpleArray()
	{
		if (m_pData)
			delete[] m_pData;
	}

	int GetCount() const { return m_nSize; }
	int GetSize() const { return m_nSize; }
	void SetSize(int nNewSize, int nGrowBy = -1)
	{
		if (nGrowBy != -1)
			m_nGrowBy = nGrowBy;
		if (nNewSize <= m_nMaxSize)
		{
			m_nSize = nNewSize;
			return;
		}
		TYPE* pNewData = new TYPE[nNewSize];
		if (m_pData)
		{
			memcpy(pNewData, m_pData, m_nSize * sizeof(TYPE));
			delete[] m_pData;
		}
		m_pData = pNewData;
		m_nMaxSize = nNewSize;
		m_nSize = nNewSize;
	}
	void RemoveAll() { m_nSize = 0; }
	TYPE GetAt(int nIndex) const { return m_pData[nIndex]; }
	void Add(const TYPE& newElement)
	{
		if (m_nSize >= m_nMaxSize)
		{
			SetSize(m_nSize + m_nGrowBy);
		}
		m_pData[m_nSize++] = newElement;
	}
	TYPE& operator[](int nIndex) { return m_pData[nIndex]; }
};

// 文件属性结构体 (替代WIN32_FIND_DATA)
struct LINUX_FIND_DATA
{
	char cFileName[MAX_PATH];
	wchar_t wcFileName[MAX_PATH];
	unsigned long dwFileAttributes;
	unsigned long nFileSizeLow;
	unsigned long nFileSizeHigh;
	time_t ftCreationTime;
	time_t ftLastAccessTime;
	time_t ftLastWriteTime;
};

// 删除操作标志 (替代FILEOP_FLAGS)
enum
{
	FOF_SILENT = 0x0004,
	FOF_NOCONFIRMATION = 0x0010,
	FOF_NOERRORUI = 0x0400,
	FOF_NOCONFIRMMKDIR = 0x0200
};

// 特殊文件夹 (替代CSIDL)
enum
{
	CSIDL_DESKTOP = 0,
	CSIDL_WINDOWS = 1,
	CSIDL_SYSTEM = 2
};

// Shell API 替代函数声明
bool SHGetSpecialFolderPath(void* hwnd, TCHAR* path, int csidl, bool fCreate);
bool PathFileExists(const TCHAR* path);
bool PathIsDirectory(const TCHAR* path);
int SHCreateDirectoryEx(void* hwnd, const TCHAR* path, void* security);
int SHFileOperation(void* opStruct);


// 文件操作类
class COMMONTOOLS_EXPORT CFileToolkit
{
public:
	/**
	 * 新建目录 （支持多级）
	 */
	static bool CreateDirectory(CString Directory);

	/**
	 * 测试文件是否存在
	 */
	static bool FileExist(CString Filename);

	/**
	 * 测试目录是否存在
	 */
	static bool DirectoryExist(CString Directory);

	/**
	 * 获取特定目录下文件以及目录信息列表
	 */
	static bool ReadDirectory(CString Directory,
							  std::vector<LINUX_FIND_DATA>& arr,
							  CString fileExt = L"*.*");

	/**
	 * 获取特定目录下文件以及目录信息列表
	 */
	static bool ReadDirectory(CString Directory,
							  CSimpleArray<CString>& arr,
							  bool Recursion = false,
							  CString fileExt = L"*.*");

	/**
	 * 取得当前目录
	 */
	static CString GetCurrentDirectory();

	/**
	 * 格式化目录
	 */
	static CString FormatDirectory(CString Directory);

	/**
	 * 格式化文件名
	 */
	static CString FormatFilename(CString Filename, CString ExtendName);

	/**
	 * 取得文件逻辑名
	 */
	static CString GetFileLogicName(CString Filename);

	/**
	 * 取得文件扩展名（包括".")
	 */
	static CString GetFileExtendName(CString Filename);

	/**
	 * 取得文件基本名
	 */
	static CString GetFileBaseName(CString Filename);

	/**
	 * 取得文件所在目录
	 */
	static CString GetFileDirectory(CString Filename);

	/**
	 * 取得文件所在驱动
	 */
	static CString GetDrive(CString Path);

	/**
	 * 设置文件扩展名
	 */
	static CString SetFileExtendName(CString Filename, CString ExtendName);

	/**
	 * 设置文件逻辑名
	 */
	static CString SetFileLogicName(CString Filename, CString LogicName);

	/**
	 * 设置文件基本名
	 */
	static CString SetFileBaseName(CString Filename, CString BaseName);

	/**
	 * 设置文件所在目录
	 */
	static CString SetFileDirectory(CString Filename, CString Directory);

	/**
	 * 删除文件或文件夹
	 * Linux下不支持回收站，直接删除
	 */
	static bool DeleteToRecycle(CString strFile,
								int fFlag = FOF_SILENT | FOF_NOCONFIRMATION | FOF_NOERRORUI |
											FOF_NOCONFIRMMKDIR);

	/**
	 * 根据文件扩展名取得对应的系统图标
	 * Linux下通过xdg-mime查询
	 */
	static bool GetIconByExtName(CString strExt,
								 void*& hIcon,
								 int& nIndex,
								 unsigned long attrib = 0);

	/**
	 * 检查文件名是否为标准文件名
	 */
	static bool CheckFileNameValidate(CString strFile, TCHAR* pChar);

	/**
	 * 根据文件句柄取得文件路径
	 */
	static bool GetFilePathFromHandle(int hFile, TCHAR* lpszPath, unsigned int cchMax);

	/**
	 * 目录拷贝
	 */
	static bool CopyDirectory(const CString& srcDir, const CString& dstDir, void* hwnd);

	/**
	 * 对文件进行异或加密
	 */
	static bool EncryptFile(const CString& srcFile,
							const CString& dstFile,
							int nHeadLen = 128,
							int nKey = 0xC5236911);

	/**
	 * 对文件进行异或解密
	 */
	static bool DeEncryptFile(const CString& srcFile,
							  const CString& dstFile,
							  int nHeadLen = 128,
							  int nKey = 0xC5236911);

	/**
	 * 判断文件是否加密
	 */
	static bool IsFileEncrypt(const CString& strFile);

	/**
	 * 计算相对路径
	 */
	static CString GetRelatePath(const CString& strSrcPath, const CString& strRelateDir);

	/**
	 * 取得临时路径
	 */
	static CString GetTempPath(void);

private:
	/**
	 * 分解出文件名中的所有目录名
	 */
	static int DecDir(CString Filename, CSimpleArray<CString>* arr);

	/**
	 * 根据文件句柄取得卷名
	 */
	static bool GetVolumeNameByHandle(int hFile, TCHAR* szVolumeName, unsigned int cchMax);

	// Linux特有的文件描述符操作
	static std::string GetSymlinkTarget(const std::string& symlink);
	static std::string GetProcFdPath(int fd);
	static bool IsExtFilesystem(int fd);
};

// 辅助函数声明
bool VaildPath(CString strPath);

#endif // FILETOOLKIT_H_