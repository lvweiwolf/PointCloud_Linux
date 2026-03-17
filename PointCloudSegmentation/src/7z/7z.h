#ifndef ZIP7Z_H_
#define ZIP7Z_H_

#include <string>
#include <vector>
#include <map>
#include <functional>
#include <bit7z/bit7z.hpp>

// 解压数据格式
typedef std::vector<unsigned char> CharBuf;
typedef std::map<std::wstring, CharBuf> DepData;
typedef DepData::iterator DepDataIter;

namespace toolkit {
	/** @addtogroup Toolkit
	 * @{
	 */
	using ProgressCallbackFunc = std::function<bool(uint64_t)>;
	using CancelCallbackFunc = std::function<bool()>;

	enum CompressFormat
	{
		eFormatNull,
		eSevenZip,
		eZip,
		eRar,
	};

	//! 7Z压缩文件解析
	class C7z
	{
	public:
		/**
		 *  函数介绍:    判断是否是rar格式文件
		 *  输入参数:	const std::wstring & strFile
		 *  返回值:   	bool
		 */
		static bool IsRarFile(const std::wstring& strFile);

		/**
		 *  函数介绍:    判断是否是zip格式文件
		 *  输入参数:	const std::wstring & strFile
		 *  返回值:   	bool
		 */
		static bool IsZipFile(const std::wstring& strFile);

		/**
		 *  函数介绍:    判断是否是7z格式文件
		 *  输入参数:	const std::wstring & strFile
		 *  返回值:   	bool
		 */
		static bool IsSevenZipFile(const std::wstring& strFile);

		/**
		 *  函数介绍:    根据枚举获取文件格式
		 *  输入参数:	const CompressFormat & format
		 *  返回值:   	const bit7z::BitInFormat &
		 */
		static const bit7z::BitInFormat& GetBitFormat(const CompressFormat& format);

		static int DecompressTypeFile(const std::wstring& strFileLoadPath,
									  const std::wstring& strPath,
									  const CompressFormat format,
									  const std::vector<std::wstring>& vecFilter = {},
									  const std::wstring& strPassword = L"",
									  ProgressCallbackFunc progressCallback = nullptr);

		/**
		 * 压缩文件
		 * @param [in] strFolder	要压缩的文件夹
		 * @param [in] strFileSavePath 压缩后的文件保存的全路径
		 * @param [in] strPassword 压缩后的文件密码
		 * @param [in] progressCallback 压缩进度回调函数指针
		 * @return int 0为压缩成功，其他为压缩失败
		 */
		static int compress(const std::wstring& strFolder,
							const std::wstring& strFileSavePath,
							const std::wstring& strPassword = L"",
							ProgressCallbackFunc progressCallback = nullptr);

		/**
		 * 解压至文件
		 * @param [in] strFileLoadPath 要解压的压缩包
		 * @param [in] strPath 解压后保存的路径
		 * @param [in] vecFilter	需要解压的文件包内路径（为空则全部解压）
		 * @param [in] strPassword 压缩包的文件密码
		 * @param [in] progressCallback 解压进度回调函数指针
		 * @return int 0为解压成功，其他为解压失败
		 */
		static int decompress(const std::wstring& strFileLoadPath,
							  const std::wstring& strPath,
							  const std::vector<std::wstring>& vecFilter = {},
							  const std::wstring& strPassword = L"",
							  ProgressCallbackFunc progressCallback = nullptr);

		/**
		 * 解压至内存
		 * @param [in] strFileLoadPath 要解压的压缩包
		 * @param [out] mapFileNameAndInfo 压缩包内文件路径和文件内容键值对
		 * @return int 0为解压成功，其他为解压失败
		 * @return	-1	无法加载7z.dll文件
		 * @return	-2	无法解压压缩包
		 */
		static int DecompressInfo(
			const std::wstring& strFileLoadPath,
			std::map<std::wstring, std::vector<unsigned char>>& mapFileNameAndInfo);

		/**
		 *	@brief	通过压缩包内文件路径解压到内存并返回其文件内容
		 *
		 *	@param	[in]	strFileLoadPath		要解压的压缩包
		 *	@param	[in]	vecFilePath			要解压的文件在压缩包内的路径
		 *	@param	[out]	mapFileNameAndInfo	压缩包内文件路径和文件内容键值对
		 *
		 *	@return	0	成功
		 *	@return	-1	压缩包文件路径为空
		 *	@return	-2	vecFilePath为空
		 */
		static int DecompressInfoByPath(
			const std::wstring& strFileLoadPath,
			const std::vector<std::wstring>& vecFilePath,
			std::map<std::wstring, std::vector<unsigned char>>& mapFileNameAndInfo);

		/**
		 * 压缩文件
		 * @param [in] fileMap 文件map
		 * @param [in] strOutputPath 压缩包输出路径
		 * @return int 0为成功，-1为失败
		 */
		static int Compress(std::map<std::wstring, std::vector<unsigned char>>& fileMap,
							std::wstring strOutputPath);

		/**
		 *  @brief    获得7Z文件信息(需要外部手动释放)
		 *
		 *  @param    const std::wstring & strGimPath
		 *  @param    bit7z::BitArchiveInfo *& pInfo
		 *  @return   int
		 */
		static int GetBitArchiveInfo(const std::wstring& strGimPath,
									 std::unique_ptr<bit7z::BitArchiveReader> pInfo);

		/**
		 *  @brief    解压文件（可通过回调取消解压）
		 *
		 *  @param    const std::wstring & strFileLoadPath
		 *  @param    const std::wstring & strPath
		 *  @param    const std::wstring & strPassword
		 *  @param    CancelCallbackFunc cancleCallback
		 *  @return   int
		 */
		static int Decompress(const std::wstring& strFileLoadPath,
							  const std::wstring& strPath,
							  const std::wstring& strPassword /*= L""*/,
							  CancelCallbackFunc cancleCallback /*= nullptr*/);

		/**
		 * 添加一个现有文件到压缩文件中
		 * @param [in] strFilePath	文件路径（相对、绝对均可）
		 * @param [in] strPressPath	压缩包路径
		 * @return int 0为成功，-1为失败
		 */
		static int CompressFileToPackage(const std::wstring& strFilePath,
										 const std::wstring& strPressPath);

		/**
		 * 添加一整个文件夹到压缩文件中
		 * @param [in] strFolderPath	文件夹路径（相对、绝对均可）
		 * @param [in] strPressPath	压缩包路径
		 * @return int 0为成功，-1为失败
		 */
		static int CompressFolderToPackage(const std::wstring& strFolderPath,
										   const std::wstring& strPressPath);

	protected:
		/**
		 * string转wstring
		 * @param [in] str string字符串
		 * @return std::wstring wstring字符串
		 */
		static std::wstring str2wstr(const std::string& str);
	};
	/** @} */
}

#endif // ZIP7Z_H_