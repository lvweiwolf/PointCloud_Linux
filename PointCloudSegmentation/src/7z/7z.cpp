#include <src/7z/7z.h>

// #include <bit7z/bit7z.hpp>
// #include <bit7z/bittypes.hpp>
// #include <bit7z/bitformat.hpp>
// #include <bit7z/bitexception.hpp>
// #include <bit7z/bitfileextractor.hpp>
// #include <bit7z/bitfilecompressor.hpp>
// #include <bit7z/bitmemcompressor.hpp>
// #include <bit7z/bitarchivereader.hpp>

#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <locale>
#include <codecvt>
#include <dirent.h>
#include <sys/stat.h>

namespace toolkit {

	// 辅助函数：将 std::wstring 转换为 bit7z::tstring
	inline bit7z::tstring to_tstring(const std::wstring& ws)
	{
#ifdef WIN32
		return ws;
#else
		std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
		return converter.to_bytes(ws);
#endif
	}
	inline bit7z::tstring to_tstring(const std::string& str)
	{
		return str; // 因为bit7z::tstring在Linux上就是std::string
	}
	// 字符串转换辅助函数
	std::wstring from_tstring(const bit7z::tstring& tstr)
	{
#ifdef WIN32
		return tstr;
#else
		std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
		return converter.from_bytes(tstr);
#endif
	}

	// 辅助函数：将 wstring 转换为 string (用于路径)
	inline std::string wstring_to_string(const std::wstring& ws)
	{
		std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
		return converter.to_bytes(ws);
	}


	// 获取库路径（跨平台）
	bit7z::tstring getLibraryPath()
	{
#ifdef WIN32
		return L"7z.dll";
#else
		return "7z.so";
#endif
	}

	// 检测文件类型
	bool C7z::IsRarFile(const std::wstring& strFile)
	{
		try
		{
			bit7z::Bit7zLibrary lib(getLibraryPath());
			bit7z::BitArchiveReader reader(lib, to_tstring(strFile), bit7z::BitFormat::Rar);
			return true;
		}
		catch (const bit7z::BitException&)
		{
			return false;
		}
	}

	bool C7z::IsZipFile(const std::wstring& strFile)
	{
		try
		{
			bit7z::Bit7zLibrary lib(getLibraryPath());
			bit7z::BitArchiveReader reader(lib, to_tstring(strFile), bit7z::BitFormat::Zip);
			return true;
		}
		catch (const bit7z::BitException&)
		{
			return false;
		}
	}

	bool C7z::IsSevenZipFile(const std::wstring& strFile)
	{
		try
		{
			bit7z::Bit7zLibrary lib(getLibraryPath());
			bit7z::BitArchiveReader reader(lib, to_tstring(strFile), bit7z::BitFormat::SevenZip);
			return true;
		}
		catch (const bit7z::BitException&)
		{
			return false;
		}
	}

	// 获取格式
	const bit7z::BitInFormat& C7z::GetBitFormat(const CompressFormat& format)
	{
		if (format == eSevenZip)
		{
			return bit7z::BitFormat::SevenZip;
		}
		else if (format == eZip)
		{
			return bit7z::BitFormat::Zip;
		}
		else if (format == eRar)
		{
			return bit7z::BitFormat::Rar;
		}
		else
		{
			throw std::invalid_argument("Unsupported format");
		}
	}

	// 根据格式解压文件
	int C7z::DecompressTypeFile(const std::wstring& strFileLoadPath,
								const std::wstring& strPath,
								const CompressFormat format,
								const std::vector<std::wstring>& vecFilter /*= {}*/,
								const std::wstring& strPassword /*= L""*/,
								ProgressCallbackFunc progressCallback /*= nullptr*/)
	{
		try
		{
			bit7z::Bit7zLibrary lib(getLibraryPath());
			const bit7z::BitInFormat& bitFormat = GetBitFormat(format);
			bit7z::BitFileExtractor extractor(lib, bitFormat);

			if (!strPassword.empty())
			{
				extractor.setPassword(to_tstring(strPassword));
			}
			if (progressCallback)
			{
				extractor.setProgressCallback(progressCallback);
			}

			if (vecFilter.empty())
			{
				extractor.extract(to_tstring(strFileLoadPath), to_tstring(strPath));
			}
			else
			{
				for (const auto& filter : vecFilter)
				{
					extractor.extractMatching(to_tstring(strFileLoadPath),
											  to_tstring(filter),
											  to_tstring(strPath));
				}
			}
			return 0;
		}
		catch (const bit7z::BitException& e)
		{
			std::cerr << e.what() << std::endl;
			return -1;
		}
	}

	int C7z::compress(const std::wstring& strFolder,
					  const std::wstring& strFileSavePath,
					  const std::wstring& strPassword /*= L""*/,
					  ProgressCallbackFunc progressCallback /*= nullptr*/)
	{
		try
		{
			bit7z::Bit7zLibrary lib(getLibraryPath());
			bit7z::BitFileCompressor compressor(lib, bit7z::BitFormat::SevenZip);


			std::vector<bit7z::tstring> vecFileAndDir;

			// 使用 dirent 遍历目录，避免 Boost filesystem
			std::string dirPathStr = wstring_to_string(strFolder);
			DIR* dir = opendir(dirPathStr.c_str());
			if (dir)
			{
				struct dirent* entry;
				while ((entry = readdir(dir)) != nullptr)
				{
					std::string filename = entry->d_name;
					if (filename == "." || filename == "..")
						continue;
					std::string fullPath = dirPathStr + "/" + filename;
					struct stat statbuf;
					if (stat(fullPath.c_str(), &statbuf) == 0)
					{
						if (S_ISREG(statbuf.st_mode) || S_ISDIR(statbuf.st_mode))
						{
							vecFileAndDir.push_back(to_tstring(filename));
						}
					}
				}
				closedir(dir);
			}

			//   std::vector<bit7z::tstring> vecFileAndDir;
			//	 // --- 使用 Boost.Filesystem 替换 ---
			//// 将输入的宽字符串路径转换为 boost::filesystem::path
			//	 boost::filesystem::path dirPath(strFolder); // path 构造函数支持 wstring
			//	 // 遍历目录
			//	 for (const auto& entry : boost::filesystem::directory_iterator(strFolder)) {
			//		 const boost::filesystem::path& filePath = entry.path();
			//		 // 判断是否为普通文件或目录
			//		 if (boost::filesystem::is_regular_file(filePath) ||
			//boost::filesystem::is_directory(filePath)) {
			//			 vecFileAndDir.push_back(to_tstring(filePath.wstring()));
			//		 }
			//	 }

			if (!strPassword.empty())
			{
				compressor.setPassword(to_tstring(strPassword));
			}
			if (progressCallback)
			{
				compressor.setProgressCallback(progressCallback);
			}

			compressor.compress(vecFileAndDir, to_tstring(strFileSavePath));
			return 0;
		}
		catch (const bit7z::BitException& e)
		{
			std::string strError = e.what();
			if (strError.find("Can't create archive file") != std::string::npos)
			{
				strError = "无法创建压缩文件";
			}
			std::cerr << strError << std::endl;
			return -1;
		}
	}

	// 解压
	int C7z::decompress(const std::wstring& strFileLoadPath,
						const std::wstring& strPath,
						const std::vector<std::wstring>& vecFilter /*= {}*/,
						const std::wstring& strPassword /*= L""*/,
						ProgressCallbackFunc progressCallback /*= nullptr*/)
	{
		try
		{
			bit7z::Bit7zLibrary lib(getLibraryPath());
			bit7z::BitFileExtractor extractor(lib, bit7z::BitFormat::SevenZip);

			if (!strPassword.empty())
			{
				extractor.setPassword(to_tstring(strPassword));
			}
			if (progressCallback)
			{
				extractor.setProgressCallback(progressCallback);
			}

			if (vecFilter.empty())
			{
				extractor.extract(to_tstring(strFileLoadPath), to_tstring(strPath));
			}
			else
			{
				for (const auto& filter : vecFilter)
				{
					extractor.extractMatching(to_tstring(strFileLoadPath),
											  to_tstring(filter),
											  to_tstring(strPath));
				}
			}
			return 0;
		}
		catch (const bit7z::BitException& e)
		{
			std::cerr << e.what() << std::endl;
			return 1;
		}
	}

	// 解压到内存
	int C7z::DecompressInfo(const std::wstring& strFileLoadPath,
							std::map<std::wstring, std::vector<unsigned char>>& mapFileNameAndInfo)
	{
		try
		{
			bit7z::Bit7zLibrary lib(getLibraryPath());
			bit7z::BitArchiveReader reader(lib,
										   to_tstring(strFileLoadPath),
										   bit7z::BitFormat::SevenZip);

			for (const auto& item : reader.items())
			{
				if (!item.isDir())
				{
					std::vector<bit7z::byte_t> buffer;
					reader.extractTo(buffer, item.index());
					mapFileNameAndInfo[from_tstring(item.path())] =
						std::vector<unsigned char>(buffer.begin(), buffer.end());
				}
			}
			return 0;
		}
		catch (const bit7z::BitException& e)
		{
			std::string strError = e.what();
			if (strError.find("Cannot load 7-zip library") != std::string::npos)
			{
				return -1;
			}
			else if (strError.find("Cannot open archive") != std::string::npos)
			{
				return -2;
			}
			return -3;
		}
	}

	// 根据路径解压到内存
	int C7z::DecompressInfoByPath(
		const std::wstring& strFileLoadPath,
		const std::vector<std::wstring>& vecFilePath,
		std::map<std::wstring, std::vector<unsigned char>>& mapFileNameAndInfo)
	{
		if (strFileLoadPath.empty())
			return -1;
		if (vecFilePath.empty())
			return -2;

		try
		{
			bit7z::Bit7zLibrary lib(getLibraryPath());
			bit7z::BitArchiveReader reader(lib,
										   to_tstring(strFileLoadPath),
										   bit7z::BitFormat::SevenZip);

			for (const auto& item : reader.items())
			{
				if (!item.isDir())
				{
					for (const auto& path : vecFilePath)
					{
						if (item.path() == to_tstring(path))
						{
							std::vector<bit7z::byte_t> buffer;
							reader.extractTo(buffer, item.index());
							mapFileNameAndInfo[path] =
								std::vector<unsigned char>(buffer.begin(), buffer.end());
							break;
						}
					}
					if (mapFileNameAndInfo.size() == vecFilePath.size())
						break;
				}
			}
			return 0;
		}
		catch (const bit7z::BitException&)
		{
			return -3;
		}
	}

	// 从内存压缩
	int C7z::Compress(std::map<std::wstring, std::vector<unsigned char>>& fileMap,
					  std::wstring strOutputPath)
	{
		try
		{
			bit7z::Bit7zLibrary lib(getLibraryPath());
			bit7z::BitMemCompressor compressor(lib, bit7z::BitFormat::SevenZip);
			compressor.setCompressionMethod(bit7z::BitCompressionMethod::Lzma2);
			compressor.setCompressionLevel(bit7z::BitCompressionLevel::Fastest);
			bit7z::BitOutputArchive outputArchive(compressor);
			for (const auto& pair : fileMap)
			{
				std::vector<bit7z::byte_t> buffer(pair.second.begin(), pair.second.end());
				outputArchive.addFile(buffer, to_tstring(pair.first));
			}
			outputArchive.compressTo(to_tstring(strOutputPath));
			return 0;
		}
		catch (const bit7z::BitException&)
		{
			return -1;
		}
	}

	// 获取档案信息

	int C7z::GetBitArchiveInfo(const std::wstring& strGimPath,
							   std::unique_ptr<bit7z::BitArchiveReader> pInfo)
	{
		try
		{
			bit7z::Bit7zLibrary lib(getLibraryPath());
			pInfo = std::make_unique<bit7z::BitArchiveReader>(lib,
															  to_tstring(strGimPath),
															  bit7z::BitFormat::SevenZip);
			return 0;
		}
		catch (const bit7z::BitException&)
		{
			return -1;
		}
	}

	int C7z::Decompress(const std::wstring& strFileLoadPath,
						const std::wstring& strPath,
						const std::wstring& strPassword /*= L""*/,
						CancelCallbackFunc cancleCallback /*= nullptr*/)
	{
		try
		{
			bit7z::Bit7zLibrary lib(getLibraryPath());
			bit7z::BitFileExtractor extractor(lib, bit7z::BitFormat::SevenZip);
			if (!strPassword.empty())
			{
				extractor.setPassword(to_tstring(strPassword));
			}

			bit7z::BitArchiveReader reader(lib,
										   to_tstring(strFileLoadPath),
										   bit7z::BitFormat::SevenZip);
			for (const auto& item : reader.items())
			{
				if (cancleCallback && cancleCallback())
					break;
				extractor.extractMatching(to_tstring(strFileLoadPath),
										  item.path(),
										  to_tstring(strPath));
			}
			return 0;
		}
		catch (const bit7z::BitException& e)
		{
			std::cerr << e.what() << std::endl;
			return -1;
		}
	}

	// 压缩文件到包
	int C7z::CompressFileToPackage(const std::wstring& strFilePath,
								   const std::wstring& strPressPath)
	{
		try
		{
			bit7z::Bit7zLibrary lib(getLibraryPath());
			bit7z::BitFileCompressor compressor(lib, bit7z::BitFormat::SevenZip);
			compressor.setUpdateMode(true);
			compressor.compressFile(to_tstring(strFilePath), to_tstring(strPressPath));
			return 0;
		}
		catch (const bit7z::BitException&)
		{
			return -1;
		}
	}

	// 压缩文件夹到包
	int C7z::CompressFolderToPackage(const std::wstring& strFolderPath,
									 const std::wstring& strPressPath)
	{
		try
		{
			bit7z::Bit7zLibrary lib(getLibraryPath());
			bit7z::BitFileCompressor compressor(lib, bit7z::BitFormat::SevenZip);
			compressor.setUpdateMode(true);
			compressor.compressDirectory(to_tstring(strFolderPath), to_tstring(strPressPath));
			return 0;
		}
		catch (const bit7z::BitException&)
		{
			return -1;
		}
	}



	std::wstring C7z::str2wstr(const std::string& str)
	{
#ifdef WIN32
		if (str.empty())
			return L"";
		int size_needed = MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, nullptr, 0);
		std::wstring wstr(size_needed, 0);
		MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, &wstr[0], size_needed);
		return wstr;
#else
		std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
		return converter.from_bytes(str);
#endif
	}


} // namespace toolkit