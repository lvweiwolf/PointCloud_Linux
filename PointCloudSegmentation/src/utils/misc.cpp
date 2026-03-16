#include <src/utils/misc.h>
#include <src/utils/stringutil.h>
#include <src/utils/logging.h>

#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>

#include <iostream>
#include <fstream>
#include <cstring>

namespace d3s {
	namespace pcs {

		std::string EnsureTrailingSlash(const std::string& str)
		{
			if (str.length() > 0)
			{
				if (str.back() != '/')
				{
					return str + "/";
				}
			}
			else
			{
				return str + "/";
			}
			return str;
		}

		bool HasFileExtension(const std::string& file_name, const std::string& ext)
		{
			CHECK(!ext.empty());
			CHECK(ext.at(0) == '.');

			std::string ext_lower = ext;
			StringToLower(&ext_lower);
			if (file_name.size() >= ext_lower.size() &&
				file_name.substr(file_name.size() - ext_lower.size(), ext_lower.size()) ==
					ext_lower)
			{
				return true;
			}
			return false;
		}

		void SplitFileExtension(const std::string& path, std::string* root, std::string* ext)
		{
			const auto parts = StringSplit(path, ".");
			CHECK(parts.size() > 0);
			if (parts.size() == 1)
			{
				*root = parts[0];
				*ext = "";
			}
			else
			{
				*root = "";
				for (size_t i = 0; i < parts.size() - 1; ++i)
				{
					*root += parts[i] + ".";
				}
				*root = root->substr(0, root->length() - 1);
				if (parts.back() == "")
				{
					*ext = "";
				}
				else
				{
					*ext = "." + parts.back();
				}
			}
		}

		void FileCopy(const std::string& src_path, const std::string& dst_path, CopyType type)
		{
			switch (type)
			{
			case CopyType::COPY:
			{
				std::ifstream src(src_path, std::ios::binary);
				std::ofstream dst(dst_path, std::ios::binary);
				dst << src.rdbuf();
				break;
			}
			case CopyType::HARD_LINK:
				if (link(src_path.c_str(), dst_path.c_str()) != 0)
				{
					// handle error
				}
				break;
			case CopyType::SOFT_LINK:
				if (symlink(src_path.c_str(), dst_path.c_str()) != 0)
				{
					// handle error
				}
				break;
			}
		}

		bool ExistsFile(const std::string& path)
		{
			struct stat st;
			return stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
		}

		bool ExistsDir(const std::string& path)
		{
			struct stat st;
			return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
		}

		bool ExistsPath(const std::string& path)
		{
			struct stat st;
			return stat(path.c_str(), &st) == 0;
		}

		void CreateDirIfNotExists(const std::string& path)
		{
			if (!ExistsDir(path))
			{
				mkdir(path.c_str(), 0755);
			}
		}

		void CreateDirsIfNotExists(const std::string& path)
		{
			if (!ExistsDir(path))
			{
				// 递归创建目录
				size_t pos = 0;
				std::string dir;
				while ((pos = path.find('/', pos)) != std::string::npos)
				{
					dir = path.substr(0, pos);
					if (!dir.empty() && !ExistsDir(dir))
					{
						mkdir(dir.c_str(), 0755);
					}
					pos++;
				}
				if (!ExistsDir(path))
				{
					mkdir(path.c_str(), 0755);
				}
			}
		}

		std::string GetPathBaseName(const std::string& path)
		{
			const std::vector<std::string> names = StringSplit(StringReplace(path, "\\", "/"), "/");
			if (names.size() > 1 && names.back() == "")
			{
				return names[names.size() - 2];
			}
			else
			{
				return names.back();
			}
		}

		std::string GetParentDir(const std::string& path)
		{
			size_t pos = path.find_last_of('/');
			if (pos != std::string::npos)
			{
				return path.substr(0, pos);
			}
			return "";
		}

		std::string GetRelativePath(const std::string& from, const std::string& to)
		{
			// This implementation is adapted from:
			// https://stackoverflow.com/questions/10167382
			// A native implementation in boost::filesystem is only available starting
			// from boost version 1.60.
			using namespace boost::filesystem;

			path from_path = canonical(path(from));
			path to_path = canonical(path(to));

			// Start at the root path and while they are the same then do nothing then
			// when they first diverge take the entire from path, swap it with '..'
			// segments, and then append the remainder of the to path.
			path::const_iterator from_iter = from_path.begin();
			path::const_iterator to_iter = to_path.begin();

			// Loop through both while they are the same to find nearest common directory
			while (from_iter != from_path.end() && to_iter != to_path.end() &&
				   (*to_iter) == (*from_iter))
			{
				++to_iter;
				++from_iter;
			}

			// Replace from path segments with '..' (from => nearest common directory)
			path rel_path;
			while (from_iter != from_path.end())
			{
				rel_path /= "..";
				++from_iter;
			}

			// Append the remainder of the to path (nearest common directory => to)
			while (to_iter != to_path.end())
			{
				rel_path /= *to_iter;
				++to_iter;
			}

			return rel_path.string();
		}

		std::vector<std::string> GetFileList(const std::string& path)
		{
			std::vector<std::string> file_list;
			DIR* dir = opendir(path.c_str());
			if (dir)
			{
				struct dirent* entry;
				while ((entry = readdir(dir)) != nullptr)
				{
					std::string filename = entry->d_name;
					if (filename == "." || filename == "..")
						continue;
					std::string fullPath = path + "/" + filename;
					struct stat st;
					if (stat(fullPath.c_str(), &st) == 0 && S_ISREG(st.st_mode))
					{
						file_list.push_back(fullPath);
					}
				}
				closedir(dir);
			}
			return file_list;
		}

		std::vector<std::string> GetRecursiveFileList(const std::string& path)
		{
			std::vector<std::string> file_list;
			// 简单实现，非递归
			DIR* dir = opendir(path.c_str());
			if (dir)
			{
				struct dirent* entry;
				while ((entry = readdir(dir)) != nullptr)
				{
					std::string filename = entry->d_name;
					if (filename == "." || filename == "..")
						continue;
					std::string fullPath = path + "/" + filename;
					struct stat st;
					if (stat(fullPath.c_str(), &st) == 0)
					{
						if (S_ISREG(st.st_mode))
						{
							file_list.push_back(fullPath);
						}
						else if (S_ISDIR(st.st_mode))
						{
							// 递归，但为了简单，暂时不实现
						}
					}
				}
				closedir(dir);
			}
			return file_list;
		}

		std::vector<std::string> GetDirList(const std::string& path)
		{
			std::vector<std::string> dir_list;
			for (auto it = boost::filesystem::directory_iterator(path);
				 it != boost::filesystem::directory_iterator();
				 ++it)
			{
				if (boost::filesystem::is_directory(*it))
				{
					const boost::filesystem::path dir_path = *it;
					dir_list.push_back(dir_path.string());
				}
			}
			return dir_list;
		}

		std::vector<std::string> GetRecursiveDirList(const std::string& path)
		{
			std::vector<std::string> dir_list;
			for (auto it = boost::filesystem::recursive_directory_iterator(path);
				 it != boost::filesystem::recursive_directory_iterator();
				 ++it)
			{
				if (boost::filesystem::is_directory(*it))
				{
					const boost::filesystem::path dir_path = *it;
					dir_list.push_back(dir_path.string());
				}
			}
			return dir_list;
		}

		size_t GetFileSize(const std::string& path)
		{
			std::ifstream file(path, std::ifstream::ate | std::ifstream::binary);
			CHECK_MSG(file.is_open(), path.c_str());
			return file.tellg();
		}

		void PrintHeading1(const std::string& heading)
		{
			std::cout << std::endl << std::string(78, '=') << std::endl;
			std::cout << heading << std::endl;
			std::cout << std::string(78, '=') << std::endl << std::endl;
		}

		void PrintHeading2(const std::string& heading)
		{
			std::cout << std::endl << heading << std::endl;
			std::cout << std::string(std::min<int>(heading.size(), 78), '-') << std::endl;
		}

		template <>
		std::vector<std::string> CSVToVector(const std::string& csv)
		{
			auto elems = StringSplit(csv, ",;");
			std::vector<std::string> values;
			values.reserve(elems.size());
			for (auto& elem : elems)
			{
				StringTrim(&elem);
				if (elem.empty())
				{
					continue;
				}
				values.push_back(elem);
			}
			return values;
		}

		template <>
		std::vector<int> CSVToVector(const std::string& csv)
		{
			auto elems = StringSplit(csv, ",;");
			std::vector<int> values;
			values.reserve(elems.size());
			for (auto& elem : elems)
			{
				StringTrim(&elem);
				if (elem.empty())
				{
					continue;
				}
				try
				{
					values.push_back(std::stoi(elem));
				}
				catch (const std::invalid_argument&)
				{
					return std::vector<int>(0);
				}
			}
			return values;
		}

		template <>
		std::vector<float> CSVToVector(const std::string& csv)
		{
			auto elems = StringSplit(csv, ",;");
			std::vector<float> values;
			values.reserve(elems.size());
			for (auto& elem : elems)
			{
				StringTrim(&elem);
				if (elem.empty())
				{
					continue;
				}
				try
				{
					values.push_back(std::stod(elem));
				}
				catch (const std::invalid_argument&)
				{
					return std::vector<float>(0);
				}
			}
			return values;
		}

		template <>
		std::vector<double> CSVToVector(const std::string& csv)
		{
			auto elems = StringSplit(csv, ",;");
			std::vector<double> values;
			values.reserve(elems.size());
			for (auto& elem : elems)
			{
				StringTrim(&elem);
				if (elem.empty())
				{
					continue;
				}
				try
				{
					values.push_back(std::stold(elem));
				}
				catch (const std::invalid_argument&)
				{
					return std::vector<double>(0);
				}
			}
			return values;
		}

		std::vector<std::string> ReadTextFileLines(const std::string& path)
		{
			std::ifstream file(path);
			CHECK_MSG(file.is_open(), path.c_str());

			std::string line;
			std::vector<std::string> lines;
			while (std::getline(file, line))
			{
				StringTrim(&line);

				if (line.empty())
				{
					continue;
				}

				lines.push_back(line);
			}

			return lines;
		}

		void RemoveCommandLineArgument(const std::string& arg, int* argc, char** argv)
		{
			for (int i = 0; i < *argc; ++i)
			{
				if (argv[i] == arg)
				{
					for (int j = i + 1; j < *argc; ++j)
					{
						argv[i] = argv[j];
					}
					*argc -= 1;
					break;
				}
			}
		}

		std::string FormatBytes(int64_t aBytes)
		{
			if (aBytes < (int64_t)1024)
				return StringPrintf("%dB", (uint32_t)aBytes & 0xffffffff);
			else if (aBytes < (int64_t)1024 * 1024)
				return StringPrintf("%.02fKB", (double)aBytes / (1024.0));
			else if (aBytes < (int64_t)1024 * 1024 * 1024)
				return StringPrintf("%.02fMB", (double)aBytes / (1024.0 * 1024.0));
			else if (aBytes < (int64_t)1024 * 1024 * 1024 * 1024)
				return StringPrintf("%.02fGB", (double)aBytes / (1024.0 * 1024.0 * 1024.0));
			else
				return StringPrintf("%.02fTB",
									(double)aBytes / (1024.0 * 1024.0 * 1024.0 * 1024.0));
		}
	}
}