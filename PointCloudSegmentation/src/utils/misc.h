#pragma once
#include<boost/filesystem.hpp>
#include <string>
#include <vector>
namespace d3s {
	namespace pcs {

		enum class CopyType { COPY, HARD_LINK, SOFT_LINK };
		/**
		 *  @brief    如果字符串的结尾没有'/'，则在字符串后面加上'/'
		 *
		 *  @param    const std::string & str  处理前字符串
		 *  @return   std::string	处理后字符串
		 */
		std::string EnsureTrailingSlash(const std::string& str);

		/**
		 *  @brief    检查文件名是否有文件扩展名（不区分大小写）
		 *
		 *  @param    const std::string & file_name	文件名
		 *  @param    const std::string & ext	扩展名
		 *  @return   bool
		 */
		bool HasFileExtension(const std::string& file_name, const std::string& ext);

		/**
		 *  @brief    将路径拆分成根和扩展名，例如 "dir/file.jpg "拆成 "dir/file "和".jpg"
		 *
		 *  @param    const std::string & path	路径
		 *  @param    std::string * root	根部
		 *  @param    std::string * ext	扩展名
		 *  @return   void
		 */
		void SplitFileExtension(const std::string& path, std::string* root, std::string* ext);

		/**
		 *  @brief    将文件从源路径复制或链接到目标路径
		 *
		 *  @param    const std::string & src_path	源路径
		 *  @param    const std::string & dst_path	目标路径
		 *  @param    CopyType type					拷贝类型
		 *
		 *  @return   void
		 */
		void FileCopy(const std::string& src_path,
					  const std::string& dst_path,
					  CopyType type = CopyType::COPY);

		/**
		 *  @brief    检查路径是否指向现有目录
		 *
		 *  @param    const std::string & path
		 *  @return   bool
		 */
		bool ExistsFile(const std::string& path);

		/**
		 *  @brief    检查路径是否指向现有目录
		 *
		 *  @param    const std::string & path
		 *  @return   bool
		 */
		bool ExistsDir(const std::string& path);

		/**
		 *  @brief    检查路径是否指向现有的文件或目录
		 *
		 *  @param    const std::string & path
		 *  @return   bool
		 */
		bool ExistsPath(const std::string& path);

		/**
		 *  @brief   如果该目录不存在，则创建该目录
		 *
		 *  @param    const std::string & path
		 *  @return   void
		 */
		void CreateDirIfNotExists(const std::string& path);
		void CreateDirsIfNotExists(const std::string& path);

		/**
		 *  @brief    提取路径的基本名称，例如，"image.jpg "为"/dir/image.jpg"
		 *
		 *  @param    const std::string & path
		 *  @return   std::string
		 */
		std::string GetPathBaseName(const std::string& path);

		/**
		 *  @brief    获取给定路径的父目录的路径
		 *
		 *  @param    const std::string & path
		 *  @return   std::string
		 */
		std::string GetParentDir(const std::string& path);

		/**
		 *  @brief    获取from和to之间的相对路径。from和to的路径必须同时存在
		 *
		 *  @param    const std::string & from
		 *  @param    const std::string & to
		 *  @return   std::string
		 */
		std::string GetRelativePath(const std::string& from, const std::string& to);

		/**
		 *  @brief    将多条路径连接成一条路径
		 *
		 *  @param    T const & ... paths
		 *  @return   std::string
		 */
		template <typename... T>
		std::string JoinPaths(T const&... paths);

		/**
		 *  @brief    返回目录中的文件列表
		 *
		 *  @param    const std::string & path
		 *  @return   std::vector<std::string>
		 */
		std::vector<std::string> GetFileList(const std::string& path);

		/**
		 *  @brief    返回文件列表，在所有子目录中递归
		 *
		 *  @param    const std::string & path
		 *  @return   std::vector<std::string>
		 */
		std::vector<std::string> GetRecursiveFileList(const std::string& path);

		/**
		 *  @brief    返回目录列表
		 *
		 *  @param    const std::string & path
		 *  @return   std::vector<std::string>
		 */
		std::vector<std::string> GetDirList(const std::string& path);

		/**
		 *  @brief    返回目录列表，在所有子目录中递归
		 *
		 *  @param    const std::string & path
		 *  @return   std::vector<std::string>
		 */
		std::vector<std::string> GetRecursiveDirList(const std::string& path);

		/**
		 *  @brief    获取文件的字节数
		 *
		 *  @param    const std::string & path
		 *  @return   size_t
		 */
		size_t GetFileSize(const std::string& path);

		/**
		 *  @brief    将带大写和下划线的一级标题打印到`std::cout`
		 *
		 *  @param    const std::string & heading 标题名称
		 *  @return   void
		 */
		void PrintHeading1(const std::string& heading);

		/**
		 *  @brief    将带下划线的二阶标题打印到`std::cout`
		 *
		 *  @param    const std::string & heading
		 *  @return   void
		 */
		void PrintHeading2(const std::string& heading);

		/**
		 *  @brief    检查数组是否包含元素
		 *
		 *  @param    const std::vector<T> & vector 元素列表
		 *  @param    const T value 检查元素
		 *  @return   bool
		 */
		template <typename T>
		bool VectorContainsValue(const std::vector<T>& vector, const T value);

		template <typename T>
		bool VectorContainsDuplicateValues(const std::vector<T>& vector);

		/**
		 *  @brief    将CSV行解析为一个值的列表
		 *
		 *  @param    const std::string & csv
		 *  @return
		 */
		template <typename T>
		std::vector<T> CSVToVector(const std::string& csv);

		/**
		 *  @brief    将列表中的值连接成逗号分隔的列表
		 *
		 *  @param    const std::vector<T> & values
		 *  @return   std::string
		 */
		template <typename T>
		std::string VectorToCSV(const std::vector<T>& values);

		/**
		 *  @brief    将文本文件的每一行读成一个单独的元素。空行会被忽略，前导/尾部的空白会被删除
		 *
		 *  @param    const std::string & path	文件路径
		 *  @return   std::vector<std::string>	所有行数据
		 */
		std::vector<std::string> ReadTextFileLines(const std::string& path);

		/**
		 *  @brief    从命令行参数列表中删除一个参数
		 *
		 *  @param    const std::string & arg	命令行参数字符串
		 *  @param    int * argc		参数数量
		 *  @param    char * * argv	参数列表
		 *  @return   void
		 */
		void RemoveCommandLineArgument(const std::string& arg, int* argc, char** argv);

		/**
		 *  @brief    从字节数抓换单位
		 *
		 *  @param    int64_t aBytes
		 *
		 *  @return   std::string
		 */
		std::string FormatBytes(int64_t aBytes);


		////////////////////////////////////////////////////////////////////////////////
		// 实现部分
		////////////////////////////////////////////////////////////////////////////////

		template <typename... T>
		std::string JoinPaths(T const&... paths)
		{
			boost::filesystem::path result;
			int unpack[]{ 0, (result = result / boost::filesystem::path(paths), 0)... };
			static_cast<void>(unpack);
			return result.string();
		}

		template <typename T>
		bool VectorContainsValue(const std::vector<T>& vector, const T value)
		{
			return std::find_if(vector.begin(), vector.end(), [value](const T element) {
					   return element == value;
				   }) != vector.end();
		}

		template <typename T>
		bool VectorContainsDuplicateValues(const std::vector<T>& vector)
		{
			std::vector<T> unique_vector = vector;
			return std::unique(unique_vector.begin(), unique_vector.end()) != unique_vector.end();
		}

		template <typename T>
		std::string VectorToCSV(const std::vector<T>& values)
		{
			std::string string;
			for (const T value : values)
			{
				string += std::to_string(value) + ", ";
			}
			return string.substr(0, string.length() - 2);
		}
	}
}
