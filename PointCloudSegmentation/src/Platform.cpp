#include <src/Platform.h>


#include <cstring>
#include <unistd.h>
#include <uuid/uuid.h>
#include <sys/stat.h>

#if defined(_WIN32) || defined(_WIN64)
#define PATH_MAX 256
#else
#define PATH_MAX 4096
#endif

namespace CLibToolkit {

	std::string CreateGuid()
	{
		uuid_t uuid;
		char guidStr[37]; // UUID字符串长度固定为36+1

		uuid_generate(uuid);
		uuid_unparse(uuid, guidStr);

		return std::string(guidStr);
	}

	std::string GetAppModulePath()
	{
		char exePath[PATH_MAX];
		ssize_t len = readlink("/proc/self/exe", exePath, sizeof(exePath) - 1);

		if (len == -1)
		{
			// 如果读取/proc/self/exe失败，尝试使用argv[0]
			return "./"; // 返回当前目录作为回退
		}

		exePath[len] = '\0';

		// 找到最后一个'/'的位置
		char* lastSlash = strrchr(exePath, '/');
		if (lastSlash != nullptr)
		{
			*(lastSlash + 1) = '\0'; // 截断文件名，保留目录部分
		}

		return std::string(exePath);
	}
	
	std::string GetSoftTempPath()
	{
		static std::string softTempPath;

		if (softTempPath.empty())
		{
			// 获取应用程序模块路径 + Temp/ + GUID + /
			std::string modulePath = GetAppModulePath();

			// 创建基础临时目录：{模块路径}/Temp/
			std::string tempDir = modulePath + "Temp/";

			// 确保目录存在
			mkdir(tempDir.c_str(), 0755); // 创建Temp目录

			// 生成GUID作为子目录名
			std::string guid = CreateGuid();

			// 完整路径：{模块路径}/Temp/{GUID}/
			softTempPath = tempDir + guid + "/";

			// 创建GUID子目录
			mkdir(softTempPath.c_str(), 0755);
		}

		return softTempPath;
	}
}
