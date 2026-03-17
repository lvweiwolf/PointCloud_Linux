//////////////////////////////////////////////////////////////////////
// 文件名称：logging.h
// 功能描述：Google系日志工具
// 创建标识：吕伟	2021/2/4
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef LOGGING_H_
#define LOGGING_H_

#include <src/utils/stringutil.h>
#include <src/d3/Log.h>

#ifdef TINYGLTF_DUMP_TO_LOG
#define PCS_ERROR(...) d3s::CLog::Log(d3s::CLog::err, __VA_ARGS__)
#define PCS_WARN(...) d3s::CLog::Log(d3s::CLog::warn, __VA_ARGS__)
#define PCS_INFO(...) d3s::CLog::Log(d3s::CLog::info, __VA_ARGS__)
#define PCS_DEBUG(...) d3s::CLog::Log(d3s::CLog::debug, __VA_ARGS__)
#else
#define PCS_ERROR(...)
#define PCS_WARN(...)
#define PCS_INFO(...)
#define PCS_DEBUG(...)
#endif

#define STR(x) (#x)

#define CHECK(condition)                                                                           \
	do                                                                                             \
	{                                                                                              \
		if (!(condition))                                                                          \
		{                                                                                          \
			PCS_ERROR("%s In File %s, in line %d\n", STR(condition), __FILE__, __LINE__);          \
			throw std::runtime_error(d3s::pcs::StringPrintf("%s In File %s, in line %d\n",         \
															STR(condition),                        \
															__FILE__,                              \
															__LINE__)                              \
										 .c_str());                                                \
		}                                                                                          \
	} while (0)


#define CHECK_MSG(condition, msg)                                                                  \
	do                                                                                             \
	{                                                                                              \
		if (!(condition))                                                                          \
		{                                                                                          \
			PCS_ERROR("%s In File %s, in line %d\n, and: %s",                                      \
					  STR(condition),                                                              \
					  __FILE__,                                                                    \
					  __LINE__,                                                                    \
					  msg);                                                                        \
			throw std::runtime_error(                                                              \
				d3s::pcs::StringPrintf("%s In File %s, in line %d\n, and: %s",                     \
									   STR(condition),                                             \
									   __FILE__,                                                   \
									   __LINE__,                                                   \
									   msg)                                                        \
					.c_str());                                                                     \
		}                                                                                          \
	} while (0)


#endif // LOGGING_H_