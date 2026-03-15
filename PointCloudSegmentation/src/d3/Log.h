#ifndef D3S_LOG_H_
#define D3S_LOG_H_


#include <src/d3/Singleton.h>
#include <spdlog/spdlog.h>

// #define TINYGLTF_DUMP_TO_LOG

// 在文件开头添加条件编译
#ifdef WIN32
#include <tchar.h>
#else
#include <cstdarg>
#include <string>
#include <memory>
#endif


namespace d3s {

	/** @addtogroup  Toolkit
	 * @{
	 */
	//! 日志记录类
	class CLog : public Singleton<CLog>
	{
	public:
		enum Level : int
		{
			debug = 1,
			info = 2,
			warn = 3,
			err = 4,
		};

		enum class Sink
		{
			Disable,
			StdOut,
			StdOutColor,
			MSVC,
		};

		CLog();

		virtual ~CLog();


		/**
		 *  @brief    启用日志重定向
		 *
		 *  @param    Sink sink			重定向目标，如：StdOutColor(带颜色控制台)、MSVC(VS 输出窗口)
		 *
		 *  @return   void
		 */
		virtual void EnableRedirectTo(Sink sink);

		/**
		 * 设置日志输出层级
		 * @param [in] message 消息
		 */
		static void SetLevel(Level level);

		/**
		 * 日志层级是否输出
		 * @param [in] message 消息
		 */
		static bool ShouldLog(Level level);

#ifdef WIN32
		/**
		 * 记录日志信息
		 * @param [in] message 消息
		 */
		static void Error(_In_z_ _Printf_format_string_ LPCTSTR pszFormat, ...);

		/**
		 * 记录日志信息
		 * @param [in] message 消息
		 */
		static void Warn(_In_z_ _Printf_format_string_ LPCTSTR pszFormat, ...);

		/**
		 * 记录日志信息
		 * @param [in] message 消息
		 */
		static void Info(_In_z_ _Printf_format_string_ LPCTSTR pszFormat, ...);

		/**
		 * 记录日志信息
		 * @param [in] message 消息
		 */
		static void Debug(_In_z_ _Printf_format_string_ LPCTSTR pszFormat, ...);

		/**
		 *  @brief    通用日志打印
		 *
		 *  @param    LPCTSTR pszFormat			格式化字符串
		 *  @param    ...
		 *
		 *  @return   void
		 */
		static void Log(Level severity, _In_z_ _Printf_format_string_ LPCTSTR pszFormat, ...);
#else
		// Linux版本
		static void Error(const char* pszFormat, ...);
		static void Warn(const char* pszFormat, ...);
		static void Info(const char* pszFormat, ...);
		static void Debug(const char* pszFormat, ...);
		static void Log(Level severity, const char* pszFormat, ...);
#endif
		static void Log(Level severity, const std::string& format, ...);
		static void Log(Level severity, std::wstring format, ...);
		// 字符串版本
		static void Log(Level severity, const std::wstring& format, ...);

		/**
		 * 创建默认的日志保存路径
		 * @return
		 */
		static std::string CreateDefaultFilePath(const std::string& strLogFileName = "");

		/**
		 * 显示内存信息
		 * @param [in] strMsg 消息
		 */
		static void MemoryInfo(const std::string& strMsg);

		/**
		 * 将日志信息写入文件
		 * @return
		 **/
		static void Flush();

		///**
		//* 创建默认的日志保存路径
		//* @return
		//*/
		// static CString CreateDefaultFilePath(CString strLogFileName = _T(""));
		///**
		//* 显示内存信息
		//* @param [in] strMsg 消息
		//*/
		// static void MemoryInfo(CString strMsg);
		///**
		//* 将日志信息写入文件
		//* @return
		//**/
		// static void Flush();

	protected:
		std::shared_ptr<spdlog::logger> _logger;
		std::shared_ptr<spdlog::logger> _logger_redirect;

	}; /** * @} */


	class CTimeLog
	{
	public:
#ifdef WIN32
		CTimeLog(_In_z_ _Printf_format_string_ LPCTSTR pszFormat, ...);
#else
		CTimeLog(const char* pszFormat, ...);
#endif
		~CTimeLog();

		std::string _strTip;
#ifdef WIN32
		DWORD _nTime;
#else
		unsigned long _nTime; // 使用更通用的类型
#endif
	};
}

#define STR(x) (#x)


#endif // D3S_LOG_H_