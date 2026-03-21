#ifndef D3S_LOG_H_
#define D3S_LOG_H_

#include <include/CommonToolsExport.h>
#include <include/Singleton.h>
#include <memory>
#include <string>

// 前向声明 spdlog 的命名空间（实际包含头文件时再引入）
namespace spdlog
{
	class logger;
}

namespace d3s
{


	class COMMONTOOLS_EXPORT CLog :public Singleton<CLog>
	{
	public:
		/**
		 * 测试输出 无具体作用
		 */
		static void LogPrint();
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
			Disable,	//取消控制台输出(默认)
			StdOut,		//控制台输出
			StdOutColor,//带颜色的控制台输出
			//MSVC,
		};

		CLog();

		virtual ~CLog();

		/**
	 *  @brief    启用日志重定向（控制 spdlog 的输出目标）
	 *  @param    Sink sink   重定向目标
	 */
		virtual void EnableRedirectTo(Sink sink);

		/**
		 * 设置日志输出层级
		 */
		static void SetLevel(Level level);

		/**
		 * 判断指定层级是否应该输出
		 */
		static bool ShouldLog(Level level);

		/**
		* 记录日志信息 (printf 风格，char* 版本)
		*/
		static void Error(const char* pszFormat, ...);
		static void Warn(const char* pszFormat, ...);
		static void Info(const char* pszFormat, ...);
		static void Debug(const char* pszFormat, ...);
		static void Log(Level severity, const char* pszFormat, ...);
		static void Log(Level severity, const std::string& format, ...);


		/**
		   * 记录日志信息 (printf 风格，wchar_t* 版本)
		   */
		static void Error(const wchar_t* pszFormat, ...);
		static void Warn(const wchar_t* pszFormat, ...);
		static void Info(const wchar_t* pszFormat, ...);
		static void Debug(const wchar_t* pszFormat, ...);
		static void Log(Level severity, const wchar_t* pszFormat, ...);
		static void Log(Level severity, const std::wstring& format, ...);

		/**
			* 创建默认的日志保存路径（基于程序所在目录的 Log 子目录，按日期命名）
			* @param strLogFileName 日志文件名前缀（默认为 "SystemLog"）
			* @return 完整路径字符串 (UTF-8)
			*/
		static std::string CreateDefaultFilePath(const std::string& strLogFileName = "");



		/**
		* 显示内存信息（写入日志）
		*/
		static void MemoryInfo(const std::string& strMsg);

		/**
			* 刷新日志缓冲区
			*/
		static void Flush();



	private:
		// 获取当前可执行文件所在目录
		static std::string GetExeDirectory();
	protected:
		std::shared_ptr<spdlog::logger> _logger;
		std::shared_ptr<spdlog::logger> _logger_redirect;

	};
	/**
		 * @brief 简单计时类，析构时输出耗时（毫秒）
		 */
	class CTimeLog
	{
	public:
		CTimeLog(const char* pszFormat, ...);
		CTimeLog(const wchar_t* pszFormat, ...);
		~CTimeLog();

	private:
		std::string _strTip;
		long long   _nStartMs = 0;   // 起始毫秒数
	};
}

#endif // D3S_LOG_H_