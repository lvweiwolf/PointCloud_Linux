#include <include/Log.h>
#include <string>

#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <regex>
#include <chrono>
#include <thread>
#include <mutex>
#include <codecvt>

#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <dirent.h>

// 获取进程内存信息所需头文件
#include <fstream>
#include <string>
#include <sstream>


#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/sinks/stdout_color_sinks.h"

using namespace d3s;
//提前定义CLog
template<>
CLog* Singleton<CLog>::_pInstance{ new CLog() };

void CLog::LogPrint()
{
	spdlog::info("Hello, {}!", "World");
}

CLog::CLog()
{
	_logger = spdlog::basic_logger_mt("SystemLog", CreateDefaultFilePath().c_str());
	_logger->set_level(spdlog::level::debug);
	_logger->flush_on(spdlog::level::info);
}

CLog::~CLog()
{

}

void CLog::EnableRedirectTo(Sink sink)
{
	if (sink == Sink::Disable)
	{
		_logger_redirect.reset();
		return;
	}

	switch (sink)
	{
	case Sink::StdOut:
		_logger_redirect = spdlog::stdout_logger_mt("Redirect");
		break;
	case Sink::StdOutColor:
		_logger_redirect = spdlog::stdout_color_mt("Redirect");
		break;
	//case Sink::MSVC:
	//	_logger_redirect = spdlog::synchronous_factory::create<spdlog::sinks::msvc_sink_mt>("Redirect");
	//	break;
	default:
		_logger_redirect.reset();
		break;
	}

	if (_logger_redirect)
	{
		_logger_redirect->set_level(_logger->level());
		_logger_redirect->flush_on(spdlog::level::info);
	}
}

void CLog::SetLevel(Level level)
{
	GetInst()->_logger->set_level((spdlog::level::level_enum)level);

	if (GetInst()->_logger_redirect)
		GetInst()->_logger_redirect->set_level((spdlog::level::level_enum)level);
}

bool CLog::ShouldLog(Level level)
{
	return GetInst()->_logger->should_log((spdlog::level::level_enum)level);
}

void CLog::Error(const char* pszFormat, ...)
{
	va_list args;
	va_start(args, pszFormat);
	char buffer[4096];
	vsnprintf(buffer, sizeof(buffer), pszFormat, args);
	va_end(args);

	GetInst()->_logger->error(buffer);

	if (GetInst()->_logger_redirect)
		GetInst()->_logger_redirect->error(buffer);
}

void CLog::Warn(const char* pszFormat, ...)
{
	va_list args;
	va_start(args, pszFormat);
	char buffer[4096];
	vsnprintf(buffer, sizeof(buffer), pszFormat, args);
	va_end(args);
	GetInst()->_logger->warn(buffer);

	if (GetInst()->_logger_redirect)
		GetInst()->_logger_redirect->warn(buffer);
}

void CLog::Info(const char* pszFormat, ...)
{
	va_list args;
	va_start(args, pszFormat);
	char buffer[4096];
	vsnprintf(buffer, sizeof(buffer), pszFormat, args);
	va_end(args);
	GetInst()->_logger->info(buffer);

	if (GetInst()->_logger_redirect)
		GetInst()->_logger_redirect->info(buffer);
}

void CLog::Debug(const char* pszFormat, ...)
{
	va_list args;
	va_start(args, pszFormat);
	char buffer[4096];
	vsnprintf(buffer, sizeof(buffer), pszFormat, args);
	va_end(args);
	GetInst()->_logger->debug(buffer);

	if (GetInst()->_logger_redirect)
		GetInst()->_logger_redirect->debug(buffer);
}

void CLog::Log(Level severity, const char* pszFormat, ...)
{
	va_list args;
	va_start(args, pszFormat);
	char buffer[4096];
	vsnprintf(buffer, sizeof(buffer), pszFormat, args);
	va_end(args);
	GetInst()->_logger->log((spdlog::level::level_enum)severity, buffer);

	if (GetInst()->_logger_redirect)
		GetInst()->_logger_redirect->log((spdlog::level::level_enum)severity, buffer);
}

void CLog::Log(Level severity, const std::string& format, ...)
{
	va_list args;
	va_start(args, format);
	char buffer[4096];
	vsnprintf(buffer, sizeof(buffer), format.c_str(), args);
	va_end(args);
	GetInst()->_logger->log((spdlog::level::level_enum)severity, buffer);
	if (GetInst()->_logger_redirect)
		GetInst()->_logger_redirect->log((spdlog::level::level_enum)severity, buffer);
}

void CLog::Error(const wchar_t* pszFormat, ...)
{
	va_list args;
	va_start(args, pszFormat);
	wchar_t wbuffer[4096];
	vswprintf(wbuffer, sizeof(wbuffer) / sizeof(wchar_t), pszFormat, args);
	va_end(args);
	std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
	std::string narrow = converter.to_bytes(wbuffer);

	GetInst()->_logger->error(narrow);

	if (GetInst()->_logger_redirect)
		GetInst()->_logger_redirect->error(narrow);
}

void CLog::Warn(const wchar_t* pszFormat, ...)
{
	va_list args;
	va_start(args, pszFormat);
	wchar_t wbuffer[4096];
	vswprintf(wbuffer, sizeof(wbuffer) / sizeof(wchar_t), pszFormat, args);
	va_end(args);
	std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
	std::string narrow = converter.to_bytes(wbuffer);

	GetInst()->_logger->warn(narrow);

	if (GetInst()->_logger_redirect)
		GetInst()->_logger_redirect->warn(narrow);
}

void CLog::Info(const wchar_t* pszFormat, ...)
{
	va_list args;
	va_start(args, pszFormat);
	wchar_t wbuffer[4096];
	vswprintf(wbuffer, sizeof(wbuffer) / sizeof(wchar_t), pszFormat, args);
	va_end(args);
	std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
	std::string narrow = converter.to_bytes(wbuffer);

	GetInst()->_logger->info(narrow);

	if (GetInst()->_logger_redirect)
		GetInst()->_logger_redirect->info(narrow);
}

void CLog::Debug(const wchar_t* pszFormat, ...)
{
	va_list args;
	va_start(args, pszFormat);
	wchar_t wbuffer[4096];
	vswprintf(wbuffer, sizeof(wbuffer) / sizeof(wchar_t), pszFormat, args);
	va_end(args);
	std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
	std::string narrow = converter.to_bytes(wbuffer);

	GetInst()->_logger->debug(narrow);

	if (GetInst()->_logger_redirect)
		GetInst()->_logger_redirect->debug(narrow);
}

void CLog::Log(Level severity, const wchar_t* pszFormat, ...)
{
	va_list args;
	va_start(args, pszFormat);
	wchar_t wbuffer[4096];
	vswprintf(wbuffer, sizeof(wbuffer) / sizeof(wchar_t), pszFormat, args);
	va_end(args);
	std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
	std::string narrow = converter.to_bytes(wbuffer);

	GetInst()->_logger->log((spdlog::level::level_enum)severity, narrow);

	if (GetInst()->_logger_redirect)
		GetInst()->_logger_redirect->log((spdlog::level::level_enum)severity, narrow);
}

void CLog::Log(Level severity, const std::wstring& format, ...)
{
	va_list args;
	va_start(args, format);
	wchar_t wbuffer[4096];
	vswprintf(wbuffer, sizeof(wbuffer) / sizeof(wchar_t), format.c_str(), args);
	va_end(args);
	std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
	std::string narrow = converter.to_bytes(wbuffer);

	GetInst()->_logger->log((spdlog::level::level_enum)severity, narrow);

	if (GetInst()->_logger_redirect)
		GetInst()->_logger_redirect->log((spdlog::level::level_enum)severity, narrow);
}

std::string CLog::CreateDefaultFilePath(const std::string& strLogFileName /*= ""*/)
{
	// 获取程序所在目录
	std::string exeDir = GetExeDirectory();
	std::string logDir = exeDir + "/Log/";

	// 创建目录（如果不存在）
	mkdir(logDir.c_str(), 0755);  // 忽略失败

	// 获取当前日期
	time_t now = time(nullptr);
	struct tm* t = localtime(&now);
	char dateBuf[64];
	strftime(dateBuf, sizeof(dateBuf), "%Y-%m-%d", t);

	std::string prefix = strLogFileName.empty() ? "SystemLog" : strLogFileName;
	std::string fullPath = logDir + prefix + "_" + dateBuf + ".txt";
	return fullPath;
}

void CLog::MemoryInfo(const std::string& strMsg)
{
	std::ifstream statm("/proc/self/statm");
	if (!statm.is_open()) {
		Info("MemoryInfo: cannot open /proc/self/statm");
		return;
	}
	long size, resident, share, text, lib, data, dt;
	statm >> size >> resident >> share >> text >> lib >> data >> dt;
	statm.close();

	long page_size = sysconf(_SC_PAGESIZE);
	long rss_kb = resident * page_size / 1024;
	Info("%s 当前内存使用：%ld KB", strMsg.c_str(), rss_kb);
}

void CLog::Flush()
{
	GetInst()->_logger->flush();
}

std::string CLog::GetExeDirectory()
{
	char buf[1024];
	ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
	if (len != -1) {
		buf[len] = '\0';
		std::string exePath(buf);
		size_t pos = exePath.find_last_of('/');
		if (pos != std::string::npos)
			return exePath.substr(0, pos);
	}
	// 默认返回当前目录
	return ".";
}

CTimeLog::CTimeLog(const char* pszFormat, ...)
{
	va_list args;
	va_start(args, pszFormat);
	char buffer[4096];
	vsnprintf(buffer, sizeof(buffer), pszFormat, args);
	va_end(args);
	_strTip = buffer;
	auto now = std::chrono::steady_clock::now();
	_nStartMs = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

CTimeLog::CTimeLog(const wchar_t* pszFormat, ...)
{
	va_list args;
	va_start(args, pszFormat);
	wchar_t wbuffer[4096];
	vswprintf(wbuffer, sizeof(wbuffer) / sizeof(wchar_t), pszFormat, args);
	va_end(args);
	std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
	std::string narrow = converter.to_bytes(wbuffer);
	_strTip = narrow.c_str();
	auto now = std::chrono::steady_clock::now();
	_nStartMs = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

CTimeLog::~CTimeLog()
{
	if (!_strTip.empty() && _nStartMs!=0)
	{
		auto now = std::chrono::steady_clock::now();
		long long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() - _nStartMs;
		// 注意：CLog::Info 应接受窄字符串
		std::string w1 = fmt::format("{}:{} 毫秒", _strTip, elapsed); //此处无法使用中文 使用则乱码 仅能使用英文ms 单位
		CLog::Info(w1.c_str());
	}
}
