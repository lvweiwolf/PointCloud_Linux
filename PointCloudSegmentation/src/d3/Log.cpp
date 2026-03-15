#include <src/d3/Log.h>

d3s::CLog::CLog() {}

d3s::CLog::~CLog() {}

void d3s::CLog::EnableRedirectTo(Sink sink) {}

void d3s::CLog::SetLevel(Level level) {}

bool d3s::CLog::ShouldLog(Level level) {}

void d3s::CLog::Error(const char* pszFormat, ...) {}

void d3s::CLog::Warn(const char* pszFormat, ...) {}

void d3s::CLog::Info(const char* pszFormat, ...) {}

void d3s::CLog::Debug(const char* pszFormat, ...) {}

void d3s::CLog::Log(Level severity, const char* pszFormat, ...) {}

void d3s::CLog::Log(Level severity, const std::string& format, ...) {}

void d3s::CLog::Log(Level severity, std::wstring format, ...) {}

void d3s::CLog::Log(Level severity, const std::wstring& format, ...) {}

std::string d3s::CLog::CreateDefaultFilePath(const std::string& strLogFileName /*= ""*/) {}

void d3s::CLog::MemoryInfo(const std::string& strMsg) {}

void d3s::CLog::Flush() {}

d3s::CTimeLog::CTimeLog(const char* pszFormat, ...) {}

d3s::CTimeLog::~CTimeLog() {}
