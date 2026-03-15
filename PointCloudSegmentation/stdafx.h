#pragma once
#ifdef WIN32

#ifndef WINVER                          // 指定要求的最低平台是 Windows Vista。
#define WINVER 0x0600           // 将此值更改为相应的值，以适用于 Windows 的其他版本。
#endif

#ifndef _WIN32_WINNT            // 指定要求的最低平台是 Windows Vista。
#define _WIN32_WINNT 0x0600     // 将此值更改为相应的值，以适用于 Windows 的其他版本。
#endif

#ifndef _WIN32_WINDOWS          // 指定要求的最低平台是 Windows 98。
#define _WIN32_WINDOWS 0x0410 // 将此值更改为适当的值，以适用于 Windows Me 或更高版本。
#endif

#ifndef _WIN32_IE                       // 指定要求的最低平台是 Internet Explorer 7.0。
#define _WIN32_IE 0x0700        // 将此值更改为相应的值，以适用于 IE 的其他版本。
#endif
#endif

#include <stdio.h>
#include <string>
#include <list>
#include <memory>

#include <assert.h>

#ifdef WIN32
#pragma comment(lib, "Ole32.lib")
#pragma comment(lib, "OleAut32.lib")
#pragma comment(lib, "User32.lib")
#else
#define _wtol(x)      wcstol(x, NULL, 10)
#define _wtof(x)      wcstof(x, NULL)
#endif 

#define _USE_MATH_DEFINES
#define GOOGLE_GLOG_DLL_DECL

#define strtofloat(f)	wcstof(f.c_str(), NULL)
#define strtoint(f)		wcstol(f.c_str(), NULL,10)

#include "../CommonTools/include/include.h"






using namespace std;
