#ifndef COMMONTOOLSDEF_H_
#define COMMONTOOLSDEF_H_

#ifdef _WIN32
#include <windows.h>
#else
// Linux下的类型定义
typedef char CHAR;
typedef wchar_t WCHAR;
typedef unsigned char BYTE;
typedef long LONG;
typedef unsigned long DWORD;
typedef unsigned int UINT;
typedef WCHAR TCHAR;

#define __T(x) L##x
#define _T(x) __T(x)

// #define _T(x) L##x

#ifndef WIN32
#define _wtol(x) wcstol(x, NULL, 10)
#define _wtof(x) wcstof(x, NULL)
#endif

#define FALSE 0
#define TRUE 1

typedef unsigned short WORD;
typedef float FLOAT;
typedef FLOAT* PFLOAT;

#define far
#define near
typedef int far* LPINT;
typedef WORD near* PWORD;
typedef WORD far* LPWORD;
typedef long far* LPLONG;
typedef DWORD near* PDWORD;
typedef DWORD far* LPDWORD;
typedef void far* LPVOID;
typedef const void* LPCVOID;
typedef BYTE far* LPBYTE;

typedef unsigned long ULONG;
typedef ULONG* PULONG;
typedef unsigned short USHORT;
typedef USHORT* PUSHORT;
typedef unsigned char UCHAR;
typedef UCHAR* PUCHAR;
// typedef LPCWSTR LPCTSTR;
typedef int INT;
typedef unsigned int UINT;
typedef unsigned int* PUINT;

#ifndef byte
typedef unsigned char byte;
#endif

#ifndef LPCSTR
typedef const char* LPCSTR;
#endif

#ifndef LPCWSTR
typedef const wchar_t* LPCWSTR;
#endif

#ifndef LPWSTR
typedef wchar_t* LPWSTR;
#endif

#ifndef LPCTSTR
typedef const wchar_t* LPCTSTR;
#endif

// #define LPCWSTR const wchar_t*
// #define LPWSTR wchar_t*
// #define LPCSTR const char*
// #define LPSTR char*

// Linux下的GUID定义
typedef struct _GUID
{
	unsigned long Data1;
	unsigned short Data2;
	unsigned short Data3;
	unsigned char Data4[8];
} GUID;
#endif

//! 颜色类型枚举
enum eCustomColorType
{
	NullColor = 0,	 // 未设置颜色
	IndexEx = 1,	 /**<索引 */
	TrueColorEx = 2, /**<真彩 */
	BookEx = 3,		 /**<色簿 */
	Reverse = 4,	 /**<反转色 */
};

#define TZERO(x) (!((x) != 0))
#define TZERO2(x) (((x) < 0.01) && ((x) > -0.01))
#define TZERO4(x) (((x) < 0.0001) && ((x) > -0.0001))
#define TZERO6(x) (((x) < 0.000001) && ((x) > -0.000001))
#define TZERO8(x) (((x) < 0.00000001) && ((x) > -0.00000001))
#define TZERO12(x) (((x) < 0.000000000001) && ((x) > -0.000000000001))

#define FLOAT_EQUAL2(x, y) TZERO2((x) - (y))
#define FLOAT_EQUAL4(x, y) TZERO4((x) - (y))
#define FLOAT_EQUAL6(x, y) TZERO6((x) - (y))
#define FLOAT_EQUAL8(x, y) TZERO8((x) - (y))
#define FLOAT_EQUAL12(x, y) TZERO12((x) - (y))


#ifndef SAFE_DELETE
// 安全删除对象
#define SAFE_DELETE(x)                                                                             \
	if (NULL != x)                                                                                 \
	{                                                                                              \
		delete x;                                                                                  \
		x = NULL;                                                                                  \
	}
#endif // !SAFE_DELETE

#ifndef SAFE_DELETE_ARRAY
// 安全删除数组对象
#define SAFE_DELETE_ARRAY(x)                                                                       \
	if (NULL != x)                                                                                 \
	{                                                                                              \
		delete[] x;                                                                                \
		x = NULL;                                                                                  \
	}
#endif // !SAFE_DELETE_ARRAY


#include <vector>

struct ENUMDATA
{
	int value;
	LPCTSTR text;
	int data;
};
typedef std::vector<ENUMDATA> EnumTextDefines;

#define BEGIN_ENUM_CONVERSION(ENUMNAME)                                                            \
	static LPCTSTR ENUMNAME##2Text(ENUMNAME val);                                                  \
	static ENUMNAME Text2##ENUMNAME(LPCTSTR txt, int def = 0);                                     \
	static int Get##ENUMNAME##List(EnumTextDefines& arr);                                          \
	static ENUMDATA ENUMNAME##TextList[] =

#define END_ENUM_CONVERSION(ENUMNAME)                                                              \
	;                                                                                              \
	LPCTSTR ENUMNAME##2Text(ENUMNAME val)                                                          \
	{                                                                                              \
		for (int i = 0; ENUMNAME##TextList[i].text != NULL && i < 1000; i++)                       \
		{                                                                                          \
			if (ENUMNAME##TextList[i].value == val)                                                \
				return ENUMNAME##TextList[i].text;                                                 \
		}                                                                                          \
		return NULL;                                                                               \
	}                                                                                              \
	ENUMNAME Text2##ENUMNAME(LPCTSTR txt, int def)                                                 \
	{                                                                                              \
		if (txt == NULL)                                                                           \
			return (ENUMNAME)def;                                                                  \
		int i = 0;                                                                                 \
		for (i = 0; ENUMNAME##TextList[i].text != NULL && i < 1000; i++)                           \
		{                                                                                          \
			if (!wcscmp(ENUMNAME##TextList[i].text, txt))                                          \
				return (ENUMNAME)ENUMNAME##TextList[i].value;                                      \
		}                                                                                          \
		return (ENUMNAME)ENUMNAME##TextList[i].value;                                              \
	}                                                                                              \
	int Get##ENUMNAME##List(EnumTextDefines& arr)                                                  \
	{                                                                                              \
		arr.clear();                                                                               \
		for (int i = 0; ENUMNAME##TextList[i].text != NULL && i < 1000; i++)                       \
		{                                                                                          \
			arr.push_back(ENUMNAME##TextList[i]);                                                  \
		}                                                                                          \
		return arr.size();                                                                         \
	}


#endif // COMMONTOOLSDEF_H_
