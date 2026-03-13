#include <include/cstring.h>
#include <cstring/fixalloc.h>

#include <cstdarg>
#include <cctype>
#include <cwctype>
#include <cerrno>
#include <memory>

#if defined(_WIN32)
#define vsnprintf _vsnprintf
#endif

static inline LPWSTR NextChar(LPCWSTR p);
static wchar_t* _cstrchr(const wchar_t* p, wchar_t ch);
static wchar_t* _cstrrchr(const wchar_t* p, wchar_t ch);
static wchar_t* _cstrrev(wchar_t* pStr);
static wchar_t* _cstrstr(const wchar_t* pStr, const wchar_t* pCharSet);

// Globals
// For an empty string, m_pchData will point here
// (note: avoids special case of checking for NULL m_pchData)
// empty string data (and locked)
static int rgInitData[] = { -1, 0, 0, 0 };
static CStringData* StrDataNil = (CStringData*)&rgInitData;
static LPCWSTR StrPchNil = (LPCWSTR)(((byte*)&rgInitData) + sizeof(CStringData));

#define ROUND(x, y) (((x) + (y - 1)) & ~(y - 1))
#define ROUND4(x) ROUND(x, 4)
static CFixedAlloc StrAlloc64(ROUND4(65 * sizeof(wchar_t) + sizeof(CStringData)));
static CFixedAlloc StrAlloc128(ROUND4(129 * sizeof(wchar_t) + sizeof(CStringData)));
static CFixedAlloc StrAlloc256(ROUND4(257 * sizeof(wchar_t) + sizeof(CStringData)));
static CFixedAlloc StrAlloc512(ROUND4(513 * sizeof(wchar_t) + sizeof(CStringData)));

#if defined(__GNUC__)
ptSpinLock CString::lockrefs;
#endif

static int vsnwprintf(wchar_t* buffer, size_t count, const wchar_t* format, va_list args)
{
	if (!buffer || count == 0)
	{
		// 计算所需长度
		// 转换为多字节，使用 vsnprintf 计算，再转换回来
		char formatMB[4096];
		size_t len = wcstombs(formatMB, format, sizeof(formatMB) - 1);
		if (len == (size_t)-1)
			return -1;
		formatMB[len] = '\0';

		// 使用窄字符版本计算长度
		va_list argsCopy;
		va_copy(argsCopy, args);
		int needed = vsnprintf(NULL, 0, formatMB, argsCopy);
		va_end(argsCopy);
		return needed;
	}

	// 转换为多字节格式字符串
	char formatMB[4096];
	size_t len = wcstombs(formatMB, format, sizeof(formatMB) - 1);
	if (len == (size_t)-1)
		return -1;
	formatMB[len] = '\0';

	// 处理参数
	va_list argsCopy;
	va_copy(argsCopy, args);

	// 分配多字节缓冲区
	char* bufferMB = new char[count * 4]; // 足够存放任何转换

	int result = vsnprintf(bufferMB, count * 4, formatMB, argsCopy);
	va_end(argsCopy);

	if (result >= 0)
	{
		bufferMB[result] = '\0';
		// 转换回宽字符
		size_t converted = mbstowcs(buffer, bufferMB, count);
		if (converted == (size_t)-1)
		{
			result = -1;
		}
	}

	delete[] bufferMB;
	return result;
}

const CString& CString::_GetEmptyString() { return *(CString*)&StrPchNil; }

CString::CString(const CString& stringSrc)
{
	assert(stringSrc.GetData()->nRefs != 0);
	if (stringSrc.GetData()->nRefs >= 0)
	{
		assert(stringSrc.GetData() != StrDataNil);
		m_pchData = stringSrc.m_pchData;
		safe_inc((int*)&GetData()->nRefs);
	}
	else
	{
		Init();
		*this = stringSrc.m_pchData;
	}
}

bool CString::AllocBuffer(int nLen)
// always allocate one extra character for '\0' termination
// assumes [optimistically] that data length will equal allocation length
{
	assert(nLen >= 0);
	assert(nLen <= INT_MAX - 1); // max size (enough room for 1 extra)

	if (nLen == 0)
	{
		Init();
	}
	else
	{
		CStringData* pData = NULL;

		if (nLen <= 64)
		{
			pData = (CStringData*)StrAlloc64.Alloc();
			pData->nAllocLength = 64;
		}
		else if (nLen <= 128)
		{
			pData = (CStringData*)StrAlloc128.Alloc();
			pData->nAllocLength = 128;
		}
		else if (nLen <= 256)
		{
			pData = (CStringData*)StrAlloc256.Alloc();
			pData->nAllocLength = 256;
		}
		else if (nLen <= 512)
		{
			pData = (CStringData*)StrAlloc512.Alloc();
			pData->nAllocLength = 512;
		}
		else
		{
			pData = (CStringData*)new byte[sizeof(CStringData) + (nLen + 1) * sizeof(wchar_t)];
			pData->nAllocLength = nLen;
		}

		pData->nRefs = 1;
		pData->data()[nLen] = '\0';
		pData->nDataLength = nLen;
		m_pchData = pData->data();
	}

	return true;
}

void CString::Release()
{
	if (GetData() != StrDataNil)
	{
		assert(GetData()->nRefs != 0);
		if (safe_dec((int*)&GetData()->nRefs) <= 0)
			FreeData(GetData());
		Init();
	}
}

void CString::Release(CStringData* pData)
{
	if (pData != StrDataNil)
	{
		assert(pData->nRefs != 0);
		if (safe_dec((int*)&pData->nRefs) <= 0)
			FreeData(pData);
	}
}

void CString::Empty()
{
	if (GetData()->nDataLength == 0)
		return;

	if (GetData()->nRefs >= 0)
		Release();
	else
		*this = L"";

	assert(GetData()->nDataLength == 0);
	assert(GetData()->nRefs < 0 || GetData()->nAllocLength == 0);
}

void CString::CopyBeforeWrite()
{
	if (GetData()->nRefs > 1)
	{
		CStringData* pData = GetData();
		Release();
		if (AllocBuffer(pData->nDataLength))
			memcpy(m_pchData, pData->data(), (pData->nDataLength + 1) * sizeof(wchar_t));
	}
	assert(GetData()->nRefs <= 1);
}

bool CString::AllocBeforeWrite(int nLen)
{
	bool bRet = true;
	if (GetData()->nRefs > 1 || nLen > GetData()->nAllocLength)
	{
		Release();
		bRet = AllocBuffer(nLen);
	}
	assert(GetData()->nRefs <= 1);
	return bRet;
}

const CString& CString::Append(int n)
{
	wchar_t szBuffer[10];
	swprintf(szBuffer, 10, L"%d", n);
	ConcatInPlace(SafeStrlen(szBuffer), szBuffer);
	return *this;
}

CString::~CString()
//  free any attached data
{
	if (GetData() != StrDataNil)
	{
		if (safe_dec((int*)&GetData()->nRefs) <= 0)
			FreeData(GetData());
	}
}

void CString::AllocCopy(CString& dest, int nCopyLen, int nCopyIndex, int nExtraLen) const
{
	// will clone the data attached to this string
	// allocating 'nExtraLen' characters
	// Places results in uninitialized string 'dest'
	// Will copy the part or all of original data to start of new string

	int nNewLen = nCopyLen + nExtraLen;
	if (nNewLen == 0)
	{
		dest.Init();
	}
	else
	{
		if (dest.AllocBuffer(nNewLen))
			memcpy(dest.m_pchData, m_pchData + nCopyIndex, nCopyLen * sizeof(wchar_t));
	}
}

CString::CString(LPCWSTR lpsz)
{
	Init();

	int nLen = SafeStrlen(lpsz);
	if (nLen != 0)
	{
		if (AllocBuffer(nLen))
			memcpy(m_pchData, lpsz, nLen * sizeof(wchar_t));
	}
}

// Assignment operators
//  All assign a new value to the string
//      (a) first see if the buffer is big enough
//      (b) if enough room, copy on top of old buffer, set size and type
//      (c) otherwise free old string data, and create a new one
//
//  All routines return the new string (but as a 'const CString&' so that
//      assigning it again will cause a copy, eg: s1 = s2 = "hi there".
//

void CString::AssignCopy(int nSrcLen, LPCWSTR lpszSrcData)
{
	if (AllocBeforeWrite(nSrcLen))
	{
		memcpy(m_pchData, lpszSrcData, nSrcLen * sizeof(wchar_t));
		GetData()->nDataLength = nSrcLen;
		m_pchData[nSrcLen] = '\0';
	}
}

const CString& CString::operator=(const CString& stringSrc)
{
	if (m_pchData != stringSrc.m_pchData)
	{
		if ((GetData()->nRefs < 0 && GetData() != StrDataNil) || stringSrc.GetData()->nRefs < 0)
		{
			// actual copy necessary since one of the strings is locked
			AssignCopy(stringSrc.GetData()->nDataLength, stringSrc.m_pchData);
		}
		else
		{
			// can just copy references around
			Release();
			assert(stringSrc.GetData() != StrDataNil);
			m_pchData = stringSrc.m_pchData;
			safe_inc((int*)&GetData()->nRefs);
		}
	}
	return *this;
}

const CString& CString::operator=(LPCWSTR lpsz)
{
	assert(lpsz != NULL);
	AssignCopy(SafeStrlen(lpsz), lpsz);
	return *this;
}

// Concatenation
// NOTE: "operator+" is done as friend functions for simplicity
//      There are three variants:
//          CString + CString
// and for ? = wchar_t, LPCWSTR
//          CString + ?
//          ? + CString

bool CString::ConcatCopy(int nSrc1Len, LPCWSTR lpszSrc1Data, int nSrc2Len, LPCWSTR lpszSrc2Data)
{
	// -- master concatenation routine
	// Concatenate two sources
	// -- assume that 'this' is a new CString object

	bool bRet = true;
	int nNewLen = nSrc1Len + nSrc2Len;
	if (nNewLen != 0)
	{
		bRet = AllocBuffer(nNewLen);
		if (bRet)
		{
			memcpy(m_pchData, lpszSrc1Data, nSrc1Len * sizeof(wchar_t));
			memcpy(m_pchData + nSrc1Len, lpszSrc2Data, nSrc2Len * sizeof(wchar_t));
		}
	}
	return bRet;
}

CString operator+(const CString& string1, const CString& string2)
{
	CString s;
	s.ConcatCopy(string1.GetData()->nDataLength,
				 string1.m_pchData,
				 string2.GetData()->nDataLength,
				 string2.m_pchData);
	return s;
}

CString operator+(const CString& string, LPCWSTR lpsz)
{
	assert(lpsz != NULL);
	CString s;
	s.ConcatCopy(string.GetData()->nDataLength, string.m_pchData, CString::SafeStrlen(lpsz), lpsz);
	return s;
}

CString operator+(LPCWSTR lpsz, const CString& string)
{
	assert(lpsz != NULL);
	CString s;
	s.ConcatCopy(CString::SafeStrlen(lpsz), lpsz, string.GetData()->nDataLength, string.m_pchData);
	return s;
}

void CString::ConcatInPlace(int nSrcLen, LPCWSTR lpszSrcData)
{
	//  -- the main routine for += operators

	// concatenating an empty string is a no-op!
	if (nSrcLen == 0)
		return;

	// if the buffer is too small, or we have a width mis-match, just
	//   allocate a new buffer (slow but sure)
	if (GetData()->nRefs > 1 || GetData()->nDataLength + nSrcLen > GetData()->nAllocLength)
	{
		// we have to grow the buffer, use the ConcatCopy routine
		CStringData* pOldData = GetData();
		if (ConcatCopy(GetData()->nDataLength, m_pchData, nSrcLen, lpszSrcData))
		{
			assert(pOldData != NULL);
			CString::Release(pOldData);
		}
	}
	else
	{
		// fast concatenation when buffer big enough
		memcpy(m_pchData + GetData()->nDataLength, lpszSrcData, nSrcLen * sizeof(wchar_t));
		GetData()->nDataLength += nSrcLen;
		assert(GetData()->nDataLength <= GetData()->nAllocLength);
		m_pchData[GetData()->nDataLength] = '\0';
	}
}

const CString& CString::operator+=(LPCWSTR lpsz)
{
	assert(lpsz != NULL);
	ConcatInPlace(SafeStrlen(lpsz), lpsz);
	return *this;
}

const CString& CString::operator+=(wchar_t ch)
{
	ConcatInPlace(1, &ch);
	return *this;
}

const CString& CString::operator+=(const CString& string)
{
	ConcatInPlace(string.GetData()->nDataLength, string.m_pchData);
	return *this;
}

LPWSTR CString::GetBuffer()
{
	if (GetData()->nRefs > 1)
	{
		// we have to grow the buffer
		CStringData* pOldData = GetData();
		int nOldLen = GetData()->nDataLength; // AllocBuffer will tromp it

		if (!AllocBuffer(nOldLen))
			return NULL;

		memcpy(m_pchData, pOldData->data(), (nOldLen + 1) * sizeof(wchar_t));
		GetData()->nDataLength = nOldLen;
		CString::Release(pOldData);
	}
	assert(GetData()->nRefs <= 1);

	// return a pointer to the character storage for this string
	assert(m_pchData != NULL);
	return m_pchData;
}

void CString::ReleaseBuffer(int nNewLength)
{
	CopyBeforeWrite(); // just in case GetBuffer was not called

	if (nNewLength == -1)
		nNewLength = wcslen(m_pchData); // zero terminated

	assert(nNewLength <= GetData()->nAllocLength);
	GetData()->nDataLength = nNewLength;
	m_pchData[nNewLength] = '\0';
}

void CString::FreeExtra()
{
	assert(GetData()->nDataLength <= GetData()->nAllocLength);
	if (GetData()->nDataLength != GetData()->nAllocLength)
	{
		CStringData* pOldData = GetData();
		if (AllocBuffer(GetData()->nDataLength))
		{
			memcpy(m_pchData, pOldData->data(), pOldData->nDataLength * sizeof(wchar_t));
			assert(m_pchData[GetData()->nDataLength] == '\0');
			CString::Release(pOldData);
		}
	}
	assert(GetData() != NULL);
}

LPWSTR CString::LockBuffer()
{
	LPWSTR lpsz = GetBuffer();
	if (lpsz != NULL)
		GetData()->nRefs = -1;
	return lpsz;
}

void CString::UnlockBuffer()
{
	assert(GetData()->nRefs == -1);
	if (GetData() != StrDataNil)
		GetData()->nRefs = 1;
}

int CString::Find(wchar_t ch) const { return Find(ch, 0); }

int CString::Find(wchar_t ch, int nStart) const
{
	int nLength = GetData()->nDataLength;
	if (nStart >= nLength)
		return -1;

	// find first single character
	LPWSTR lpsz = _cstrchr(m_pchData + nStart, (wchar_t)ch);

	// return -1 if not found and index otherwise
	return (lpsz == NULL) ? -1 : (int)(lpsz - m_pchData);
}

int CString::FindOneOf(LPCWSTR lpszCharSet) const
{
	assert(lpszCharSet != NULL);
	LPWSTR lpsz = wcspbrk(m_pchData, lpszCharSet);
	return (lpsz == NULL) ? -1 : (int)(lpsz - m_pchData);
}

CString& CString::MakeUpper()
{
	CopyBeforeWrite();
#if defined(_WIN32)
	_wcsupr(m_pchData);
#else
	LPWSTR p = m_pchData;
	while (*p != '\0')
	{
		*p = toupper(*p);
		p = NextChar(p);
	}
#endif
	return *this;
}

CString& CString::MakeLower()
{
	CopyBeforeWrite();
#if defined(_WIN32)
	_wcslwr(m_pchData);
#else
	LPWSTR p = m_pchData;
	while (*p != '\0')
	{
		*p = tolower(*p);
		p = NextChar(p);
	}
#endif
	return *this;
}

void CString::MakeReverse()
{
	CopyBeforeWrite();
	_cstrrev(m_pchData);
}

CString& CString::Trim()
{
	TrimLeft();
	TrimRight();
	return *this;
}

void CString::SetAt(int nIndex, wchar_t ch)
{
	assert(nIndex >= 0);
	assert(nIndex < GetData()->nDataLength);

	CopyBeforeWrite();
	m_pchData[nIndex] = ch;
}

CString::CString(wchar_t ch, int nLength)
{
	Init();
	if (nLength >= 1)
	{
		if (AllocBuffer(nLength))
			memset(m_pchData, ch, nLength);
	}
}

CString::CString(LPCWSTR lpch, int nLength)
{
	Init();
	if (nLength != 0)
	{
		if (AllocBuffer(nLength))
			memcpy(m_pchData, lpch, nLength * sizeof(wchar_t));
	}
}

const CString& CString::operator=(wchar_t ch)
{
	AssignCopy(1, &ch);
	return *this;
}

CString operator+(const CString& string1, wchar_t ch)
{
	CString s;
	s.ConcatCopy(string1.GetData()->nDataLength, string1.m_pchData, 1, &ch);
	return s;
}

CString operator+(wchar_t ch, const CString& string)
{
	CString s;
	s.ConcatCopy(1, &ch, string.GetData()->nDataLength, string.m_pchData);
	return s;
}

CString CString::Mid(int nFirst) const { return Mid(nFirst, GetData()->nDataLength - nFirst); }

CString CString::Mid(int nFirst, int nCount) const
{
	// out-of-bounds requests return sensible things
	if (nFirst < 0)
		nFirst = 0;
	if (nCount < 0)
		nCount = 0;

	if (nFirst + nCount > GetData()->nDataLength)
		nCount = GetData()->nDataLength - nFirst;
	if (nFirst > GetData()->nDataLength)
		nCount = 0;

	CString dest;
	AllocCopy(dest, nCount, nFirst, 0);
	return dest;
}

CString CString::Right(int nCount) const
{
	if (nCount < 0)
		nCount = 0;
	else if (nCount > GetData()->nDataLength)
		nCount = GetData()->nDataLength;

	CString dest;
	AllocCopy(dest, nCount, GetData()->nDataLength - nCount, 0);
	return dest;
}

CString CString::Left(int nCount) const
{
	if (nCount < 0)
		nCount = 0;
	else if (nCount > GetData()->nDataLength)
		nCount = GetData()->nDataLength;

	CString dest;
	AllocCopy(dest, nCount, 0, 0);
	return dest;
}

// strspn equivalent
CString CString::SpanIncluding(LPCWSTR lpszCharSet) const
{
	assert(lpszCharSet != NULL);
	return Left(wcsspn(m_pchData, lpszCharSet));
}

// strcspn equivalent
CString CString::SpanExcluding(LPCWSTR lpszCharSet) const
{
	assert(lpszCharSet != NULL);
	return Left(wcsspn(m_pchData, lpszCharSet));
}

int CString::ReverseFind(wchar_t ch) const
{
	// find last single character
	LPWSTR lpsz = _cstrrchr(m_pchData, (wchar_t)ch);

	// return -1 if not found, distance from beginning otherwise
	return (lpsz == NULL) ? -1 : (int)(lpsz - m_pchData);
}

// find a sub-string (like strstr)
int CString::Find(LPCWSTR lpszSub) const { return Find(lpszSub, 0); }

int CString::Find(LPCWSTR lpszSub, int nStart) const
{
	assert(lpszSub != NULL);

	int nLength = GetData()->nDataLength;
	if (nStart > nLength)
		return -1;

	// find first matching substring
	LPWSTR lpsz = _cstrstr(m_pchData + nStart, lpszSub);

	// return -1 for not found, distance from beginning otherwise
	return (lpsz == NULL) ? -1 : (int)(lpsz - m_pchData);
}

int CString::Format(LPCWSTR lpszFormat, ...)
{
	if (lpszFormat == NULL)
	{
		Empty();
		return -1;
	}

	va_list args;
	va_start(args, lpszFormat);

	int result = FormatV(lpszFormat, args);

	va_end(args);
	return result;
}

int CString::FormatV(LPCWSTR lpszFormat, va_list args)
{
	// 使用动态缓冲区
	const size_t INITIAL_SIZE = 256;
	std::vector<wchar_t> buffer(INITIAL_SIZE);
	int result = -1;

	while (true)
	{
		// 复制参数列表（因为每次调用都会消耗参数）
		va_list argsCopy;
		va_copy(argsCopy, args);

		// 尝试格式化
		result = vswprintf(buffer.data(), buffer.size(), lpszFormat, argsCopy);
		va_end(argsCopy);

		if (result < 0)
		{
			// 在 Linux 上，如果缓冲区不够，vswprintf 返回 -1
			// 我们需要检查 errno 来确定是否是缓冲区太小
			if (errno == 0 || buffer.size() >= 65536)
			{
				// 真正的错误或缓冲区已经太大
				break;
			}

			// 增加缓冲区大小并重试
			buffer.resize(buffer.size() * 2);
		}
		else if (static_cast<size_t>(result) >= buffer.size())
		{
			// 缓冲区刚好或几乎满了，需要更大的缓冲区
			buffer.resize(result + 1);
		}
		else
		{
			// 成功
			buffer[result] = L'\0';
			*this = buffer.data();
			return result;
		}
	}

	// 如果上面的方法失败，尝试使用 C++11 的方法
	return FormatModern(lpszFormat, args);
}

int CString::FormatModern(LPCWSTR lpszFormat, va_list args)
{
	// 使用 vsnwprintf 的 Linux 兼容版本
	va_list argsCopy;
	va_copy(argsCopy, args);

	// 首先计算需要的长度
	int needed = vsnwprintf(NULL, 0, lpszFormat, argsCopy);
	va_end(argsCopy);

	if (needed < 0)
	{
		Empty();
		return -1;
	}

	// 分配缓冲区
	std::unique_ptr<wchar_t[]> buffer(new wchar_t[needed + 1]);

	va_copy(argsCopy, args);
	int result = vsnwprintf(buffer.get(), needed + 1, lpszFormat, argsCopy);
	va_end(argsCopy);

	if (result >= 0)
	{
		buffer[result] = L'\0';
		*this = buffer.get();
	}
	else
	{
		Empty();
	}

	return result;
}

//// formatting (using wsprintf style formatting)
// int CString::Format(LPCWSTR lpszFormat, ...)
//{
//     assert(lpszFormat != NULL);
//     wchar_t buff[4096];
//
//     va_list argList;
//     va_start(argList, lpszFormat);
//     int bRet = vswprintf((LPWSTR)buff, sizeof(buff) -1, lpszFormat, argList);
//     int bRet = vswprintf((LPWSTR)buff, sizeof(buff) / sizeof(buff[0]) - 1,
//     lpszFormat, argList); assert(bRet>=0); if( bRet >= 0 ) {
//         buff[bRet] = '\0';
//     }
//     va_end(argList);
//     *this = buff;
//     return bRet;
// }

void CString::TrimRight()
{
	CopyBeforeWrite();

	// find beginning of trailing spaces by starting at beginning (DBCS aware)
	LPWSTR lpsz = m_pchData;
	LPWSTR lpszLast = NULL;
	while (*lpsz != '\0')
	{
		if (isspace(*lpsz))
		{
			if (lpszLast == NULL)
				lpszLast = lpsz;
		}
		else
		{
			lpszLast = NULL;
		}
		lpsz = (LPWSTR)NextChar(lpsz);
	}

	if (lpszLast != NULL)
	{
		// truncate at trailing space start
		*lpszLast = '\0';
		GetData()->nDataLength = (int)(lpszLast - m_pchData);
	}
}

void CString::TrimLeft()
{
	CopyBeforeWrite();

	// find first non-space character
	LPCWSTR lpsz = m_pchData;
	while (iswspace(*lpsz))
		lpsz = NextChar(lpsz);

	// fix up data and length
	int nDataLength = GetData()->nDataLength - (int)(lpsz - m_pchData);
	memmove(m_pchData, lpsz, (nDataLength + 1) * sizeof(wchar_t));
	GetData()->nDataLength = nDataLength;
}

void CString::TrimRight(LPCWSTR lpszTargetList)
{
	// find beginning of trailing matches
	// by starting at beginning (DBCS aware)

	CopyBeforeWrite();
	LPWSTR lpsz = m_pchData;
	LPWSTR lpszLast = NULL;

	while (*lpsz != '\0')
	{
		if (_cstrchr(lpszTargetList, *lpsz) != NULL)
		{
			if (lpszLast == NULL)
				lpszLast = lpsz;
		}
		else
			lpszLast = NULL;
		lpsz = NextChar(lpsz);
	}

	if (lpszLast != NULL)
	{
		// truncate at left-most matching character
		*lpszLast = '\0';
		GetData()->nDataLength = (int)(lpszLast - m_pchData);
	}
}

void CString::TrimRight(wchar_t chTarget)
{
	// find beginning of trailing matches
	// by starting at beginning (DBCS aware)

	CopyBeforeWrite();
	LPWSTR lpsz = m_pchData;
	LPWSTR lpszLast = NULL;

	while (*lpsz != '\0')
	{
		if (*lpsz == chTarget)
		{
			if (lpszLast == NULL)
				lpszLast = lpsz;
		}
		else
			lpszLast = NULL;
		lpsz = NextChar(lpsz);
	}

	if (lpszLast != NULL)
	{
		// truncate at left-most matching character
		*lpszLast = '\0';
		GetData()->nDataLength = (int)(lpszLast - m_pchData);
	}
}

void CString::TrimLeft(LPCWSTR lpszTargets)
{
	// if we're not trimming anything, we're not doing any work
	if (SafeStrlen(lpszTargets) == 0)
		return;

	CopyBeforeWrite();
	LPCWSTR lpsz = m_pchData;

	while (*lpsz != '\0')
	{
		if (_cstrchr(lpszTargets, *lpsz) == NULL)
			break;
		lpsz = NextChar(lpsz);
	}

	if (lpsz != m_pchData)
	{
		// fix up data and length
		int nDataLength = GetData()->nDataLength - (int)(lpsz - m_pchData);
		memmove(m_pchData, lpsz, (nDataLength + 1) * sizeof(wchar_t));
		GetData()->nDataLength = nDataLength;
	}
}

void CString::TrimLeft(wchar_t chTarget)
{
	// find first non-matching character

	CopyBeforeWrite();
	LPCWSTR lpsz = m_pchData;

	while (chTarget == *lpsz)
		lpsz = NextChar(lpsz);

	if (lpsz != m_pchData)
	{
		// fix up data and length
		int nDataLength = GetData()->nDataLength - (int)(lpsz - m_pchData);
		memmove(m_pchData, lpsz, (nDataLength + 1) * sizeof(wchar_t));
		GetData()->nDataLength = nDataLength;
	}
}

int CString::Delete(int nIndex, int nCount /* = 1 */)
{
	if (nIndex < 0)
		nIndex = 0;
	int nNewLength = GetData()->nDataLength;
	if (nCount > 0 && nIndex < nNewLength)
	{
		CopyBeforeWrite();
		int nbytesToCopy = nNewLength - (nIndex + nCount) + 1;

		memmove(m_pchData + nIndex, m_pchData + nIndex + nCount, nbytesToCopy * sizeof(wchar_t));
		GetData()->nDataLength = nNewLength - nCount;
	}

	return nNewLength;
}

int CString::Insert(int nIndex, wchar_t ch)
{
	CopyBeforeWrite();

	if (nIndex < 0)
		nIndex = 0;

	int nNewLength = GetData()->nDataLength;
	if (nIndex > nNewLength)
		nIndex = nNewLength;
	nNewLength++;

	if (GetData()->nAllocLength < nNewLength)
	{
		CStringData* pOldData = GetData();
		LPWSTR pstr = m_pchData;
		if (!AllocBuffer(nNewLength))
			return -1;
		memcpy(m_pchData, pstr, (pOldData->nDataLength + 1) * sizeof(wchar_t));
		CString::Release(pOldData);
	}

	// move existing bytes down
	memmove(m_pchData + nIndex + 1, m_pchData + nIndex, (nNewLength - nIndex) * sizeof(wchar_t));
	m_pchData[nIndex] = ch;
	GetData()->nDataLength = nNewLength;

	return nNewLength;
}

int CString::Insert(int nIndex, LPCWSTR pstr)
{
	if (nIndex < 0)
		nIndex = 0;

	int nInsertLength = SafeStrlen(pstr);
	int nNewLength = GetData()->nDataLength;
	if (nInsertLength > 0)
	{
		CopyBeforeWrite();
		if (nIndex > nNewLength)
			nIndex = nNewLength;
		nNewLength += nInsertLength;

		if (GetData()->nAllocLength < nNewLength)
		{
			CStringData* pOldData = GetData();
			LPWSTR pstr = m_pchData;
			if (!AllocBuffer(nNewLength))
				return -1;
			memcpy(m_pchData, pstr, (pOldData->nDataLength + 1) * sizeof(wchar_t));
			CString::Release(pOldData);
		}

		// move existing bytes down
		memmove(m_pchData + nIndex + nInsertLength,
				m_pchData + nIndex,
				(nNewLength - nIndex - nInsertLength + 1) * sizeof(wchar_t));
		memcpy(m_pchData + nIndex, pstr, nInsertLength * sizeof(wchar_t));
		GetData()->nDataLength = nNewLength;
	}

	return nNewLength;
}

int CString::Replace(wchar_t chOld, wchar_t chNew)
{
	int nCount = 0;

	// short-circuit the nop case
	if (chOld != chNew)
	{
		// otherwise modify each character that matches in the string
		CopyBeforeWrite();
		LPWSTR psz = m_pchData;
		LPWSTR pszEnd = psz + GetData()->nDataLength;
		while (psz < pszEnd)
		{
			// replace instances of the specified character only
			if (*psz == chOld)
			{
				*psz = chNew;
				nCount++;
			}
			psz = NextChar(psz);
		}
	}
	return nCount;
}

int CString::Replace(LPCWSTR lpszOld, LPCWSTR lpszNew)
{
	// can't have empty or NULL lpszOld

	int nSourceLen = SafeStrlen(lpszOld);
	if (nSourceLen == 0)
		return 0;
	int nReplacementLen = SafeStrlen(lpszNew);

	// loop once to figure out the size of the result string
	int nCount = 0;
	LPWSTR lpszStart = m_pchData;
	LPWSTR lpszEnd = m_pchData + GetData()->nDataLength;
	LPWSTR lpszTarget;
	while (lpszStart < lpszEnd)
	{
		while ((lpszTarget = _cstrstr(lpszStart, lpszOld)) != NULL)
		{
			nCount++;
			lpszStart = lpszTarget + nSourceLen;
		}
		lpszStart += wcslen(lpszStart) + 1;
	}

	// if any changes were made, make them
	if (nCount > 0)
	{
		CopyBeforeWrite();

		// if the buffer is too small, just
		//   allocate a new buffer (slow but sure)
		int nOldLength = GetData()->nDataLength;
		int nNewLength = nOldLength + (nReplacementLen - nSourceLen) * nCount;
		if (GetData()->nAllocLength < nNewLength || GetData()->nRefs > 1)
		{
			CStringData* pOldData = GetData();
			LPWSTR pstr = m_pchData;
			if (!AllocBuffer(nNewLength))
				return -1;
			memcpy(m_pchData, pstr, pOldData->nDataLength * sizeof(wchar_t));
			CString::Release(pOldData);
		}
		// else, we just do it in-place
		lpszStart = m_pchData;
		lpszEnd = m_pchData + GetData()->nDataLength;

		// loop again to actually do the work
		while (lpszStart < lpszEnd)
		{
			while ((lpszTarget = _cstrstr(lpszStart, lpszOld)) != NULL)
			{
				int nBalance = nOldLength - ((int)(lpszTarget - m_pchData) + nSourceLen);
				memmove(lpszTarget + nReplacementLen,
						lpszTarget + nSourceLen,
						nBalance * sizeof(wchar_t));
				memcpy(lpszTarget, lpszNew, nReplacementLen * sizeof(wchar_t));
				lpszStart = lpszTarget + nReplacementLen;
				lpszStart[nBalance] = '\0';
				nOldLength += (nReplacementLen - nSourceLen);
			}
			lpszStart += wcslen(lpszStart) + 1;
		}
		assert(m_pchData[nNewLength] == '\0');
		GetData()->nDataLength = nNewLength;
	}

	return nCount;
}

int CString::Remove(wchar_t chRemove)
{
	CopyBeforeWrite();

	LPWSTR pstrSource = m_pchData;
	LPWSTR pstrDest = m_pchData;
	LPWSTR pstrEnd = m_pchData + GetData()->nDataLength;

	while (pstrSource < pstrEnd)
	{
		if (*pstrSource != chRemove)
		{
			*pstrDest = *pstrSource;
			pstrDest = NextChar(pstrDest);
		}
		pstrSource = NextChar(pstrSource);
	}
	*pstrDest = '\0';
	int nCount = (int)(pstrSource - pstrDest);
	GetData()->nDataLength -= nCount;

	return nCount;
}

void CString::FreeData(CStringData* pData)
{
	int nLen = pData->nAllocLength;
	if (nLen == 64)
		StrAlloc64.Free(pData);
	else if (nLen == 128)
		StrAlloc128.Free(pData);
	else if (nLen == 256)
		StrAlloc256.Free(pData);
	else if (nLen == 512)
		StrAlloc512.Free(pData);
	else if (nLen > 512)
	{
		assert(nLen > 512);
		delete[] (byte*)pData;
	}
	else
	{
	}
}

wchar_t* _cstrchr(const wchar_t* p, wchar_t ch)
{
	// strchr for '\0' should succeed
	while (*p != 0)
	{
		if (*p == ch)
			break;
		p = NextChar(p);
	}
	return (wchar_t*)((*p == ch) ? p : NULL);
}
wchar_t* _cstrrchr(const wchar_t* p, wchar_t ch)
{
	const wchar_t* lpsz = NULL;
	while (*p != 0)
	{
		if (*p == ch)
			lpsz = p;
		p = NextChar(p);
	}
	return (wchar_t*)lpsz;
}
wchar_t* _cstrrev(wchar_t* pStr)
{
	// Optimize NULL, zero-length, and single-wchar_t case.
	if ((pStr == NULL) || (pStr[0] == '\0') || (pStr[1] == '\0'))
		return pStr;

	wchar_t* p = pStr;

	while (p[1] != 0)
	{
		wchar_t* pNext = (wchar_t*)NextChar(p);
		if (pNext > p + 1)
		{
			wchar_t p1 = *(wchar_t*)p;
			*(wchar_t*)p = *(wchar_t*)(p + 1);
			*(wchar_t*)(p + 1) = p1;
		}
		p = pNext;
	}

	wchar_t* q = pStr;

	while (q < p)
	{
		wchar_t t = *q;
		*q = *p;
		*p = t;
		q++;
		p--;
	}
	return (wchar_t*)pStr;
}
wchar_t* _cstrstr(const wchar_t* pStr, const wchar_t* pCharSet)
{
	int nLen = wcslen(pCharSet);
	if (nLen == 0)
		return (wchar_t*)pStr;

	const wchar_t* pRet = NULL;
	const wchar_t* pCur = pStr;
	while ((pStr = _cstrchr(pCur, *pCharSet)) != NULL)
	{
		if (memcmp(pCur, pCharSet, nLen * sizeof(wchar_t)) == 0)
		{
			pRet = pCur;
			break;
		}
		pCur = NextChar(pCur);
	}
	return (wchar_t*)pRet;
}
// 指向下一个字符，对于多字节来说，指针可能要加两次
inline LPWSTR NextChar(LPCWSTR p)
{
	if ((wchar_t)*p == 0)
		return (LPWSTR)p;
	else // 多字节字符
		return (LPWSTR)(p + 1);
}

// 实现辅助方法
void CString::Truncate(int nNewLength)
{
	CopyBeforeWrite();
	if (nNewLength < 0)
		nNewLength = 0;

	if (nNewLength > GetData()->nDataLength)
		return;

	GetData()->nDataLength = nNewLength;
	m_pchData[nNewLength] = L'\0';
}

LPWSTR CString::GetBuffer(int nMinBufLength)
{
	if (nMinBufLength == -1)
		return GetBuffer(); // 调用现有的无参版本

	if (nMinBufLength > GetData()->nAllocLength)
	{
		// 需要重新分配缓冲区
		CStringData* pOldData = GetData();
		if (!AllocBuffer(nMinBufLength))
			return NULL;

		if (pOldData->nDataLength > 0)
		{
			memcpy(m_pchData, pOldData->data(), pOldData->nDataLength * sizeof(wchar_t));
		}
		m_pchData[GetData()->nDataLength] = L'\0';
		CString::Release(pOldData);
	}

	assert(GetData()->nRefs <= 1);
	return m_pchData;
}

void CString::ReleaseBufferSetLength(int nNewLength)
{
	CopyBeforeWrite();

	if (nNewLength < 0)
		nNewLength = wcslen(m_pchData);

	assert(nNewLength <= GetData()->nAllocLength);
	GetData()->nDataLength = nNewLength;
	m_pchData[nNewLength] = L'\0';
}

// 实现 Trim 方法
CString& CString::TrimW(wchar_t chTarget) { return (TrimRightW(chTarget).TrimLeftW(chTarget)); }

CString& CString::TrimRightW(wchar_t chTarget)
{
	// 查找尾部的连续匹配字符
	// 从开头开始遍历（支持多字节字符）
	CopyBeforeWrite();

	LPWSTR psz = m_pchData;
	LPWSTR pszLast = NULL;

	while (*psz != L'\0')
	{
		if (*psz == chTarget)
		{
			if (pszLast == NULL)
			{
				pszLast = psz;
			}
		}
		else
		{
			pszLast = NULL;
		}
		// 对于宽字符，下一个字符就是指针加1
		psz++;
	}

	if (pszLast != NULL)
	{
		// 在最后一个匹配字符处截断
		int iLast = (int)(pszLast - m_pchData);
		Truncate(iLast);
	}

	return *this;
}

CString& CString::TrimLeftW(wchar_t chTarget)
{
	// 查找第一个不匹配的字符
	CopyBeforeWrite();

	LPCWSTR psz = m_pchData;

	while (chTarget == *psz)
	{
		psz++;
	}

	if (psz != m_pchData)
	{
		// 调整数据和长度
		int iFirst = (int)(psz - m_pchData);
		LPWSTR pszBuffer = GetBuffer(GetLength());
		psz = pszBuffer + iFirst;
		int nDataLength = GetLength() - iFirst;

		// 移动内存
		memmove(pszBuffer, psz, (nDataLength + 1) * sizeof(wchar_t));
		ReleaseBufferSetLength(nDataLength);
	}

	return *this;
}
