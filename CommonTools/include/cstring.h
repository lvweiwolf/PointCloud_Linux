#ifndef __CSTRING_H__
#define __CSTRING_H__

#include <include/CommonToolsExport.h>
#include <include/CommonToolsDef.h>

#include <cwchar>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <limits.h>
#include <string.h>


#if defined(_WIN32)
#include <windows.h>
#define vsnprintf _vsnprintf
#elif defined(__GNUC__)
#include "ptspinlock.h"
#endif

// #define INT_MAX 2147483647


// #ifndef
// typedef
// #endif

/////////////////////////////////////////////////////////////////////////////
// CString - String class

struct CStringData
{
	volatile long nRefs; // reference count
	int nDataLength;
	int nAllocLength;
	// wchar_t data[nAllocLength]

	wchar_t* data() { return (wchar_t*)(this + 1); }
};


class COMMONTOOLS_EXPORT CString
{
public:
	// Constructors
	CString();
	CString(const CString& stringSrc);
	CString(wchar_t ch, int nRepeat = 1);
	CString(LPCWSTR lpsz);
	CString(LPCWSTR lpch, int nLength);

	// Attributes & Operations
	// as an array of characters
	int GetLength() const;
	bool IsEmpty() const;
	void Empty(); // free up the data

	wchar_t GetAt(int nIndex) const;	  // 0 based
	wchar_t operator[](int nIndex) const; // same as GetAt
	void SetAt(int nIndex, wchar_t ch);
	operator LPCWSTR() const; // as a C string

	// overloaded assignment
	const CString& operator=(const CString& stringSrc);
	const CString& operator=(wchar_t ch);
	const CString& operator=(LPCWSTR lpsz);

	// string concatenation
	const CString& operator+=(const CString& string);
	const CString& operator+=(wchar_t ch);
	const CString& operator+=(LPCWSTR lpsz);

	friend CString operator+(const CString& string1, const CString& string2);
	friend CString operator+(const CString& string, wchar_t ch);
	friend CString operator+(wchar_t ch, const CString& string);
	friend CString operator+(const CString& string, LPCWSTR lpsz);
	friend CString operator+(LPCWSTR lpsz, const CString& string);

	// string comparison
	int Compare(LPCWSTR lpsz) const;	   // straight character
	int CompareNoCase(LPCWSTR lpsz) const; // ignore case

	// simple sub-string extraction
	CString Mid(int nFirst, int nCount) const;
	CString Mid(int nFirst) const;
	CString Left(int nCount) const;
	CString Right(int nCount) const;

	CString SpanIncluding(LPCWSTR lpszCharSet) const;
	CString SpanExcluding(LPCWSTR lpszCharSet) const;

	// upper/lower/reverse conversion
	CString& MakeUpper();
	CString& MakeLower();
	void MakeReverse();

	CString& TrimW(wchar_t chTarget);
	CString& TrimRightW(wchar_t chTarget);
	CString& TrimLeftW(wchar_t chTarget);

	// trimming whitespace (either side)
	CString& Trim();
	void TrimRight();
	void TrimLeft();

	// remove continuous occurrences of chTarget starting from right
	void TrimRight(wchar_t chTarget);
	// remove continuous occcurrences of characters in passed string,
	// starting from right
	void TrimRight(LPCWSTR lpszTargets);
	// remove continuous occurrences of chTarget starting from left
	void TrimLeft(wchar_t chTarget);
	// remove continuous occcurrences of characters in
	// passed string, starting from left
	void TrimLeft(LPCWSTR lpszTargets);

	// advanced manipulation
	// replace occurrences of chOld with chNew
	int Replace(wchar_t chOld, wchar_t chNew);
	// replace occurrences of substring lpszOld with lpszNew;
	// empty lpszNew removes instances of lpszOld
	int Replace(LPCWSTR lpszOld, LPCWSTR lpszNew);
	// remove occurrences of chRemove
	int Remove(wchar_t chRemove);
	// insert character at zero-based index; concatenates
	// if index is past end of string
	int Insert(int nIndex, wchar_t ch);
	// insert substring at zero-based index; concatenates
	// if index is past end of string
	int Insert(int nIndex, LPCWSTR pstr);
	// delete nCount characters starting at zero-based index
	int Delete(int nIndex, int nCount = 1);

	// searching (return starting index, or -1 if not found)
	// look for a single character match
	int Find(wchar_t ch) const; // like "C" strchr
	int ReverseFind(wchar_t ch) const;
	int Find(wchar_t ch, int nStart) const; // starting at index
	int FindOneOf(LPCWSTR lpszCharSet) const;

	// look for a specific sub-string
	int Find(LPCWSTR lpszSub) const;			 // like "C" strstr
	int Find(LPCWSTR lpszSub, int nStart) const; // starting at index

	// Concatentation for non strings
	const CString& Append(int n);

	// simple formatting
	int Format(LPCWSTR lpszFormat, ...);

	int FormatV(LPCWSTR lpszFormat, va_list args);

	int FormatModern(LPCWSTR lpszFormat, va_list args);

	// Access to string implementation buffer as "C" character array
	LPWSTR GetBuffer();
	void ReleaseBuffer(int nNewLength = -1);
	void FreeExtra();

	// Use LockBuffer/UnlockBuffer to turn refcounting off
	LPWSTR LockBuffer();
	void UnlockBuffer();

	void Truncate(int nNewLength);
	LPWSTR GetBuffer(int nMinBufLength);
	void ReleaseBufferSetLength(int nNewLength);
	// Implementation
public:
	~CString();
	int GetAllocLength() const;

protected:
	LPWSTR m_pchData; // pointer to ref counted string data
#if defined(__GNUC__)
	static ptSpinLock lockrefs;
#endif

	// implementation helpers
	CStringData* GetData() const;
	void Init();
	void AllocCopy(CString& dest, int nCopyLen, int nCopyIndex, int nExtraLen) const;
	bool AllocBuffer(int nLen);
	void AssignCopy(int nSrcLen, LPCWSTR lpszSrcData);
	bool ConcatCopy(int nSrc1Len, LPCWSTR lpszSrc1Data, int nSrc2Len, LPCWSTR lpszSrc2Data);
	void ConcatInPlace(int nSrcLen, LPCWSTR lpszSrcData);
	void CopyBeforeWrite();
	bool AllocBeforeWrite(int nLen);
	void Release();
	static void Release(CStringData* pData);
	static int SafeStrlen(LPCWSTR lpsz);
	static void FreeData(CStringData* pData);
	static const CString& _GetEmptyString();

	static inline int safe_inc(int* value)
	{
#if defined(_WIN32)
		return InterlockedIncrement((long*)value);
#elif defined(__GNUC__)
		ptSpinLockAdp lock(&lockrefs);
		return ++(*value);
#else
#error not support
#endif
	}

	static inline int safe_dec(int* value)
	{
#if defined(_WIN32)
		return InterlockedDecrement((long*)value);
#elif defined(__GNUC__)
		ptSpinLockAdp lock(&lockrefs);
		return --(*value);
#else
#error not support
#endif
	}
};


// Compare helpers
bool operator==(const CString& s1, const CString& s2);
bool operator==(const CString& s1, LPCWSTR s2);
bool operator==(LPCWSTR s1, const CString& s2);
bool operator!=(const CString& s1, const CString& s2);
bool operator!=(const CString& s1, LPCWSTR s2);
bool operator!=(LPCWSTR s1, const CString& s2);
bool operator<(const CString& s1, const CString& s2);
bool operator<(const CString& s1, LPCWSTR s2);
bool operator<(LPCWSTR s1, const CString& s2);
bool operator>(const CString& s1, const CString& s2);
bool operator>(const CString& s1, LPCWSTR s2);
bool operator>(LPCWSTR s1, const CString& s2);
bool operator<=(const CString& s1, const CString& s2);
bool operator<=(const CString& s1, LPCWSTR s2);
bool operator<=(LPCWSTR s1, const CString& s2);
bool operator>=(const CString& s1, const CString& s2);
bool operator>=(const CString& s1, LPCWSTR s2);
bool operator>=(LPCWSTR s1, const CString& s2);


/////////////////////////////////////////////////////////////////////////////
// CString Implementation

inline CStringData* CString::GetData() const
{
	assert(m_pchData != NULL);
	return ((CStringData*)m_pchData) - 1;
}
inline void CString::Init() { m_pchData = _GetEmptyString().m_pchData; }
inline int CString::GetLength() const { return GetData()->nDataLength; }
inline int CString::GetAllocLength() const { return GetData()->nAllocLength; }
inline bool CString::IsEmpty() const { return GetData()->nDataLength == 0; }
inline CString::operator LPCWSTR() const { return m_pchData; }
inline int CString::SafeStrlen(LPCWSTR lpsz) { return (lpsz == NULL) ? 0 : wcslen(lpsz); }
inline CString::CString() { Init(); }

// CString support (windows specific)
inline int CString::Compare(LPCWSTR lpsz) const { return wcscmp(m_pchData, lpsz); }
inline int CString::CompareNoCase(LPCWSTR lpsz) const
{
#if defined(_WIN32)
	return _wcsicmp(m_pchData, lpsz);
#elif defined(__GNUC__)
	return wcscmp(m_pchData, lpsz);
#endif
}

inline wchar_t CString::GetAt(int nIndex) const
{
	assert(nIndex >= 0);
	assert(nIndex < GetData()->nDataLength);
	return m_pchData[nIndex];
}
inline wchar_t CString::operator[](int nIndex) const
{
	// same as GetAt
	assert(nIndex >= 0);
	assert(nIndex < GetData()->nDataLength);
	return m_pchData[nIndex];
}
inline bool operator==(const CString& s1, const CString& s2) { return s1.Compare(s2) == 0; }
inline bool operator==(const CString& s1, LPCWSTR s2) { return s1.Compare(s2) == 0; }
inline bool operator==(LPCWSTR s1, const CString& s2) { return s2.Compare(s1) == 0; }
inline bool operator!=(const CString& s1, const CString& s2) { return s1.Compare(s2) != 0; }
inline bool operator!=(const CString& s1, LPCWSTR s2) { return s1.Compare(s2) != 0; }
inline bool operator!=(LPCWSTR s1, const CString& s2) { return s2.Compare(s1) != 0; }
inline bool operator<(const CString& s1, const CString& s2) { return s1.Compare(s2) < 0; }
inline bool operator<(const CString& s1, LPCWSTR s2) { return s1.Compare(s2) < 0; }
inline bool operator<(LPCWSTR s1, const CString& s2) { return s2.Compare(s1) > 0; }
inline bool operator>(const CString& s1, const CString& s2) { return s1.Compare(s2) > 0; }
inline bool operator>(const CString& s1, LPCWSTR s2) { return s1.Compare(s2) > 0; }
inline bool operator>(LPCWSTR s1, const CString& s2) { return s2.Compare(s1) < 0; }
inline bool operator<=(const CString& s1, const CString& s2) { return s1.Compare(s2) <= 0; }
inline bool operator<=(const CString& s1, LPCWSTR s2) { return s1.Compare(s2) <= 0; }
inline bool operator<=(LPCWSTR s1, const CString& s2) { return s2.Compare(s1) >= 0; }
inline bool operator>=(const CString& s1, const CString& s2) { return s1.Compare(s2) >= 0; }
inline bool operator>=(const CString& s1, LPCWSTR s2) { return s1.Compare(s2) >= 0; }
inline bool operator>=(LPCWSTR s1, const CString& s2) { return s2.Compare(s1) <= 0; }

#endif // __CSTRING_H__
