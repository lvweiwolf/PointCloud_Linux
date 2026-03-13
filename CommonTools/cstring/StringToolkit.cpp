// 文件系统
/////////////////////////////////////////////////////////////////////////////

#include <include/StringToolkit.h>

#include <math.h>

#if defined(__GNUC__)
#include <locale>
#include <codecvt>
#endif

// 删除浮点数字符串数据的 0
// 例： 0012.34000 返回为 12.34；12.00 返回为 12
CString CStringToolkit::TrimNumberString(CString NumStr)
{
	int i;
	// 如果没有小数点则直接退出
	if (NumStr.Find(L".") < 0)
		return NumStr;

	// 删除字符串前面的0
	for (i = 0; i < NumStr.GetLength(); i++)
		if (NumStr.GetAt(i) == L'0')
		{
			NumStr.Delete(i, 1);
			i--;
		}
		else
			break;

	// 查找小数点或最后一个数值的位置
	for (i = NumStr.GetLength() - 1; i > 0; i--)
		if (NumStr.GetAt(i) != L'0')
			break;

	// 处理最第一个.号的问题
	if (i >= 0 && NumStr.Left(1) == L".")
	{
		i++;
		NumStr = L"0" + NumStr;
	}
	// 删除最后一个小数点
	if (i >= 0 && NumStr.Mid(i, 1) == L".")
		i--;
	// 返回最后的字符串
	if (i >= 0)
		return NumStr.Left(i + 1);
	else
		return L"0";
}

CString CStringToolkit::DblToStr(double value)
{
#ifdef _UNICODE
	wchar_t valstr[255];
	swprintf_s(valstr, L"%f", value);
#else
	char valstr[255];
	sprintf(valstr, "%f", value);
#endif
	return TrimNumberString(CStringToolkit::convertUTF8toUTF16(valstr).c_str());
}

CString CStringToolkit::Trim(CString sString)
{
	CString cs = sString;
	cs.TrimLeft();
	cs.TrimRight();
	return cs;
}

int CStringToolkit::CountWord(CString sString, wchar_t sParChr)
{
	int cnt, idx, sLen;
	wchar_t PrevChr;

	cnt = 0;
	PrevChr = sParChr;

	sLen = sString.GetLength();
	for (idx = 0; idx < sLen; idx++)
	{
		if ((sString.GetAt(idx) != sParChr) && (PrevChr == sParChr))
			cnt++;
		PrevChr = sString.GetAt(idx);
	}
	return cnt;
}

CString CStringToolkit::ReadWord(CString sString, int WordIdx, wchar_t sParChr)
{
	int cnt, idx, sLen, idx2;
	wchar_t PrevChr;
	CString NewWord;

	cnt = 0;
	PrevChr = sParChr;
	sLen = sString.GetLength();
	for (idx = 0; idx < sLen; idx++)
	{
		if ((sString.GetAt(idx) != sParChr) && (PrevChr == sParChr))
		{
			cnt++;
			if (cnt == WordIdx)
			{
				idx2 = idx;
				while (idx2 < sString.GetLength() && sString.GetAt(idx2) != sParChr)
					idx2++;
				if (idx2 >= sString.GetLength())
					return sString.Mid(idx);
				else
					return sString.Mid(idx, idx2 - idx);
			}
		}
		PrevChr = sString.GetAt(idx);
	}
	return L"";
}

int CStringToolkit::FindWord(CString SourStr,
							 CString Word,
							 bool RigorFind,
							 int StartPos,
							 wchar_t sParChr)
{
	int i, cnt;
	CString s1, f1;
	cnt = CountWord(SourStr, sParChr);
	if (!cnt || cnt < StartPos)
		return 0;

	if (RigorFind == false)
	{
		s1 = Trim(SourStr);
		f1 = Trim(Word);
		s1.MakeUpper();
		f1.MakeUpper();
		// 将分割字符转为大写 SHL
		if (sParChr >= L'a' && sParChr <= L'z')
			sParChr -= 32;
	}
	else
	{
		s1 = SourStr;
		f1 = Word;
	}

	for (i = StartPos; i <= cnt; i++)
	{
		if (ReadWord(s1, i, sParChr) == f1)
			return i;
	}
	return 0;
}

int CStringToolkit::FindLastChar(CString str, wchar_t chr, int pos)
{
	// 不支持负数位置
	if (pos < 0)
		return -1;

	int repos = str.GetLength() - pos - 1;
	while (repos >= 0)
		if (str.GetAt(repos) == chr)
			break;
		else
			repos--;
	if (repos >= 0)
		return repos;
	else
		return -1;
}

CString CStringToolkit::IntToStr(long value)
{
	CString str;
	str.Format(L"%ld", value);
	return str;
}

long CStringToolkit::StrToInt(CString str) { return _wtol(str); }

// Str To Double 字符串必须是 "1234.696" 形式
double CStringToolkit::StrToDouble(CString str)
{
	double Result = 0;

	if (str.Find(L".") == -1)
	{
		Result = (double)_wtol(str);
		return Result;
	}

	// 用来支持形如“.123”的转换（“-.123”形式已支持，不需要特使处理）
	if (str.GetAt(0) == L'.')
		str = L"0" + str;

	CString TmpStr;

	// 取得小数点前部分值
	CString BeforeStr = ReadWord(str, 1, L'.');
	int len = BeforeStr.GetLength() - 1;
	for (int i = 0; i < BeforeStr.GetLength(); i++)
	{
		TmpStr.Format(L"%c", BeforeStr[i]);
		int value = _wtol(TmpStr);
		Result += value * powl(10, len--);
	}

	// 取得小数点前部分值
	CString AfterStr = ReadWord(str, 2, L'.');
	for (int i = 0; i < AfterStr.GetLength(); i++)
	{
		TmpStr.Format(L"%c", AfterStr[i]);
		int value = _wtol(TmpStr);
		Result += value * powl(0.1, i + 1);
	}

	// 如果是负数，则再把原来的值乘以-1
	if (str.Left(1) == L'-')
		Result *= -1;

	return Result;
}

// 比较两个版本号大小
int CStringToolkit::CompareVersion(CString version1, CString version2, int bit)
{
	int nCount1 = CStringToolkit::CountWord(version1, L'.');
	int nCount2 = CStringToolkit::CountWord(version2, L'.');
	if (bit != -1 && nCount1 > bit)
		nCount1 = bit;
	for (int i = 1; i <= nCount1; ++i)
	{
		if (i > nCount2)
			return 1;

		if (CStringToolkit::ReadWord(version1, i, L'.') ==
			CStringToolkit::ReadWord(version2, i, L'.'))
			continue;
		else if (_wtol(CStringToolkit::ReadWord(version1, i, L'.')) <
				 _wtol(CStringToolkit::ReadWord(version2, i, L'.')))
			return -1;
		else
			return 1;
	}

	if (bit != -1)
		return 0;

	if (nCount1 > nCount2)
		return 1;
	else if (nCount1 < nCount2)
		return -1;

	return 0;
}


bool CStringToolkit::IsNumber(CString strText)
{
	if (strText.IsEmpty())
		return false;
	int nDot = 0;
	// 数值只能是0到9及小数点组成
	for (int i = 0; i < strText.GetLength(); i++)
	{
		char ch = strText.GetAt(i);
		if ('.' == ch)
		{
			nDot++;
			continue;
		}
		if (ch >= '0' && ch <= '9')
			continue;
		if (ch == '-' && i == 0)
			continue;
		return false;
	}
	if (nDot > 1)
		return false;
	else
		return true;
}

void CStringToolkit::SplitString(const CString& strNeedSplit,
								 std::vector<CString>& strSplitStrings,
								 CString strSep /*= _L("#")*/)
{
	if (strNeedSplit.IsEmpty())
		return;

	int nSepLen = strSep.GetLength();

	// 1、获取分隔数据的起始位置与结束位置
	int nTextStart = 0;
	int nTextEnd = strNeedSplit.Find(strSep, nTextStart);
	while (-1 != nTextEnd)
	{
		// 2、将解析好的数据添加到返回值中
		CString strText = strNeedSplit.Mid(nTextStart, nTextEnd - nTextStart);
		strSplitStrings.push_back(strText);

		// 3、获取下一个分隔数据的起始位置与结束位置
		nTextStart = nTextEnd + nSepLen;
		nTextEnd = strNeedSplit.Find(strSep, nTextStart);
	}

	// 4、获取最后一个分隔数据并添加到返回值中
	CString strLastText = strNeedSplit.Mid(nTextStart, strNeedSplit.GetLength() - nTextStart);
	if (!strLastText.IsEmpty())
		strSplitStrings.push_back(strLastText);
}

void CStringToolkit::ReplaceSpecialChar(CString& strValue, CString strReplace)
{
	strValue.Replace(L" ", strReplace);
	strValue.Replace(L"=", strReplace);
	strValue.Replace(L"\"", strReplace);
}

std::wstring CStringToolkit::convertUTF8toUTF16(const char* from)
{
	try
	{
		wchar_t* unicode = NULL;
		int wchars, err;
		try
		{
			setlocale(LC_ALL, "");
			wchars = mbstowcs(unicode, from, 0);
		}
		catch (const std::exception&)
		{
			std::wstring to;
			return to;
		}

		std::wstring to(wchars, L'\0');

		err = mbstowcs(&to[0], from, wchars);
		return to;
	}
	catch (const std::exception&)
	{
		std::wstring to;
		return to;
	}
}

std::string CStringToolkit::convertUTF16toUTF8(const wchar_t* from)
{
	char* utf8 = NULL;
	setlocale(LC_ALL, "");
	int chars = wcstombs(utf8, from, 0);

	std::string to(chars, '\0');

	// setlocale(LC_ALL,"");
	wcstombs(&to[0], from, chars);
	setlocale(LC_ALL, "C");

	return to;
}


std::string CStringToolkit::CStringToUTF8(const CString& str)
{
	if (str.IsEmpty())
		return "";

#if defined(_WIN32)
	// Windows: CString是wchar_t，转换为UTF8
	int utf8Length = WideCharToMultiByte(CP_UTF8, 0, str, -1, NULL, 0, NULL, NULL);
	if (utf8Length == 0)
		return "";

	std::string utf8Str(utf8Length, 0);
	WideCharToMultiByte(CP_UTF8, 0, str, -1, &utf8Str[0], utf8Length, NULL, NULL);
	utf8Str.resize(utf8Length - 1); // 去除末尾的\0

	return utf8Str;
#else
	// Linux: CString是wchar_t，使用codecvt转换为UTF8
	LPCWSTR wstr = str;
	std::wstring wstrS(wstr);
	std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
	return converter.to_bytes(wstrS);
#endif
}

CString CStringToolkit::UTF8ToCString(const std::string& utf8Str)
{
	if (utf8Str.empty())
		return CString();

#if defined(_WIN32)
	// Windows: UTF8转换为wchar_t
	int wcharLength = MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, NULL, 0);
	if (wcharLength == 0)
		return CString();

	std::wstring wstr(wcharLength, 0);
	MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, &wstr[0], wcharLength);
	wstr.resize(wcharLength - 1); // 去除末尾的\0

	return CString(wstr.c_str());
#else
	// Linux: UTF8转换为wchar_t
	std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
	std::wstring wstr = converter.from_bytes(utf8Str);
	return CString(wstr.c_str());
#endif
}
