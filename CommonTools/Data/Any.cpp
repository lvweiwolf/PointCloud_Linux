
#include <include/Any.h>

#include <cstdint>

const int gl_nMinPntValue = 3; // 三维点坐标最少三个分量

namespace pc {
	namespace data {
		// Linux下字符串转换函数实现
		int64_t CAny::WStringToInt64(const wchar_t* str)
		{
			if (!str)
				return 0;
			wchar_t* endptr;
			errno = 0;
			int64_t result = wcstoll(str, &endptr, 10);
			if (errno == ERANGE)
			{
				// 处理溢出
				return (result == LLONG_MAX) ? INT64_MAX : INT64_MIN;
			}
			return result;
		}

		double CAny::WStringToDouble(const wchar_t* str)
		{
			if (!str)
				return 0.0;
			wchar_t* endptr;
			errno = 0;
			double result = wcstod(str, &endptr);
			if (errno == ERANGE)
			{
				// 处理溢出
				return result; // wcstod会返回HUGE_VAL或0
			}
			return result;
		}

		long CAny::WStringToLong(const wchar_t* str)
		{
			if (!str)
				return 0;
			wchar_t* endptr;
			errno = 0;
			long result = wcstol(str, &endptr, 10);
			if (errno == ERANGE)
			{
				// 处理溢出
				return (result == LONG_MAX) ? LONG_MAX : LONG_MIN;
			}
			return result;
		}

		CAny::CAny()
		{
			_vdType = ANY_EMPTY;
			_vdValue.valInt64 = 0;
		}

		CAny::CAny(const osg::Vec3d& value)
		{
			_vdType = ANY_POINT;
			_vdValue.valPoint = new osg::Vec3d();
			*_vdValue.valPoint = value;
		}

		CAny::CAny(const std::vector<osg::Vec3d>& value)
		{
			_vdType = ANY_POINTDS;
			_vdValue.valPointds = new std::vector<osg::Vec3d>;
			*_vdValue.valPointds = value;
		}

		CAny::CAny(const osg::Matrix& value)
		{
			_vdType = ANY_MATRIX;
			_vdValue.valMatrix = new osg::Matrix();
			*_vdValue.valMatrix = value;
		}

		CAny& CAny::operator=(const osg::Matrix& value)
		{
			ClearTypeMemory();
			_vdType = ANY_MATRIX;
			_vdValue.valMatrix = new osg::Matrix();
			*_vdValue.valMatrix = value;

			return *this;
		}

		CAny::operator osg::Matrix() const
		{
			switch (_vdType)
			{
			case ANY_MATRIX:
				return (*_vdValue.valMatrix);
			default:
				return osg::Matrix();
			}
		}

		CAny& CAny::operator=(const osg::Vec3d& value)
		{
			ClearTypeMemory();
			_vdType = ANY_POINT;
			_vdValue.valPoint = new osg::Vec3d();
			*_vdValue.valPoint = value;

			return *this;
		}
		CAny::operator osg::Vec3d() const
		{
			switch (_vdType)
			{
			case ANY_POINT:
				return (*_vdValue.valPoint);
			default:
				return osg::Vec3();
			}
		}

		CAny& CAny::operator=(const std::vector<osg::Vec3d>& value)
		{
			ClearTypeMemory();
			_vdType = ANY_POINTDS;
			_vdValue.valPointds = new std::vector<osg::Vec3d>();
			*_vdValue.valPointds = value;

			return *this;
		}
		CAny::operator std::vector<osg::Vec3d>() const
		{
			switch (_vdType)
			{
			case ANY_POINTDS:
				return (*_vdValue.valPointds);
			default:
				return std::vector<osg::Vec3d>();
			}
		}

		CAny::~CAny() { ClearTypeMemory(); }

		// bool
		CAny::CAny(const bool& value)
		{
			_vdType = ANY_BOOL;
			_vdValue.valBool = value;
		}

		void CAny::Attach(const CAny& other) const
		{
			// 采用特殊手段来处理
			CAny* pThis = const_cast<CAny*>(this);
			CAny* pOther = const_cast<CAny*>(&other);

			pThis->ClearTypeMemory();

			pThis->_vdType = pOther->_vdType;
			pThis->_vdValue.valInt64 = pOther->_vdValue.valInt64; // 随便取一个最大的值赋值

			pOther->_vdType = ANY_EMPTY;
			pOther->_vdValue.valInt64 = 0;
		}

		CAny& CAny::operator=(const bool& value)
		{
			ClearTypeMemory();
			_vdType = ANY_BOOL;
			_vdValue.valBool = value;

			return *this;
		}

		CAny::operator bool() const
		{
			switch (_vdType)
			{
			case ANY_EMPTY:
				return false;
			case ANY_BOOL:
				return _vdValue.valBool;
			case ANY_INT:
				return !!_vdValue.valInt;
			case ANY_INT64:
				return !!_vdValue.valInt64;
			case ANY_DOUBLE:
				return !!(long)_vdValue.valDouble;
			case ANY_POINT:
				return (_vdValue.valPoint->x() != 0 || _vdValue.valPoint->y() != 0 ||
						_vdValue.valPoint->z() != 0);
			case ANY_STRING:
				return !wcscmp(_vdValue.valString, L"True");
			default:
				return false;
			}
		}

		// int
		CAny::CAny(const int& value)
		{
			_vdType = ANY_INT;
			_vdValue.valInt = value;
		}

		CAny& CAny::operator=(const int& value)
		{
			ClearTypeMemory();
			_vdType = ANY_INT;
			_vdValue.valInt = value;

			return *this;
		}

		CAny::operator int() const
		{
			switch (_vdType)
			{
			case ANY_EMPTY:
				return 0;
			case ANY_BOOL:
				return _vdValue.valBool ? 1 : 0;
			case ANY_INT:
				return _vdValue.valInt;
			case ANY_INT64:
				return _vdValue.valInt64;
			case ANY_DOUBLE:
				return (int)_vdValue.valDouble;
			case ANY_STRING:
				return WStringToLong(_vdValue.valString);
			default:
				return 0;
			}
		}

		//// int64
		// CAny::CAny(const int64_t& value)
		//{
		//	_vdType = ANY_INT64;
		//	_vdValue.valInt64 = value;
		// }

		// CAny& CAny::operator = (const int64_t& value)
		//{
		//	ClearTypeMemory();
		//	_vdType = ANY_INT64;
		//	_vdValue.valInt64 = value;

		//	return *this;
		//}

		// CAny::operator int64_t() const
		//{
		//	switch (_vdType)
		//	{
		//	case ANY_EMPTY:
		//		return 0;
		//	case ANY_BOOL:
		//		return _vdValue.valBool ? 1 : 0;
		//	case ANY_INT:
		//		return _vdValue.valInt;
		//	case ANY_INT64:
		//		return _vdValue.valInt64;
		//	case ANY_DOUBLE:
		//		return (int64_t)_vdValue.valDouble;
		//	case ANY_STRING:
		//		return WStringToInt64(_vdValue.valString);
		//	default:
		//		return 0;
		//	}
		// }


		// size_t
		CAny::CAny(const size_t& value)
		{
			_vdType = ANY_INT;
			_vdValue.valInt = int(value);
		}

		CAny& CAny::operator=(const size_t& value)
		{
			ClearTypeMemory();
			_vdType = ANY_INT;
			_vdValue.valInt = int(value);

			return *this;
		}

		CAny::operator size_t() const
		{
			switch (_vdType)
			{
			case ANY_EMPTY:
				return 0;
			case ANY_BOOL:
				return _vdValue.valBool ? 1 : 0;
			case ANY_INT:
				return (size_t)_vdValue.valInt;
			case ANY_INT64:
				return (size_t)_vdValue.valInt64;
			case ANY_DOUBLE:
				return (size_t)_vdValue.valDouble;
			case ANY_STRING:
				return (size_t)WStringToLong(_vdValue.valString);
			default:
				return 0;
			}
		}

		// long
		CAny::CAny(const long& value)
		{
			_vdType = ANY_INT;
			_vdValue.valInt = value;
		}

		CAny& CAny::operator=(const long& value)
		{
			ClearTypeMemory();
			_vdType = ANY_INT;
			_vdValue.valInt = value;

			return *this;
		}

		CAny::operator long() const
		{
			switch (_vdType)
			{
			case ANY_EMPTY:
				return 0;
			case ANY_BOOL:
				return _vdValue.valBool ? 1 : 0;
			case ANY_INT:
				return (long)_vdValue.valInt;
			case ANY_INT64:
				return (long)_vdValue.valInt64;
			case ANY_DOUBLE:
				return (long)_vdValue.valDouble;
			case ANY_STRING:
				return _wtol(_vdValue.valString);
			default:
				return 0;
			}
		}

		// double
		CAny::CAny(const double& value)
		{
			_vdType = ANY_DOUBLE;
			_vdValue.valDouble = value;
		}

		CAny& CAny::operator=(const double& value)
		{
			ClearTypeMemory();
			_vdType = ANY_DOUBLE;
			_vdValue.valDouble = value;

			return *this;
		}

		CAny::operator double() const
		{
			switch (_vdType)
			{
			case ANY_EMPTY:
				return 0.0;
			case ANY_BOOL:
				return _vdValue.valBool ? 1.0 : 0.0;
			case ANY_INT:
				return (double)_vdValue.valInt;
			case ANY_INT64:
				return (double)_vdValue.valInt64;
			case ANY_DOUBLE:
				return _vdValue.valDouble;
			case ANY_STRING:
				return _wtof(_vdValue.valString);
			default:
				return 0.0;
			}
		}

		// String
		CAny::CAny(const LPCWSTR& value)
		{
			_vdType = ANY_STRING;

			int len = int(wcslen(value));
			_vdValue.valString = new WCHAR[len + 1];
			wcsncpy(_vdValue.valString, value, len);
			_vdValue.valString[len] = 0;
		}

		CAny& CAny::operator=(const LPCWSTR& value)
		{
			ClearTypeMemory();
			_vdType = ANY_STRING;

			int len = int(wcslen(value));
			_vdValue.valString = new WCHAR[len + 1];
			wcsncpy(_vdValue.valString, value, len);
			_vdValue.valString[len] = 0;

			return *this;
		}

		LPWSTR CAny::BuildString(UINT nLen)
		{
			ClearTypeMemory();
			_vdType = ANY_STRING;

			_vdValue.valString = new WCHAR[nLen + 1];
			_vdValue.valString[nLen] = 0;
			return _vdValue.valString;
		}

		CAny::operator LPCWSTR() const
		{
			switch (_vdType)
			{
			case ANY_EMPTY:
				return NULL;
			case ANY_BOOL:
				return _vdValue.valBool ? L"True" : L"False";
			case ANY_STRING:
				return (LPCWSTR)_vdValue.valString;
			default:
				return NULL;
			}

			return NULL;
		}

		CAny::operator CString() const
		{
			CString strFormat;
			switch (_vdType)
			{
			case ANY_EMPTY:
				return L"";
			case ANY_BOOL:
				return _vdValue.valBool ? L"True" : L"False";
			case ANY_INT:
				strFormat.Format(L"%d", _vdValue.valInt);
				return strFormat;
			case ANY_INT64:
				strFormat.Format(L"%I64d", _vdValue.valInt64);
				return strFormat;
			case ANY_DOUBLE:
				strFormat.Format(L"%f", _vdValue.valDouble);
				return strFormat;
			case ANY_STRING:
				return (LPCWSTR)_vdValue.valString;
			case ANY_POINT:
				strFormat.Format(L"%f,%f,%f",
								 _vdValue.valPoint->x(),
								 _vdValue.valPoint->y(),
								 _vdValue.valPoint->z());
				return strFormat;
			case ANY_POINTDS:
				for (const auto& iter : *(_vdValue.valPointds))
					strFormat.Format(L"%s%lf,%lf,%lf:",
									 static_cast<LPCWSTR>(strFormat),
									 iter.x(),
									 iter.y(),
									 iter.z());
				return strFormat;
			case ANY_MATRIX:
			{
				double* pMat = _vdValue.valMatrix->ptr();
				strFormat.Format(L"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
								 *pMat,
								 *(pMat + 1),
								 *(pMat + 2),
								 *(pMat + 3),
								 *(pMat + 4),
								 *(pMat + 5),
								 *(pMat + 6),
								 *(pMat + 7),
								 *(pMat + 8),
								 *(pMat + 9),
								 *(pMat + 10),
								 *(pMat + 11),
								 *(pMat + 12),
								 *(pMat + 13),
								 *(pMat + 14),
								 *(pMat + 15));
				return strFormat;
			}
			default:
				return L"";
			}
		}

		// CString
		CAny::CAny(const CString& value)
		{
			_vdType = ANY_STRING;

			int len = value.GetLength();
			_vdValue.valString = new WCHAR[len + 1];
			wcsncpy(_vdValue.valString, value, len);
			_vdValue.valString[len] = 0;
		}

		CAny& CAny::operator=(const CString& value)
		{
			ClearTypeMemory();
			_vdType = ANY_STRING;

			int len = value.GetLength();
			_vdValue.valString = new WCHAR[len + 1];
			wcsncpy(_vdValue.valString, value, len);
			_vdValue.valString[len] = 0;

			return *this;
		}

		CAny::CAny(const CAny& value)
		{
			_vdType = ANY_EMPTY;
			_vdValue.valString = NULL;

			switch (value._vdType)
			{
			case ANY_BOOL:
				operator=(value._vdValue.valBool);
				break;
			case ANY_INT:
				operator=(value._vdValue.valInt);
				break;
			case ANY_INT64:
				operator=(value._vdValue.valInt64);
				break;
			case ANY_DOUBLE:
				operator=(value._vdValue.valDouble);
				break;
			case ANY_STRING:
				operator=(value._vdValue.valString);
				break;
			case ANY_POINT:
				operator=(*value._vdValue.valPoint);
				break;
			case ANY_POINTDS:
				operator=(*value._vdValue.valPointds);
				break;
			case ANY_STREAM:
			{
				BYTE* valStream = NULL;
				int nByte = value.GetStreamValue(valStream);
				SetStreamValue(valStream, nByte);
			}
			break;
			case ANY_MATRIX:
				operator=(*value._vdValue.valMatrix);
				break;
			case ANY_EMPTY:
			default:
				_vdType = ANY_EMPTY;
				_vdValue.valString = NULL;
				break;
			}
		}

		CAny& CAny::operator=(const CAny& value)
		{
			ClearTypeMemory();
			switch (value._vdType)
			{
			case ANY_BOOL:
				operator=(value._vdValue.valBool);
				break;
			case ANY_INT:
				operator=(value._vdValue.valInt);
				break;
			case ANY_INT64:
				operator=(value._vdValue.valInt64);
				break;
			case ANY_DOUBLE:
				operator=(value._vdValue.valDouble);
				break;
			case ANY_STRING:
				operator=(value._vdValue.valString);
				break;
			case ANY_POINT:
				operator=(*value._vdValue.valPoint);
				break;
			case ANY_POINTDS:
				operator=(*value._vdValue.valPointds);
				break;
			case ANY_STREAM:
			{
				BYTE* valStream = NULL;
				int nByte = value.GetStreamValue(valStream);
				SetStreamValue(valStream, nByte);
			}
			break;
			case ANY_MATRIX:
				operator=(*value._vdValue.valMatrix);
				break;
			case ANY_EMPTY:
			default:
				_vdType = ANY_EMPTY;
				_vdValue.valString = NULL;
				break;
			}

			return *this;
		}

		CAny::CAny(const BYTE* value, int len) { SetStreamValue(value, len); }

		void CAny::SetStreamValue(const BYTE* value, int len)
		{
			ClearTypeMemory();
			_vdType = ANY_STREAM;

			_vdValue.valStream = new BYTE[len + sizeof(int)];

			// 赋值字节长度
			memcpy(_vdValue.valStream, (void*)(&len), sizeof(int));
			memcpy(_vdValue.valStream + sizeof(int), (void*)value, len);
		}

		int CAny::GetStreamValue(BYTE*& value) const
		{
			int nLen = 0;
			value = NULL;

			if (_vdType != ANY_STREAM || _vdValue.valStream == NULL)
				return nLen;

			// 获取字节长度
			memcpy((void*)(&nLen), _vdValue.valStream, sizeof(int));
			value = _vdValue.valStream + sizeof(int);

			return nLen;
		}

		BYTE* CAny::BuildStreamValue(int len)
		{
			ClearTypeMemory();
			_vdType = ANY_STREAM;

			_vdValue.valStream = new BYTE[len + sizeof(int)];

			// 赋值字节长度
			memcpy(_vdValue.valStream, (void*)(&len), sizeof(int));
			return _vdValue.valStream + sizeof(int);
		}

		osg::Vec3d CAny::StringToPoint(LPCWSTR lpPoint)
		{
			CString strPoint(lpPoint);
			int nFirst = strPoint.Find(L",");
			if (-1 == nFirst)
				return osg::Vec3d();

			int nSecond = strPoint.Find(L",", nFirst + 1);
			if (-1 == nSecond)
				return osg::Vec3d();

			return osg::Vec3d(
				CStringToolkit::StrToDouble(strPoint.Mid(0, nFirst)),
				CStringToolkit::StrToDouble(strPoint.Mid(nFirst + 1, nSecond - nFirst - 1)),
				CStringToolkit::StrToDouble(
					strPoint.Mid(nSecond + 1, strPoint.GetLength() - nSecond - 1)));
		}

		std::vector<osg::Vec3d> CAny::StringToPointDS(LPCWSTR lpPoint)
		{
			std::vector<CString> strPnts;
			std::vector<osg::Vec3d> pnts;
			CStringToolkit::SplitString(lpPoint, strPnts, L":");
			for (const auto& iter : strPnts)
			{
				std::vector<CString> values;
				CStringToolkit::SplitString(iter, values, L",");
				if (values.size() < gl_nMinPntValue)
					continue;
				pnts.push_back(osg::Vec3d(CStringToolkit::StrToDouble(values.front()),
										  CStringToolkit::StrToDouble(values.at(1)),
										  CStringToolkit::StrToDouble(values.back())));
			}

			return pnts;
		}

		osg::Matrix CAny::StringToMatrix(LPCWSTR lpMatrix)
		{
			CString strMatrix(lpMatrix);
			if (strMatrix.IsEmpty())
				return osg::Matrix();

			int nPos = -1, nRow = 0, nCol = 0;
			double mat[4][4];

			for (int i = 0; i < 4; ++i)
			{
				for (int j = 0; j < 4; ++j)
				{
					int nSplit = strMatrix.Find(L",", nPos + 1);
					CString strDlb = strMatrix.Mid(nPos + 1, nSplit - nPos - 1);
					mat[i][j] = WStringToDouble(strDlb);
					nPos = nSplit;
				}
			}

			return osg::Matrix((double*)mat);
		}

		void CAny::ClearTypeMemory()
		{
			switch (_vdType)
			{
			case ANY_STRING:
				if (NULL != _vdValue.valString)
					delete[] _vdValue.valString;
				break;
			case ANY_STREAM:
				if (NULL != _vdValue.valStream)
					delete[] _vdValue.valStream;
				break;
			case ANY_GUID:
				if (NULL != _vdValue.valGuid)
					delete _vdValue.valGuid;
				break;
			case ANY_POINT:
				if (NULL != _vdValue.valPoint)
					delete _vdValue.valPoint;
				break;
			case ANY_POINTDS:
				if (NULL != _vdValue.valPointds)
					delete _vdValue.valPointds;
				break;
			case ANY_MATRIX: /// add Matrix
				if (NULL != _vdValue.valMatrix)
					delete _vdValue.valMatrix;
				break;
			}

			_vdType = ANY_EMPTY;
			_vdValue.valString = NULL;
		}

		bool CAny::IsEmpty() const { return _vdType == ANY_EMPTY; }

		EAnyType CAny::GetType() const { return (EAnyType)_vdType; }

		CString CAny::GetStringValue(int nDotNum) const
		{
			CString strDotNum, strFormat;
			switch (_vdType)
			{
			case ANY_EMPTY:
				return L"";
			case ANY_BOOL:
				return _vdValue.valBool ? L"True" : L"False";
			case ANY_INT:
				strFormat.Format(L"%d", _vdValue.valInt);
				return strFormat;
			case ANY_INT64:
				strFormat.Format(L"%I64d", _vdValue.valInt64);
				return strFormat;
			case ANY_DOUBLE:
				strDotNum.Format(L"%%.%df", nDotNum);
				return strFormat.Format((LPCWSTR)strDotNum, _vdValue.valDouble);
			case ANY_STRING:
				return (LPCWSTR)_vdValue.valString;
			case ANY_POINT:
				strDotNum.Format(L"%%.%df,%%.%df,%%.%df", nDotNum, nDotNum, nDotNum);
				strFormat.Format(strDotNum,
								 _vdValue.valPoint->x(),
								 _vdValue.valPoint->y(),
								 _vdValue.valPoint->z());
				return strFormat;
			case ANY_MATRIX:
				strFormat = std::wstring(*this).c_str();
				return strFormat;
			default:
				return L"";
			}
		}

		void CAny::SetStringValue(LPCWSTR lpValue, EAnyType vType)
		{
			CString strValue(lpValue);
			CString strDotNum, strFormat;
			switch (vType)
			{
			case ANY_EMPTY:
				break;
			case ANY_BOOL:
				operator=((strValue == L"True" || strValue == L"1") ? true : false);
				break;
			case ANY_INT:
				operator=(WStringToLong(strValue));
				break;
			case ANY_INT64:
				operator=((int64_t)(WStringToInt64(strValue)));
				break;
			case ANY_DOUBLE:
				operator=(_wtof(strValue));
				break;
			case ANY_STRING:
				operator=(strValue);
				break;
			case ANY_POINT:
				operator=(StringToPoint(strValue));
				break;
			case ANY_MATRIX:
				operator=(StringToMatrix(strValue));
				break;
			case ANY_GUID:
				break;
			default:
				break;
			}
		}

		bool CAny::operator==(const CAny& value) const
		{
			// 修复XMDX保存导致空字符串问题
			if ((_vdType == ANY_EMPTY && value._vdType == ANY_STRING) ||
				(_vdType == ANY_STRING && value._vdType == ANY_EMPTY))
			{
				return (value._vdValue.valString ? wcslen(value._vdValue.valString) : 0) ==
					   (_vdValue.valString ? wcslen(_vdValue.valString) : 0);
			}

			if (_vdType != value._vdType)
				return false;

			switch (_vdType)
			{
			case ANY_EMPTY:
				return true;
			case ANY_BOOL:
				return value._vdValue.valBool == _vdValue.valBool;
			case ANY_INT:
				return value._vdValue.valInt == _vdValue.valInt;
			case ANY_INT64:
				return value._vdValue.valInt64 == _vdValue.valInt64;
			case ANY_DOUBLE:
				return value._vdValue.valDouble == _vdValue.valDouble;
			case ANY_STRING:
				return !wcscmp(value._vdValue.valString, _vdValue.valString);
			case ANY_GUID:
				return _vdValue.valGuid->Data1 != value._vdValue.valGuid->Data1 ||
					   _vdValue.valGuid->Data2 != value._vdValue.valGuid->Data2 ||
					   _vdValue.valGuid->Data3 != value._vdValue.valGuid->Data3 ||
					   _vdValue.valGuid->Data4 != value._vdValue.valGuid->Data4;
			case ANY_MATRIX:
				return (*_vdValue.valMatrix) == (*value._vdValue.valMatrix);
			case ANY_STREAM:
				return false;
			case ANY_POINT:
				return (*_vdValue.valPoint) == (*value._vdValue.valPoint);
			case ANY_POINTDS:
				return (*_vdValue.valPointds) == (*value._vdValue.valPointds);
			default:
				return true;
			}
			return true;
		}

		bool CAny::operator!=(const CAny& value) const { return !(*this == value); }
	}
}