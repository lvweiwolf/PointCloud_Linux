#include <src/service/Options.h>

#include <src/utils/logging.h>

namespace d3s {
	namespace pcs {

		COptions::COptions() {}

		COptions::~COptions() {}

		void COptions::Set(const char* pszName, float fVal)
		{
			CHECK(pszName);
			_values.put(pszName, fVal);
		}

		void COptions::Set(const char* pszName, double dVal)
		{
			CHECK(pszName);
			_values.put(pszName, dVal);
		}

		void COptions::Set(const char* pszName, int iVal)
		{
			CHECK(pszName);
			_values.put(pszName, iVal);
		}

		void COptions::Set(const char* pszName, const char* pszVal)
		{
			CHECK(pszName);
			_values.put(pszName, std::string(pszVal));
		}

		void COptions::Set(const char* pszName, bool bVal)
		{
			CHECK(pszName);
			_values.put(pszName, bVal);
		}

		void COptions::Set(const char* pszName, char* pBuffer)
		{
			CHECK(pszName);
			_buffer_map[std::string(pszName)] = pBuffer;
		}

		float COptions::GetFloat(const char* pszName) const
		{
			CHECK(pszName);

			float value = 0.f;

			try
			{

				value = _values.get<float>(pszName);
			}
			catch (std::exception& e)
			{
				PCS_ERROR(e.what());
			}

			return value;
		}

		double COptions::GetDouble(const char* pszName) const
		{
			CHECK(pszName);

			double value = 0.0;

			try
			{
				value = _values.get<double>(pszName);
			}
			catch (std::exception& e)
			{
				PCS_ERROR(e.what());
			}

			return value;
		}

		int COptions::GetInt(const char* pszName) const
		{
			CHECK(pszName);

			int value = 0;

			try
			{
				value = _values.get<int>(pszName);
			}
			catch (std::exception& e)
			{
				PCS_ERROR(e.what());
			}

			return value;
		}

		bool COptions::GetBool(const char* pszName) const
		{
			CHECK(pszName);

			bool value = false;

			try
			{
				value = _values.get<bool>(pszName);
			}
			catch (std::exception& e)
			{
				PCS_ERROR(e.what());
			}

			return value;
		}

		std::string COptions::GetStr(const char* pszName) const
		{
			CHECK(pszName);
			std::string value;

			try
			{
				value = _values.get<std::string>(pszName);
			}
			catch (std::exception& e)
			{
				PCS_ERROR(e.what());
			}

			return value;
		}

		void* COptions::GetData(const char* pszName) const
		{
			CHECK(pszName);
			std::string name(pszName);

			auto iter = _buffer_map.find(name);
			if (iter == _buffer_map.end())
				return nullptr;

			return static_cast<void*>(iter->second);
		}

	}
}