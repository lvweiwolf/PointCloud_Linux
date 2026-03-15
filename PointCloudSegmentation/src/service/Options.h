//////////////////////////////////////////////////////////////////////
// 文件名称：options.h
// 功能描述：算法参数配置类
// 创建标识：吕伟	2022/6/24
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once

#include <boost/property_tree/ptree.hpp>

#include "../../include/ICloudSegmentation.h"

namespace d3s {
	namespace pcs {

		class COptions : public IOptions
		{
		public:
			COptions();

			virtual ~COptions();


			/**
			 *  @brief    设置参数值
			 *
			 *  @param    const char *						参数名称
			 *  @param    float/double/int/string		各种类型的参数值
			 *
			 *  @return   void
			 */
			virtual void Set(const char* pszName, float fVal);
			virtual void Set(const char* pszName, double dVal);
			virtual void Set(const char* pszName, int iVal);
			virtual void Set(const char* pszName, bool bVal);
			virtual void Set(const char* pszName, const char* pszVal);
			virtual void Set(const char* pszName, char* pBuffer);

			/**
			 *  @brief    获得参数值
			 *
			 *  @param    const char *						参数名称
			 *
			 *  @return   float/double/int/string
			 */
			virtual float GetFloat(const char* pszName) const;
			virtual double GetDouble(const char* pszName) const;
			virtual int GetInt(const char* pszName) const;
			virtual bool GetBool(const char* pszName) const;
			virtual std::string GetStr(const char* pszName) const;
			virtual void* GetData(const char* pszName) const;

		private:
			boost::property_tree::ptree _values;
			std::map<std::string, char*> _buffer_map;
		};
	}
}
