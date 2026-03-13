//////////////////////////////////////////////////////////////////////
// 文件名称：Any.h
// 功能描述：可变数据类型
// 创建标识：吴建峰 2025/01/17
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////

#ifndef ANY_H_
#define ANY_H_

#include <include/StringToolkit.h>

#include <osg/Vec3>
#include <osg/Vec3d>
#include <osg/Matrix>

namespace pc
{
	namespace data
	{
		/*
		* 变型数据类型
		*/
		enum EAnyType
		{
			ANY_EMPTY = 0,		///< 空类型(没有赋值)
			ANY_BOOL = 1,		///< bool
			ANY_INT = 2,		///< int
			ANY_DOUBLE = 5,		///< double
			ANY_STRING = 6,		///< WCHAR*
			ANY_POINT = 7,		///< CGePoint3d
			ANY_GUID = 8,		///< GUID
			ANY_STREAM = 9,		///< BYTE*
			ANY_PTARRAY = 10,	///< CGePoint3d Array
			ANY_INT64 = 11,		///< int64
			ANY_MATRIX = 12,		///< Matrix
			ANY_POINTDS = 13,		///< CGePoint3ds
		};
	
		/**
		*  @class    可变数据类型类
		*
		*  @brief
		*/
		class COMMONTOOLS_EXPORT CAny
		{
		public:
			CAny();
			~CAny();
	
			/**
			*  @brief    判断是否为空
			*
			*  @return   bool 没有进行赋值则为false
			*/
			bool IsEmpty() const;
	
			/**
			*  @brief    获取数据类型
			*
			*  @return   EAnyType 返回对应数据类型
			*/
			EAnyType GetType() const;
	
			/**
			*  @brief    绑定其它对象数据(特定场景使用,用于提高赋值性能)
			*
			*  @param   const CAny& other(此对象调用后数据会清空)
			*/
			void Attach(const CAny& other) const;
	
		public:
			/**
			*  @brief    获取字符串值(内部进行转换)
			*
			*  @param    int nDotNum 小数点位数
			*  @return   CString 返回字符串值
			*/
			CString GetStringValue(int nDotNum = 2) const;
	
			/**
			*  @brief    置字符串值转成指定类型值
			*
			*  @param    LPWSTR lpValue 字符串值输入值
			*  @param    EAnyType vType 转换的数据类型
			*/
			void SetStringValue(LPCWSTR lpValue, EAnyType vType);
	
			/**
			*  @brief    判断两个CAny是否相等
			*
			*  @param    const CAny& value
			*/
			bool operator == (const CAny& value) const;
	
			/**
			*  @brief    判断两个CAny是否不相等
			*
			*  @param    const CAny& value
			*/
			bool operator != (const CAny& value) const;
	
		public:
			/**
			*  @brief    拷贝构造函数
			*
			*  @param    const CAny & value
			*/
			CAny(const CAny& value);
			CAny& operator = (const CAny& value);
	
			/**
			*  @brief    bool值构造
			*
			*  @param    const bool & value
			*  @return
			*/
			CAny(const bool& value);
			CAny& operator = (const bool& value);
			operator bool() const;
	
			/**
			*  @brief    int值构造
			*
			*  @param    const int & value
			*/
			CAny(const int& value);
			CAny& operator = (const int& value);
			operator int() const;
	
			///**
			//*  @brief    int64值构造
			//*
			//*  @param    const INT64 & value
			//*/
			//CAny(const int64_t& value);
			//CAny& operator = (const int64_t& value);
			//operator int64_t() const;
	
			/**
			*  @brief    size_t值构造
			*
			*  @param    const size_t & value
			*/
			CAny(const size_t& value);
			CAny& operator = (const size_t& value);
			operator size_t() const;
	
			/**
			*  @brief    long值构造
			*
			*  @param    const long & value
			*/
			CAny(const long& value);
			CAny& operator = (const long& value);
			operator long() const;
	
			/**
			*  @brief    double值构造
			*
			*  @param    const double & value
			*/
			CAny(const double& value);
			CAny& operator = (const double& value);
			operator double() const;
	
			/**
			*  @brief    字符串值构造
			*			 BuildString方法不明白的请不要使用
			*
			*  @param    const LPCTSTR & value
			*/
			CAny(const LPCWSTR& value);
			CAny& operator = (const LPCWSTR& value);
			operator LPCWSTR() const;
			LPWSTR BuildString(UINT nLen);
	
			/**
			*  @brief    字符串值构造
			*
			*  @param    const CString&  value
			*/
			CAny(const CString& value);
			CAny& operator = (const CString& value);
			operator CString() const;
		
			/**
			 *  @brief    点构造
			 *
			 *  @param    const CGePoint3d & value
			 */
			CAny(const osg::Vec3d& value);
			CAny& operator=(const osg::Vec3d& value);
			operator osg::Vec3d() const;

			CAny(const std::vector<osg::Vec3d>& values);
			CAny& operator=(const std::vector<osg::Vec3d>& value);
			operator std::vector<osg::Vec3d>() const;
			
			/**
			 *  @brief    矩阵构造
			 *
			 *  @param    const osg::Matrix & value
			 */
			CAny(const osg::Matrix& value);
			CAny& operator=(const osg::Matrix& value);
			operator osg::Matrix() const;

			/**
			*  @brief    字节流构造(使用请注意)
			*			 构造/设置时传入的BYTE*会进行拷贝。
			*			 获取时BYTE*&为指向CAny内部内存，不需要自己释放，但需要注意CAny生命周期。
			*			 以免BYTE*&内存过早释放
			*
			*  @param    const BYTE * value
			*  @param    int len 数据长度
			*/
			CAny(const BYTE* value, int len);
			void SetStreamValue(const BYTE* value, int len);
			int GetStreamValue(BYTE*& value) const;
			BYTE* BuildStreamValue(int len);

			/**
			* 字符串转点
			* @param [in] lpPoint
			* @return
			**/
			static osg::Vec3d StringToPoint(LPCWSTR lpPoint);
			static std::vector<osg::Vec3d> StringToPointDS(LPCWSTR lpPoint);
			/**
			 *  @brief    字符串转点
			 *
			 */
			static osg::Matrix StringToMatrix(LPCWSTR lpMatrix);
		protected:
			/**
			*  @brief    清理内存
			*
			*/
			void ClearTypeMemory();
	

			// Linux下字符串转换辅助函数
			static int64_t WStringToInt64(const wchar_t* str);
			static double WStringToDouble(const wchar_t* str);
			static long WStringToLong(const wchar_t* str);
		private:
			union vdValue
			{
				bool			valBool;	///< bool
				int				valInt;		///< int/BOOL/size_t
				double			valDouble;	///< double
				int64_t			valInt64;	///< int64
				WCHAR			*valString;	///< WCHAR*
				GUID			*valGuid;	///< GUID
				BYTE			*valStream;	///< BYTE*
				osg::Vec3d		*valPoint;		///< CGePoint3d
				std::vector<osg::Vec3d>* valPointds;		///< CGePoint3d
				osg::Matrix		*valMatrix;		///< Matrix
			}_vdValue;						///< Data值
			char _vdType;					///< Data类型
		};
	}
}

#endif // ANY_H_