/******************************************
 * 文件名称：SpatialRef.h
 * 功能描述：空间坐标系描述对象，用于各类坐标参数的记录和坐标转换
 *
 * 创建标识： 欧阳俊杰 2019.3.1
 *
 */
#ifndef SPATIALREF_H_
#define SPATIALREF_H_

#include <include/cstring.h>
#include <include/CommonToolsDef.h>
#include <include/PointCloudManagerExport.h>

#include <osg/Vec3d>

#include <map>

// 默认为wgs84坐标系

namespace pc {
	namespace tool {
		/*以下仅用于缩小存储数据使用*/
		static LPCTSTR WebMercator_Short_wkt = L"WebMercator";
		static LPCTSTR SphereWgs84_Short_wkt = L"WGS84";
		/*以上仅用于缩小存储数据使用*/

		class POINTCLOUDMANAGER_EXPORT CSpatialRef
		{
		public:
			/*
			 * 函数介绍：查找空间地理坐标系
			 * 输入参数：LPCTSTR szWkt wkt文本描述
			 * 返回值：CSpatialRef*
			 */
			static CSpatialRef* FindSpatialRef(LPCTSTR szWkt);
			static CSpatialRef* FindSpatialRef(int nEPSG);
			static void ClearAll();

			static LPCTSTR WebMercatorWkt;
			static LPCTSTR SphereWgs84;
			static LPCTSTR CGCS2000;
			static int Wgs84EPSG;

		public:
			/*
			 * 函数介绍：转换坐标系
			 * 输入参数：const pc::Vec3d &srcPnt 源坐标系下坐标
			 * 输入参数：CSpatialRef* pSrcSpatRef 源坐标系
			 * 返回值：pc::Vec3d 当前坐标系下
			 */
			osg::Vec3d ToThisSpatialRef(const osg::Vec3d& srcPnt, const CSpatialRef* pSrcSpatRef);

			/**
			 *  函数介绍： 转换坐标数组
			 *
			 *  输入参数： const GeoPointList & srcInPntList 源坐标数组
			 *  输出参数： GeoPointList & destOutPntList     输出转换后的坐标
			 *  输入参数： const CSpatialRef * pSrcSpatRef   源坐标系
			 */
			void ConvertCoordinate(const std::vector<osg::Vec3d>& srcInPntList,
								   std::vector<osg::Vec3d>& destOutPntList,
								   const CSpatialRef* pSrcSpatRef);

			LPCTSTR GetWkt() const { return _strWktDescr; }
			int GetEPSG() const { return _nEPSGCode; }

		protected:
			CSpatialRef(LPCTSTR szWtk);
			CSpatialRef(int nEPSG);
			~CSpatialRef();

		protected:
			CString _strWktDescr; // wkt描述符
			int _nEPSGCode;		  // epsg编码
			void* _sptRef;		  // 空间坐标系索引

			static std::map<CString, CSpatialRef*> s_Wkt2SpatialRefDic;
			static std::map<void*, CSpatialRef*> s_RefH2SpatialRefDic;
			static std::map<int, CSpatialRef*> s_EPSG2SpatialRefDic; // EPSG编码对坐标系
		};
	}
}

#endif // SPATIALREF_H_