//////////////////////////////////////////////////////////////////////
// 文件名称：GeoProjectionConvertor.h
// 功能描述：地理坐标转换接口
// 创建标识：吕伟	2022/12/14
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once

#include "../io/geoProjectionConverter.h"

namespace d3s {
	namespace pcs {
		
		class CGeoProjectionConvertor : public IGeoProjectionConvertor
		{
		public:	
			CGeoProjectionConvertor(int epsg);

			virtual ~CGeoProjectionConvertor() {}

			virtual bool GetWktFromProjection(int& len, char** ogc_wkt, bool source = true);

			virtual bool HasProjection(bool source = true);

			virtual bool GetGeoKeysFromProjection(int& num_geo_keys,
												  GeoProjectionGeoKeys** geo_keys,
												  int& num_geo_double_params,
												  double** geo_double_params,
												  bool source = true);

		protected:
			int _epsg;
			std::shared_ptr<GeoProjectionConverter> _geo_proj_convertor;
		};
	}
}
