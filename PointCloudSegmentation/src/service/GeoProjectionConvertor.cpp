#include <src/service/GeoProjectionConvertor.h>
#include <src/utils/logging.h>

namespace d3s {
	namespace pcs {

		CGeoProjectionConvertor::CGeoProjectionConvertor(int epsg) : _epsg(epsg)
		{
			// 初始化
			_geo_proj_convertor = std::make_shared<GeoProjectionConverter>();
			_geo_proj_convertor->set_epsg_code(_epsg);
		}

		bool CGeoProjectionConvertor::GetWktFromProjection(int& len, char** ogc_wkt, bool source)
		{
			CHECK(_geo_proj_convertor.get());

			if (!_geo_proj_convertor)
				return false;

			return _geo_proj_convertor->get_ogc_wkt_from_projection(len, ogc_wkt, source);
		}

		bool CGeoProjectionConvertor::HasProjection(bool source)
		{
			CHECK(_geo_proj_convertor.get());

			if (!_geo_proj_convertor)
				return false;

			return _geo_proj_convertor->has_projection(source);
		}

		bool CGeoProjectionConvertor::GetGeoKeysFromProjection(int& num_geo_keys,
															   GeoProjectionGeoKeys** geo_keys,
															   int& num_geo_double_params,
															   double** geo_double_params,
															   bool source)
		{
			CHECK(_geo_proj_convertor.get());

			if (!_geo_proj_convertor)
				return false;

			return _geo_proj_convertor->get_geo_keys_from_projection(num_geo_keys,
																	 geo_keys,
																	 num_geo_double_params,
																	 geo_double_params,
																	 source);
		}

	}
}
