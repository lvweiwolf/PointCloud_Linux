//////////////////////////////////////////////////////////////////////
// 文件名称：IGeoProjectionConvertor.h
// 功能描述：地理坐标转换接口
// 创建标识：吕伟	2022/12/14
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////

#ifndef IGEOPROJECTIONCONVERTOR_H_
#define IGEOPROJECTIONCONVERTOR_H_

#include <include/Export.h>
#include <include/share_ptr.h>
#include <include/ReferenceCountObj.h>

struct GeoProjectionGeoKeys
{
	unsigned short key_id;
	unsigned short tiff_tag_location;
	unsigned short count;
	unsigned short value_offset;
};

#define GEO_ELLIPSOID_AIRY 1
#define GEO_ELLIPSOID_BESSEL_1841 3
#define GEO_ELLIPSOID_BESSEL_NAMIBIA 4
#define GEO_ELLIPSOID_CLARKE1866 5
#define GEO_ELLIPSOID_CLARKE1880 6
#define GEO_ELLIPSOID_GRS1967 10
#define GEO_ELLIPSOID_GRS1980 11
#define GEO_ELLIPSOID_INTERNATIONAL 14
#define GEO_ELLIPSOID_KRASSOWSKY 15
#define GEO_ELLIPSOID_WGS72 22
#define GEO_ELLIPSOID_WGS84 23
#define GEO_ELLIPSOID_ID74 24

#define GEO_GCS_CH1903 4149
#define GEO_GCS_NAD83_HARN 4152
#define GEO_GCS_NZGD2000 4167
#define GEO_GCS_HD72 4237
#define GEO_GCS_ETRS89 4258
#define GEO_GCS_NAD27 4267
#define GEO_GCS_NAD83 4269
#define GEO_GCS_OSGB1936 4277
#define GEO_GCS_GDA94 4283
#define GEO_GCS_SAD69 4291
#define GEO_GCS_WGS72 4322
#define GEO_GCS_WGS72BE 4324
#define GEO_GCS_WGS84 4326
#define GEO_GCS_NAD83_CSRS 4617
#define GEO_GCS_SWEREF99 4619
#define GEO_GCS_NAD83_NSRS2007 4759
#define GEO_GCS_NAD83_2011 6318
#define GEO_GCS_NAD83_PA11 6322
#define GEO_GCS_NAD83_CORS96 6783
#define GEO_GCS_GDA2020 7844

#define GEO_SPHEROID_AIRY 7001
#define GEO_SPHEROID_BESSEL1841 7004
#define GEO_SPHEROID_CLARKE1866 7008
#define GEO_SPHEROID_GRS80 7019
#define GEO_SPHEROID_INTERNATIONAL 7022
#define GEO_SPHEROID_WGS84 7030
#define GEO_SPHEROID_GRS67 7036
#define GEO_SPHEROID_WGS72 7043

#define GEO_VERTICAL_WGS84 5030
#define GEO_VERTICAL_NGVD29 5102
#define GEO_VERTICAL_NAVD88 5103
#define GEO_VERTICAL_CGVD28 5114
#define GEO_VERTICAL_DVR90 5206
#define GEO_VERTICAL_EVRF2007 5215
#define GEO_VERTICAL_NN54 5776
#define GEO_VERTICAL_DHHN92 5783
#define GEO_VERTICAL_NN2000 5941
#define GEO_VERTICAL_CGVD2013 6647
#define GEO_VERTICAL_DHHN2016 7837
#define GEO_VERTICAL_NZVD2016 7839

#define GEO_VERTICAL_NAVD88_GEOID96 965103
#define GEO_VERTICAL_NAVD88_GEOID99 995103
#define GEO_VERTICAL_NAVD88_GEOID03 1035103
#define GEO_VERTICAL_NAVD88_GEOID06 1065103
#define GEO_VERTICAL_NAVD88_GEOID09 1095103
#define GEO_VERTICAL_NAVD88_GEOID12 1125103
#define GEO_VERTICAL_NAVD88_GEOID12A 1135103
#define GEO_VERTICAL_NAVD88_GEOID12B 1145103

class GeoProjectionEllipsoid
{
public:
	int id;
	char const* name;
	double equatorial_radius;
	double polar_radius;
	double eccentricity_squared;
	double inverse_flattening;
	double eccentricity_prime_squared;
	double eccentricity;
	double eccentricity_e1;
};

class GeoProjectionParameters
{
public:
	int type;
	short geokey;
	short datum;
	char name[256];
	GeoProjectionParameters()
	{
		type = -1;
		geokey = 0;
		datum = 0;
		name[0] = '\0';
	};
};

class GeoProjectionParametersUTM : public GeoProjectionParameters
{
public:
	int utm_zone_number;
	char utm_zone_letter;
	bool utm_northern_hemisphere;
	int utm_long_origin;
};

class GeoProjectionParametersLCC : public GeoProjectionParameters
{
public:
	double lcc_false_easting_meter;
	double lcc_false_northing_meter;
	double lcc_lat_origin_degree;
	double lcc_long_meridian_degree;
	double lcc_first_std_parallel_degree;
	double lcc_second_std_parallel_degree;
	double lcc_lat_origin_radian;
	double lcc_long_meridian_radian;
	double lcc_first_std_parallel_radian;
	double lcc_second_std_parallel_radian;
	double lcc_n;
	double lcc_aF;
	double lcc_rho0;
};

class GeoProjectionParametersTM : public GeoProjectionParameters
{
public:
	double tm_false_easting_meter;
	double tm_false_northing_meter;
	double tm_lat_origin_degree;
	double tm_long_meridian_degree;
	double tm_scale_factor;
	double tm_lat_origin_radian;
	double tm_long_meridian_radian;
	double tm_ap;
	double tm_bp;
	double tm_cp;
	double tm_dp;
	double tm_ep;
};

class GeoProjectionParametersAEAC : public GeoProjectionParameters
{
public:
	double aeac_false_easting_meter;
	double aeac_false_northing_meter;
	double aeac_latitude_of_center_degree;
	double aeac_longitude_of_center_degree;
	double aeac_first_std_parallel_degree;
	double aeac_second_std_parallel_degree;
	double aeac_latitude_of_center_radian;
	double aeac_longitude_of_center_radian;
	double aeac_first_std_parallel_radian;
	double aeac_second_std_parallel_radian;
	double aeac_n;
	double aeac_C;
	double aeac_two_es;
	double aeac_rho0;
	double aeac_one_MINUS_es2;
	double aeac_Albers_a_OVER_n;
};

class GeoProjectionParametersHOM : public GeoProjectionParameters
{
public:
	double hom_false_easting_meter;
	double hom_false_northing_meter;
	double hom_latitude_of_center_degree;
	double hom_longitude_of_center_degree;
	double hom_azimuth_degree;
	double hom_rectified_grid_angle_degree;
	double hom_scale_factor;
	double hom_latitude_of_center_radian;
	double hom_longitude_of_center_radian;
	double hom_azimuth_radian;
	double hom_rectified_grid_angle_radian;

	double hom_A;
	double hom_B;
	double hom_H;
	double hom_g0;
	double hom_l0;
};

class GeoProjectionParametersOS : public GeoProjectionParameters
{
public:
	double os_false_easting_meter;
	double os_false_northing_meter;
	double os_lat_origin_degree;
	double os_long_meridian_degree;
	double os_scale_factor;
	double os_lat_origin_radian;
	double os_long_meridian_radian;
	double os_R2;
	double os_C;
	double os_phic0;
	double os_sinc0;
	double os_cosc0;
	double os_ratexp;
	double os_K;
	double os_gf;
};

namespace d3s {
	namespace pcs {

		// 地理坐标转换
		class IGeoProjectionConvertor : public d3s::ReferenceCountObj
		{
		public:
			virtual ~IGeoProjectionConvertor() {}

			virtual bool GetWktFromProjection(int& len, char** ogc_wkt, bool source = true) = 0;

			virtual bool HasProjection(bool source = true) = 0;

			virtual bool GetGeoKeysFromProjection(int& num_geo_keys,
												  GeoProjectionGeoKeys** geo_keys,
												  int& num_geo_double_params,
												  double** geo_double_params,
												  bool source = true) = 0;
		};
	}
}

typedef d3s::share_ptr<d3s::pcs::IGeoProjectionConvertor> IGeoProjectionConvertorPtr;


#endif // IGEOPROJECTIONCONVERTOR_H_