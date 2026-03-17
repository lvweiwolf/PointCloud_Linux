//////////////////////////////////////////////////////////////////////
// 文件名称：geoProjectionConverter.h
// 功能描述：地理坐标转换工具
// 创建标识：吕伟	2022/11/7
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef GEO_PROJECTION_CONVERTER_H_
#define GEO_PROJECTION_CONVERTER_H_

#include <include/IGeoProjectionConvertor.h>

class GeoProjectionConverter
{
public:
	// parse command line arguments

	bool parse(int argc, char* argv[]);
	int unparse(char* string) const;

	// set & get current projection

	bool set_projection_from_geo_keys(int num_geo_keys,
									  const GeoProjectionGeoKeys* geo_keys,
									  char* geo_ascii_params,
									  double* geo_double_params,
									  char* description = 0,
									  bool source = true);
	bool get_geo_keys_from_projection(int& num_geo_keys,
									  GeoProjectionGeoKeys** geo_keys,
									  int& num_geo_double_params,
									  double** geo_double_params,
									  bool source = true);
	bool set_projection_from_ogc_wkt(const char* ogc_wkt, char* description = 0);
	bool get_ogc_wkt_from_projection(int& len, char** ogc_wkt, bool source = true);
	bool get_prj_from_projection(int& len, char** prj, bool source = true);
	bool get_proj4_string_from_projection(int& len, char** proj4, bool source = true);

	short get_GTRasterTypeGeoKey() const;
	short get_GeographicTypeGeoKey() const;
	short get_GeogGeodeticDatumGeoKey() const;
	short get_GeogPrimeMeridianGeoKey() const;
	short get_GeogLinearUnitsGeoKey() const;
	double get_GeogLinearUnitSizeGeoKey() const;
	short get_GeogAngularUnitsGeoKey() const;
	double get_GeogAngularUnitSizeGeoKey() const;
	double get_GeogSemiMajorAxisGeoKey() const;
	double get_GeogSemiMinorAxisGeoKey() const;
	double get_GeogInvFlatteningGeoKey() const;
	short get_GeogAzimuthUnitsGeoKey() const;
	double get_GeogPrimeMeridianLongGeoKey() const;

	bool set_GTModelTypeGeoKey(short value, char* description = 0);
	short get_GTModelTypeGeoKey() const;

	bool set_ProjectedCSTypeGeoKey(short value, char* description = 0, bool source = true);
	short get_ProjectedCSTypeGeoKey(bool source = true) const;

	int set_GeogEllipsoidGeoKey(short value);
	short get_GeogEllipsoidGeoKey() const;

	bool set_ProjLinearUnitsGeoKey(short value, bool source = true);
	short get_ProjLinearUnitsGeoKey(bool source = true) const;

	bool set_VerticalUnitsGeoKey(short value);
	short get_VerticalUnitsGeoKey(bool source = true) const;

	bool set_VerticalCSTypeGeoKey(short value, char* description = 0);
	short get_VerticalCSTypeGeoKey();

	bool set_reference_ellipsoid(int id, char* description = 0);
	int get_ellipsoid_id() const;
	const char* get_ellipsoid_name() const;

	bool set_gcs(short code, char* description = 0);
	const char* get_gcs_name() const;

	bool set_no_projection(char* description = 0, bool source = true);
	bool set_latlong_projection(char* description = 0, bool source = true);
	bool set_longlat_projection(char* description = 0, bool source = true);
	bool is_longlat_projection(bool source = true) const;

	bool set_ecef_projection(char* description, bool source = true, const char* name = 0);
	bool is_ecef_projection(bool source = true) const;

	bool set_target_utm_projection(char* description, const char* name = 0);
	bool set_utm_projection(char* zone,
							char* description = 0,
							bool source = true,
							const char* name = 0,
							bool is_mga = false);
	bool set_utm_projection(int zone,
							bool northern,
							char* description = 0,
							bool source = true,
							const char* name = 0,
							bool is_mga = false);
	void set_lambert_conformal_conic_projection(double falseEastingMeter,
												double falseNorthingMeter,
												double latOriginDegree,
												double longMeridianDegree,
												double firstStdParallelDegree,
												double secondStdParallelDegree,
												char* description = 0,
												bool source = true,
												const char* name = 0);
	void set_transverse_mercator_projection(double falseEastingMeter,
											double falseNorthingMeter,
											double latOriginDegree,
											double longMeridianDegree,
											double scaleFactor,
											char* description = 0,
											bool source = true,
											const char* name = 0);
	void set_albers_equal_area_conic_projection(double falseEastingMeter,
												double falseNorthingMeter,
												double latCenterDegree,
												double longCenterDegree,
												double firstStdParallelDegree,
												double secondStdParallelDegree,
												char* description = 0,
												bool source = true,
												const char* name = 0);
	void set_hotine_oblique_mercator_projection(double falseEastingMeter,
												double falseNorthingMeter,
												double latCenterDegree,
												double longCenterDegree,
												double azimuthDegree,
												double rectifiedGridAngleDegree,
												double scaleFactor,
												char* description = 0,
												bool source = true,
												const char* name = 0);
	void set_oblique_stereographic_projection(double falseEastingMeter,
											  double falseNorthingMeter,
											  double latOriginDegree,
											  double longMeridianDegree,
											  double scaleFactor,
											  char* description = 0,
											  bool source = true,
											  const char* name = 0);

	const char* get_state_plane_nad27_lcc_zone(int i) const;
	bool set_state_plane_nad27_lcc(const char* zone,
								   char* description = 0,
								   bool source = true,
								   const char* name = 0);
	void print_all_state_plane_nad27_lcc() const;
	const char* get_state_plane_nad83_lcc_zone(int i) const;
	bool set_state_plane_nad83_lcc(const char* zone,
								   char* description = 0,
								   bool source = true,
								   const char* name = 0);
	void print_all_state_plane_nad83_lcc() const;

	const char* get_state_plane_nad27_tm_zone(int i) const;
	bool set_state_plane_nad27_tm(const char* zone,
								  char* description = 0,
								  bool source = true,
								  const char* name = 0);
	void print_all_state_plane_nad27_tm() const;
	const char* get_state_plane_nad83_tm_zone(int i) const;
	bool set_state_plane_nad83_tm(const char* zone,
								  char* description = 0,
								  bool source = true,
								  const char* name = 0);
	void print_all_state_plane_nad83_tm() const;

	bool set_epsg_code(short code, char* description = 0, bool source = true);

	void reset_projection(bool source = true);
	bool has_projection(bool source = true) const;
	const char* get_projection_name(bool source = true) const;

	void set_coordinates_in_survey_feet(bool source = true);
	void set_coordinates_in_feet(bool source = true);
	void set_coordinates_in_meter(bool source = true);

	void reset_coordinate_units(bool source = true);
	bool has_coordinate_units(bool source = true) const;
	const char* get_coordinate_unit_description_string(bool abrev = true, bool source = true) const;

	void set_elevation_in_survey_feet(bool source = true);
	void set_elevation_in_feet(bool source = true);
	void set_elevation_in_meter(bool source = true);

	void reset_elevation_units(bool source = true);
	bool has_elevation_units(bool source = true) const;
	const char* get_elevation_unit_description_string(bool abrev = true, bool source = true);

	void set_elevation_offset_in_meter(float elevation_offset);

	// specific conversion routines

	bool compute_utm_zone(const double LatDegree,
						  const double LongDegree,
						  GeoProjectionParametersUTM* utm) const;
	bool UTMtoLL(const double UTMEastingMeter,
				 const double UTMNorthingMeter,
				 double& LatDegree,
				 double& LongDegree,
				 const GeoProjectionEllipsoid* ellipsoid,
				 const GeoProjectionParametersUTM* utm) const;
	bool LLtoUTM(const double LatDegree,
				 const double LongDegree,
				 double& UTMEastingMeter,
				 double& UTMNorthingMeter,
				 const GeoProjectionEllipsoid* ellipsoid,
				 const GeoProjectionParametersUTM* utm) const;

	bool LCCtoLL(const double LCCEastingMeter,
				 const double LCCNorthingMeter,
				 double& LatDegree,
				 double& LongDegree,
				 const GeoProjectionEllipsoid* ellipsoid,
				 const GeoProjectionParametersLCC* lcc) const;
	bool LLtoLCC(const double LatDegree,
				 const double LongDegree,
				 double& LCCEastingMeter,
				 double& LCCNorthingMeter,
				 const GeoProjectionEllipsoid* ellipsoid,
				 const GeoProjectionParametersLCC* lcc) const;

	bool TMtoLL(const double TMEastingMeter,
				const double TMNorthingMeter,
				double& LatDegree,
				double& LongDegree,
				const GeoProjectionEllipsoid* ellipsoid,
				const GeoProjectionParametersTM* tm) const;
	bool LLtoTM(const double LatDegree,
				const double LongDegree,
				double& TMEastingMeter,
				double& TMNorthingMeter,
				const GeoProjectionEllipsoid* ellipsoid,
				const GeoProjectionParametersTM* tm) const;

	bool ECEFtoLL(const double ECEFMeterX,
				  const double ECEFMeterY,
				  const double ECEFMeterZ,
				  double& LatDegree,
				  double& LongDegree,
				  double& ElevationMeter,
				  const GeoProjectionEllipsoid* ellipsoid) const;
	bool LLtoECEF(const double LatDegree,
				  const double LongDegree,
				  const double ElevationMeter,
				  double& ECEFMeterX,
				  double& ECEFMeterY,
				  double& ECEFMeterZ,
				  const GeoProjectionEllipsoid* ellipsoid) const;

	bool AEACtoLL(const double AEACEastingMeter,
				  const double AEACNorthingMeter,
				  double& LatDegree,
				  double& LongDegree,
				  const GeoProjectionEllipsoid* ellipsoid,
				  const GeoProjectionParametersAEAC* aeac) const;
	bool LLtoAEAC(const double LatDegree,
				  const double LongDegree,
				  double& AEACEastingMeter,
				  double& AEACNorthingMeter,
				  const GeoProjectionEllipsoid* ellipsoid,
				  const GeoProjectionParametersAEAC* aeac) const;

	bool HOMtoLL(const double HOMEastingMeter,
				 const double HOMNorthingMeter,
				 double& LatDegree,
				 double& LongDegree,
				 const GeoProjectionEllipsoid* ellipsoid,
				 const GeoProjectionParametersHOM* hom) const;
	bool LLtoHOM(const double LatDegree,
				 const double LongDegree,
				 double& HOMEastingMeter,
				 double& HOMNorthingMeter,
				 const GeoProjectionEllipsoid* ellipsoid,
				 const GeoProjectionParametersHOM* hom) const;

	bool OStoLL(const double OSEastingMeter,
				const double OSNorthingMeter,
				double& LatDegree,
				double& LongDegree,
				const GeoProjectionEllipsoid* ellipsoid,
				const GeoProjectionParametersOS* os) const;
	bool LLtoOS(const double LatDegree,
				const double LongDegree,
				double& OSEastingMeter,
				double& OSNorthingMeter,
				const GeoProjectionEllipsoid* ellipsoid,
				const GeoProjectionParametersOS* os) const;

	GeoProjectionConverter();
	~GeoProjectionConverter();

	// check before any reprojection

	bool check_horizontal_datum_before_reprojection();

	// from current projection to longitude/latitude/elevation_in_meter

	bool to_lon_lat_ele(double* point) const;
	bool to_lon_lat_ele(const double* point,
						double& longitude,
						double& latitude,
						double& elevation_in_meter) const;

	// from current projection to target projection

	bool to_target(double* point) const;
	bool to_target(const double* point, double& x, double& y, double& elevation) const;

	bool has_target_precision() const;
	double get_target_precision() const;
	void set_target_precision(double target_precision);

	bool has_target_elevation_precision() const;
	double get_target_elevation_precision() const;
	void set_target_elevation_precision(double target_elevation_precision);

	// for interfacing with common geo-spatial formats

	//  int get_img_projection_number(bool source=true) const;
	bool get_dtm_projection_parameters(short* horizontal_units,
									   short* vertical_units,
									   short* coordinate_system,
									   short* coordinate_zone,
									   short* horizontal_datum,
									   short* vertical_datum,
									   bool source = true);
	bool set_dtm_projection_parameters(short horizontal_units,
									   short vertical_units,
									   short coordinate_system,
									   short coordinate_zone,
									   short horizontal_datum,
									   short vertical_datum,
									   bool source = true);

	// helps us to find the 'pcs.csv' file
	char* argv_zero;

private:
	// parameters for gtiff
	int num_geo_keys;
	GeoProjectionGeoKeys* geo_keys;
	char* geo_ascii_params;
	double* geo_double_params;

	// codes and names according to EPSG
	int gcs_code;
	char gcs_name[28];
	int datum_code;
	char datum_name[60];
	int spheroid_code;

	// parameters for the reference ellipsoid
	GeoProjectionEllipsoid* ellipsoid;

	// parameters for the projection
	GeoProjectionParameters* source_projection;
	GeoProjectionParameters* target_projection;

	// vertical coordinate system
	short vertical_geokey;
	int vertical_geoid;

	// parameters for coordinate scaling
	bool coordinate_units_set[2];
	double coordinates2meter, meter2coordinates;
	bool elevation_units_set[2];
	double elevation2meter, meter2elevation;
	float elevation_offset_in_meter;

	double target_precision;
	double target_elevation_precision;

	// helper functions
	void set_projection(GeoProjectionParameters* projection, bool source);
	void set_geokey(short geokey, bool source);
	void check_geokey(short geokey, bool source);
	GeoProjectionParameters* get_projection(bool source) const;
	void compute_lcc_parameters(bool source);
	void compute_tm_parameters(bool source);
	void compute_aeac_parameters(bool source);
	void compute_hom_parameters(bool source);
	void compute_os_parameters(bool source);
};

#endif // GEO_PROJECTION_CONVERTER_H_