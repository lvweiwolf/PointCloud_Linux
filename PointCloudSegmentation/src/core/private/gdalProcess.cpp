//stdafx.h
#include "gdalProcess.h"
#include "../../7z/7z.h"
#include <mutex>
#include <memory>

#include <libgen.h> 
#include <string.h>

#include <gdal.h>
#include <gdal_priv.h>
#include <gdal_utils.h>
#include <gdalwarper.h>
#include <ogr_spatialref.h>
#include <ogrsf_frmts.h>
#include <cpl_conv.h>

#include <osgEarth/GeoData>
#include <osgEarth/TileKey>
#include <osgEarth/Profile>
#include <osgEarth/SpatialReference>

#include "../../utils/endian.h"
#include "../../utils/logging.h"
#include "../../utils/timer.h"
#include "../../utils/misc.h"
#include "../../Platform.h"
//#include "Toolkit/7z/7z.h"
#ifndef RGB
#define RGB(r,g,b)          ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))
#endif // !RGB
//#include "Toolkit/include.h"	

extern std::string DebugDirectory;

typedef osgEarth::TileKey TileKey;
typedef osgEarth::Profile Profile;
typedef osgEarth::GeoExtent GeoExtent;
typedef osgEarth::SpatialReference SpatialReference;

namespace {
	/**
	 *  @brief    像素坐标转地理坐标
	 *
	 *  @prarm	 double * geoTransform		坐标系变换矩阵
	 *  @prarm	 double x					像素坐标 X
	 *  @prarm	 double y					像素坐标 Y
	 *  @prarm	 double & geoX				地理坐标 X
	 *  @prarm	 double & geoY				地理坐标 Y
	 *
	 *  @return   void
	 */
	void pixelToGeo(double* geoTransform, double x, double y, double& geoX, double& geoY)
	{
		geoX = geoTransform[0] + geoTransform[1] * x + geoTransform[2] * y;
		geoY = geoTransform[3] + geoTransform[4] * x + geoTransform[5] * y;
	}

	/**
	 *  @brief    地理坐标转换像素坐标
	 *
	 *  @prarm	 double * invtTransform		坐标系变换逆矩阵
	 *  @prarm	 double geoX				地理坐标 X
	 *  @prarm	 double geoY				地理坐标 Y
	 *  @prarm	 double & x					像素坐标 X
	 *  @prarm	 double & y					像素坐标 Y
	 *
	 *  @return   void
	 */
	void geoToPixel(double* invtTransform,
					int width,
					int height,
					double geoX,
					double geoY,
					double& x,
					double& y)
	{
		x = invtTransform[0] + invtTransform[1] * geoX + invtTransform[2] * geoY;
		y = invtTransform[3] + invtTransform[4] * geoX + invtTransform[5] * geoY;

		double eps = 0.0001;
		if (osg::equivalent(x, 0, eps))
			x = 0;
		if (osg::equivalent(y, 0, eps))
			y = 0;
		if (osg::equivalent(x, (double)width, eps))
			x = width;
		if (osg::equivalent(y, (double)height, eps))
			y = height;
	}
}


namespace d3s {
	namespace pcs {

		static std::once_flag flag;

		void initializeGDAL()
		{
			auto init = []() -> void {
				GDALAllRegister();
				OGRRegisterAll();

				CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
				CPLSetConfigOption("SHAPE_ENCODING", "");
				//CPLSetConfigOption("GDAL_CACHEMAX", "0");

				#ifdef WIN32
				char szModulePath[MAX_PATH];
				GetModuleFileNameA(nullptr, szModulePath, MAX_PATH);
				// Windows 特定的处理逻辑
				#else
				// Linux/macOS 的替代方案
				char szModulePath[PATH_MAX];
				ssize_t count = readlink("/proc/self/exe", szModulePath, PATH_MAX);
				if (count != -1) {
					szModulePath[count] = '\0';
				}
				else {
					// 错误处理
					strcpy(szModulePath, ".");
				}
				#endif

				#ifdef WIN32
				PathRemoveFileSpecA(szModulePath);
				#else
				char* dir = dirname(szModulePath);
				strcpy(szModulePath, dir);
				#endif
				std::string gdaldata = std::string(szModulePath) + R"(\system\gdal_data)";

				CPLSetConfigOption("GDAL_DATA", gdaldata.c_str());
			};

			std::call_once(flag, init);
		}

		void VectorLineString::Write(std::ostream& stream)
		{
			// epsg
			// WriteBinaryLittleEndian<int>(&stream, epsg);

			// Bound
			WriteBinaryLittleEndian<float>(&stream, bound.xMin());
			WriteBinaryLittleEndian<float>(&stream, bound.yMin());
			WriteBinaryLittleEndian<float>(&stream, bound.xMax());
			WriteBinaryLittleEndian<float>(&stream, bound.yMax());

			// verts size
			WriteBinaryLittleEndian<size_t>(&stream, verts.size());

			// verts
			for (size_t i = 0; i < verts.size(); ++i)
			{
				const osg::Vec2& vert = verts.at(i);

				WriteBinaryLittleEndian<float>(&stream, vert.x());
				WriteBinaryLittleEndian<float>(&stream, vert.y());
			}
		}

		void VectorLineString::Read(std::istream& stream)
		{
			// epsg
			// epsg = ReadBinaryLittleEndian<int>(&stream);

			// Bound
			double xMin = ReadBinaryLittleEndian<float>(&stream);
			double yMin = ReadBinaryLittleEndian<float>(&stream);
			double xMax = ReadBinaryLittleEndian<float>(&stream);
			double yMax = ReadBinaryLittleEndian<float>(&stream);

			bound = BoundingBox2D(xMin, yMin, xMax, yMax);

			// verts size
			size_t nVerts = ReadBinaryLittleEndian<size_t>(&stream);

			// verts
			verts.clear();
			for (size_t i = 0; i < nVerts; ++i)
			{
				double x = ReadBinaryLittleEndian<float>(&stream);
				double y = ReadBinaryLittleEndian<float>(&stream);
				verts.push_back(osg::Vec2(x, y));
			}
		}

		size_t VectorLineString::GetBytes()
		{
			return (sizeof(bound) + verts.size() * sizeof(osg::Vec2));
		}

		// GDAL 相关函数
		//////////////////////////////////////////////////////////////////////////
		std::string getWktFromEPSGCode(unsigned int epsg)
		{
			initializeGDAL(); // 初始化GDAL

			OGRSpatialReference srs;

			OGRErr err = srs.importFromEPSG(epsg);
			CHECK(err == OGRERR_NONE);

			char* pszWKT = NULL;
			err = srs.exportToWkt(&pszWKT);
			CHECK(err == OGRERR_NONE);

			return std::string(pszWKT);
		}

		void coordinateSystemTransform(int epsg_src, int epsg_dst, double* x, double* y, double* z)
		{
			initializeGDAL(); // 初始化GDAL

			OGRCoordinateTransformation* coordTrans = nullptr;

			OGRSpatialReference source, dest;

			OGRErr err = source.importFromEPSG(epsg_src);
			if (err != OGRERR_NONE)
				return;

			err = dest.importFromEPSG(epsg_dst);
			if (err != OGRERR_NONE)
				return;

			coordTrans = OGRCreateCoordinateTransformation(&source, &dest);
			if (!coordTrans)
				return;

			coordTrans->Transform(1, x, y, z);
		}

		const std::string road_fclass[] = {
			"tertiary",		  //	第三级道路
			"tertiary_link",  // 第三季道路-连接
			"residential",	  // 居住区道路
			"unclassified",	  // 未分类道路
			"secondary",	  // 次要道路
			"secondary_link", // 次要道路-连接
			"primary",		  // 主要道路
			"primary_link",	  // 主要道路-连接
			"motorway",		  // 高速公路
			"motorway_link",  // 高速公路-连接
			"trunk",		  // 干道
			"trunk_link",	  // 干道-连接
			"track",		  // 小路
			"track_grade1",	  // 小路 级别1
			"track_grade2",	  // 小路 级别2
			"track_grade3",	  // 小路 级别3
			"track_grade4",	  // 小路 级别4
			"track_grade5",	  // 小路 级别5
			"bridleway",	  // 马道
			"living_street",  // 生活街道
			"path",			  // 小道
			"service",		  // 服务性道路
			"footway",		  // 人行道
			"pedestrian",	  // 步行街道
			"steps",		  // 台阶踏步
			"cycleway",		  // 自行车道
			"unknown"		  // 未知道路
		};

		const std::string rail_fclass[] = {
			"funicular",		 // 索道
			"light_rail",		 // 轻轨
			"miniature_railway", // 微型铁路
			"monorail",			 // 单轨
			"narrow_gauge",		 // 窄轨
			"rail",				 // 常规铁路
			"subway",			 // 地铁
			"tram"				 // 有轨电车轨道
		};


		void readRoadDataset(const std::string& shapefile,
							 std::vector<VectorLineString*>& roadDS,
							 int& epsg)
		{
			initializeGDAL(); // 初始化GDAL

			Timer timer;
			timer.Start();

			GDALDataset* poDS = nullptr;

			poDS = (GDALDataset*)
				GDALOpenEx(shapefile.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);

			if (poDS == nullptr)
			{
				PCS_ERROR("[readRoadDataset] GDAL 打开文件失败.");
				return;
			}

			// 投影坐标系
			int nFeatureRead = 0;

			OGRLayer* poLayer = poDS->GetLayer(0);
			OGRFeatureDefn* poFDefn = poLayer->GetLayerDefn();
			epsg = poLayer->GetSpatialRef()->GetEPSGGeogCS();

			std::string strLayerFields;
			int fclass_field_index = -1;

			// 获取属性信息
			for (int iField = 0; iField < poFDefn->GetFieldCount(); iField++)
			{
				OGRFieldDefn* poFieldDefn = poFDefn->GetFieldDefn(iField);

				std::string fieldname = poFieldDefn->GetNameRef();

				if (fieldname == std::string("fclass"))
					fclass_field_index = iField;

				strLayerFields += fieldname;

				if (iField != poFDefn->GetFieldCount() - 1)
					strLayerFields += std::string("，");
			}

			PCS_INFO("Filed Names: %s", strLayerFields.c_str());

			if (fclass_field_index < 0)
			{
				GDALClose(poDS);
				return;
			}

			OGRFeature* poFeature = nullptr;
			uint64_t nFeatures = poLayer->GetFeatureCount();
			PCS_INFO("Features Count: %d", nFeatures);

			// 遍历层中要素
			while ((poFeature = poLayer->GetNextFeature()) != NULL)
			{
				std::string fclass = poFeature->GetFieldAsString(fclass_field_index);

				if (fclass == "motorway" || fclass == "motorway_link" || fclass == "primary" ||
					fclass == "primary_link" || fclass == "secondary" ||
					fclass == "secondary_link" || fclass == "tertiary" || fclass == "tertiary_link")
				{

					OGRGeometry* poGeometry = poFeature->GetGeometryRef();
					if (poGeometry == nullptr)
						continue;

					// 获得包围框
					OGREnvelope envelope;
					poGeometry->getEnvelope(&envelope);

					if (wkbFlatten(poGeometry->getGeometryType()) == wkbLineString)
					{
						VectorLineString* lineString = new VectorLineString();

						OGRLineString* poLineString = (OGRLineString*)poGeometry;
						int nPoints = poLineString->getNumPoints();

						for (int j = 0; j < nPoints; ++j)
						{
							double dX = poLineString->getX(j);
							double dY = poLineString->getY(j);

							lineString->verts.push_back(osg::Vec2(dX, dY));
							lineString->bound.expandBy(osg::Vec2(dX, dY));
						}

						roadDS.push_back(lineString);
						++nFeatureRead;
					}
					else if (wkbFlatten(poGeometry->getGeometryType()) == wkbPolygon)
					{
						PCS_INFO("多边形，跳过.");
					}
					else
					{
						PCS_INFO("其他类型 %d，跳过.", wkbFlatten(poGeometry->getGeometryType()));
					}
				}

				OGRFeature::DestroyFeature(poFeature);
			}

			GDALClose(poDS);

			PCS_INFO("已读取 %d, 用时 %.3f s.", nFeatureRead, timer.ElapsedSeconds());
		}

		class membuf : public std::basic_streambuf<char>
		{
		public:
			membuf(const uint8_t* p, size_t l) { setg((char*)p, (char*)p, (char*)p + l); }
		};

		class memstream : public std::istream
		{
		public:
			memstream(const uint8_t* p, size_t l) : std::istream(&_buffer), _buffer(p, l)
			{
				rdbuf(&_buffer);
			}

		private:
			membuf _buffer;
		};

		void readRoadDataset2(const std::string& filename,
							  std::vector<VectorLineString*>& roadDS,
							  int& epsg)
		{
			if (filename.empty() || !ExistsPath(filename))
				return;

			Timer timer;
			timer.Start();

			std::map<std::wstring, std::vector<unsigned char>> fileMap;
			// 7z 解压数据到内存
			std::wstring filePath = CA2T(filename.c_str());
			toolkit::C7z::DecompressInfo(filePath, fileMap);
			std::vector<unsigned char>& buffer = fileMap[L"Road.dat"];

			memstream stream(&buffer[0], buffer.size());

			// 读取EPSG
			epsg = ReadBinaryLittleEndian<int>(&stream);

			// 读取道路矢量数据
			size_t nSize = ReadBinaryLittleEndian<size_t>(&stream);

			if (nSize > 0)
			{
				// 销毁内存
				for (size_t i = 0; i < roadDS.size(); ++i)
				{
					VectorLineString* road = roadDS.at(i);

					if (road)
						delete road;
				}

				roadDS.clear();

				for (size_t i = 0; i < nSize; ++i)
				{
					VectorLineString* road = new VectorLineString();
					road->Read(stream);

					roadDS.push_back(road);
				}
			}

			PCS_INFO("已读取 %d, 用时 %.3f s.", roadDS.size(), timer.ElapsedSeconds());
		}

		void writeRoadDataset(const std::vector<VectorLineString*>& roadDS,
							  int epsg,
							  const std::string& filename)
		{
			if (roadDS.empty())
				return;

			std::map<std::wstring, std::vector<unsigned char>> fileMap;
			std::vector<unsigned char>& buffer = fileMap[L"Road.dat"];

			{
				std::ostringstream stream(std::ios::binary);

				// 写入坐标系EPSG
				WriteBinaryLittleEndian<int>(&stream, epsg);

				// 写入数量
				size_t nSize = roadDS.size();
				WriteBinaryLittleEndian<size_t>(&stream, nSize);

				for (size_t i = 0; i < nSize; ++i)
				{
					VectorLineString* road = roadDS.at(i);
					CHECK(road);

					road->Write(stream);
				}

				size_t datasize = stream.str().size();
				buffer.resize(datasize);

				// 内存数据拷贝到 std::vector<unsigned char>
				memcpy(&buffer[0], stream.str().c_str(), datasize);
			}

			// 7z 压缩内存数据，写入到文件
			std::wstring compressFilePath = CA2T(filename.c_str());
			toolkit::C7z::Compress(fileMap, compressFilePath);
		}

		static std::atomic<int> post (0);

		std::shared_ptr<DOMDataset> createDOMDataset(const std::string& filename, bool shared)
		{
			initializeGDAL(); // 初始化GDAL

			std::shared_ptr<DOMDataset> ds;

			const char* drivers[2] = { "GTiff", 0 };
			unsigned int openflags = (shared ? GDAL_OF_SHARED : GDAL_OF_READONLY) | GDAL_OF_RASTER;

			GDALDataset* poDS =
				(GDALDataset*)GDALOpenEx(filename.c_str(), openflags, drivers, nullptr, nullptr);

			if (poDS == nullptr)
			{
				PCS_ERROR("[createDOMDataset] GDAL 打开文件失败.");
				return ds;
			}

			ds = std::make_shared<DOMDataset>();

			ds->srs = poDS->GetProjectionRef();
			ds->width = poDS->GetRasterXSize();
			ds->height = poDS->GetRasterYSize();


			poDS->GetGeoTransform(ds->geotransform);
			const double* gt = ds->geotransform;

			ds->xmin = gt[0];
			ds->xmax = gt[0] + (ds->width - 1) * gt[1] + (ds->height - 1) * gt[2];
			ds->ymin = gt[3] + (ds->width - 1) * gt[4] + (ds->height - 1) * gt[5];
			ds->ymax = gt[3];
			ds->hDS = (GDALDatasetH)poDS;

			return ds;
		}

		std::shared_ptr<d3s::pcs::DOMDataset> createDOMDataset(
			const std::vector<std::string>& files)
		{
			initializeGDAL(); // 初始化GDAL

			std::shared_ptr<DOMDataset> ds;

			CPLStringList fileList;
			for (const auto& filename : files)
				fileList.AddString(filename.c_str());

			// 初始化构建VRT虚拟数据集参数
			char** argv = nullptr;
			argv = CSLAddString(argv, "-resolution");
			argv = CSLAddString(argv, "highest");
			GDALBuildVRTOptions* psOptions = GDALBuildVRTOptionsNew(argv, nullptr);
			CSLDestroy(argv);

			#ifdef WIN32
			// Windows版本
			CString cstrVrtFile;
			cstrVrtFile.Format(_T("%stemp_%s.vrt"),
				toolkit::CLibToolkit::GetSoftTempPath(),
				toolkit::CLibToolkit::CreateGuid());
			std::string strVrtFile = (LPCSTR)CT2A(cstrVrtFile);
			#else
			// Linux版本
			std::string tempPath = CLibToolkit::GetSoftTempPath();
			std::string guid = CLibToolkit::CreateGuid();
			std::stringstream ss;
			ss << tempPath << "temp_" << guid << ".vrt";
			std::string strVrtFile = ss.str();
			#endif
			GDALDatasetH hDS =
				GDALBuildVRT(strVrtFile.c_str(), fileList.size(), nullptr, fileList.List(), psOptions, nullptr);
			GDALBuildVRTOptionsFree(psOptions);

			if (!hDS)
			{
				PCS_ERROR("[createDOMDataset] GDAL 打开文件失败.");
				return ds;
			}

			// 获得投影坐标系
			std::string src_wkt = GDALGetProjectionRef(hDS);

			// 获得坐标系统一代号
			int EPSG = 0;
			OGRSpatialReference osr(src_wkt.c_str());

			if (osr.AutoIdentifyEPSG() == OGRERR_NONE)
			{
				std::string targetKey = osr.IsProjected() ? "PROJCS" : "GEOGCS";
				std::string epsgCode = osr.GetAuthorityCode(targetKey.c_str());
				EPSG = atoi(epsgCode.c_str());
			}

			char* wktBuf;
			if (osr.importFromEPSG(EPSG) == OGRERR_NONE)
			{
				if (osr.exportToWkt(&wktBuf) == OGRERR_NONE)
				{
					src_wkt = wktBuf;
					OGRFree(wktBuf);
				}
			}

			// 创建自动转换投影的虚拟数据源
			GDALDatasetH hWarpedDS = GDALAutoCreateWarpedVRT(hDS,
															 src_wkt.c_str(),
															 src_wkt.c_str(),
															 GRA_NearestNeighbour,
															 5.0,
															 nullptr);

			if (!hWarpedDS)
			{
				PCS_ERROR("[createDOMDataset] WrapedVRT创建失败.");
				return ds;
			}

			GDALDataset* poDS = (GDALDataset*)hWarpedDS;

			ds = std::make_shared<DOMDataset>();
			ds->srs = poDS->GetProjectionRef();
			ds->width = poDS->GetRasterXSize();
			ds->height = poDS->GetRasterYSize();

			poDS->GetGeoTransform(ds->geotransform);
			const double* gt = ds->geotransform;

			ds->xmin = gt[0];
			ds->xmax = gt[0] + (ds->width - 1) * gt[1] + (ds->height - 1) * gt[2];
			ds->ymin = gt[3] + (ds->width - 1) * gt[4] + (ds->height - 1) * gt[5];
			ds->ymax = gt[3];
			ds->hDS = hWarpedDS;
			ds->hSourceDS = hDS;

			GDALFlushCache(hWarpedDS);
			GDALFlushCache(hDS);
			return ds;
		}

		std::vector<std::vector<TileKey>> getTiles(int EPSG,
												   int tileSize,
												   double resolution,
												   const BoundingBox2D& bound)
		{
			std::vector<std::vector<TileKey>> gridTiles;

			try
			{
				// 将EPSG转换为WKT
				OGRSpatialReference osr;
				if (osr.importFromEPSG(EPSG) != OGRERR_NONE)
					throw std::runtime_error("EPSG 转换出错.");

				osg::ref_ptr<SpatialReference> srs = SpatialReference::createFromHandle(&osr);

				osg::Vec3d point(180.0, 0.0, 0.0);
				srs->getGeographicSRS()->transform(point, srs.get(), point);
				double e = point.x();
				osg::ref_ptr<const Profile> profile = Profile::create(srs.get(), -e, -e, e, e);

				// 根据分辨率计算level
				unsigned int level =
					profile->getLevelOfDetailForHorizResolution(resolution, tileSize);

				TileKey ll = profile->createTileKey(bound.xMin(), bound.yMin(), level);
				TileKey ur = profile->createTileKey(bound.xMax(), bound.yMax(), level);

				/*PCS_INFO(_T("Column: (%d -> %d); Row: (%d -> %d)"),
						ll.getTileX(),
						ur.getTileX(),
						ur.getTileY(),
						ll.getTileY());*/

				int numCols = ur.getTileX() - ll.getTileX() + 1;
				int numRows = ll.getTileY() - ur.getTileY() + 1;
				gridTiles.resize(numRows, std::vector<TileKey>(numCols));

				for (unsigned int r = 0; r < numRows; r++)
				{
					for (unsigned int c = 0; c < numCols; c++)
					{
						TileKey key(level, ll.getTileX() + c, ur.getTileY() + r, profile);
						// tileKeys.push_back(key);
						gridTiles[r][c] = key;
					}
				}
			}
			catch (const std::exception& e)
			{
				PCS_ERROR(_T("[CImageToolkit] 获取瓦片出错, 错误信息: %s."), CA2T(e.what()));
			}

			return gridTiles;
		}

		void closeDOMDataset(std::shared_ptr<DOMDataset> ds)
		{
			if (ds.get())
			{
				// 关闭GDAL数据集句柄，释放内存
				if (ds->hDS)
				{
					GDALFlushCache(ds->hDS);
					GDALClose(ds->hDS);
				}

				if (ds->hSourceDS)
				{
					GDALFlushCache(ds->hSourceDS);
					GDALClose(ds->hSourceDS);
				}

				ds->hDS = ds->hSourceDS = nullptr;
			}
		}

		void readDOMImage_deprecated(std::shared_ptr<DOMDataset> ds, BoundingBox2D bound, cv::Mat& img)
		{
			if (!ds.get() || !ds->hDS)
				return;

			BoundingBox2D ds_bbox(ds->xmin, ds->ymin, ds->xmax, ds->ymax);

			if (!ds_bbox.intersects(bound))
				return;

			Timer timer;
			timer.Start();

			const double* gt = ds->geotransform;

			int col_min = std::floor((bound.xMin() - ds->xmin) / gt[1]);
			int col_max = std::floor((bound.xMax() - ds->xmin) / gt[1]);
			int row_min = std::floor((bound.yMax() - ds->ymax) / gt[5]);
			int row_max = std::floor((bound.yMin() - ds->ymax) / gt[5]);

			int xSize = col_max - col_min + 1;
			int ySize = row_max - row_min + 1;

			int sub_col_min = 0;
			int sub_col_max = col_max - col_min;
			int sub_row_min = 0;
			int sub_row_max = row_max - row_min;

			if (col_min < 0)
			{
				sub_col_min = -col_min;
				sub_col_max = (col_max - col_min) - sub_col_min;
				col_min = 0;
			}
			
			if (col_max >= (int)ds->width)
			{
				sub_col_max = (col_max - col_min) - (col_max - (int)ds->width + 1);
				col_max = (int)ds->width - 1;
			}

			if (row_min < 0)
			{
				sub_row_min = -row_min;
				sub_row_max = (row_max - row_min) - sub_row_min;
				row_min = 0;
			}

			if (row_max >= (int)ds->height)
			{
				sub_row_max = (row_max - row_min) - (row_max - (int)ds->height + 1);
				row_max = (int)ds->height - 1;
			}

			int xSizeSub = sub_col_max - sub_col_min + 1;
			int ySizeSub = sub_row_max - sub_row_min + 1;

			if (xSizeSub <= 0 || ySizeSub <= 0)
				return;
			
			GDALDataset* pDS = (GDALDataset*)(ds->hDS);
			int numBand = pDS->GetRasterCount();

			if (numBand < 3)
				return;

			std::vector<cv::Mat> cvBands(3);

			for (int i = 0; i < 3; ++i)
			{
				cvBands[i] = cv::Mat(ySizeSub, xSizeSub, CV_8U);
				// std::vector<uint8_t> buffer(xSizeSub * ySizeSub, 0u);
				GDALRasterBand* pBand = pDS->GetRasterBand(i + 1);

				auto err = pBand->RasterIO(GF_Read,
										   col_min,
										   row_min,
										   xSizeSub,
										   ySizeSub,
										   cvBands[i].data,
										   xSizeSub,
										   ySizeSub,
										   GDT_Byte,
										   0,
										   0);

				if (err != CE_None)
					PCS_ERROR("[readDOMImage] GDAL读取通道 %d 出现错误!", i + 1);

				// cvBands.at(i) = cv::Mat(ySizeSub, xSizeSub, CV_8U, buffer.data()).clone();
			}

			// 合并 r, g, b通道
			cv::Mat subset;
			cv::merge(cvBands, subset);
			cv::cvtColor(subset, subset, cv::COLOR_RGB2BGR);
			
			img = cv::Mat(ySize, xSize, CV_8UC3, cv::Scalar(0, 0, 0));
			subset.copyTo(img(cv::Rect(sub_col_min, sub_row_min, xSizeSub, ySizeSub)));

			PCS_DEBUG("[readDomImage] 耗时 %lf ms", timer.ElapsedSeconds() * 1000.0);
		}

		bool createImageFromCache(const std::string& cachedir,
								  const TileKey& tile,
								  cv::Mat& mat)
		{

			std::string filename = StringPrintf("%s%d\\%d\\%d.jpg",
												cachedir.c_str(),
												tile.getLOD(),
												tile.getTileY(),
												tile.getTileX());

			if (!ExistsPath(filename))
				return false;
			
			mat = cv::imread(filename);
			return true;
		}

		void writeImage(const std::string& cache_dir, const TileKey& tile, const cv::Mat& mat)
		{
			if (mat.empty())
				return;

			std::string tile_path = StringPrintf("%s%d\\%d\\%d.jpg",
												 cache_dir.c_str(),
												 tile.getLOD(),
												 tile.getTileY(),
												 tile.getTileX());

			std::string parent_dir = GetParentDir(tile_path);
			CreateDirsIfNotExists(parent_dir);

			cv::imwrite(tile_path, mat);
		}


		bool createImageFromRastersInBoundary(GDALDatasetH hWarpedDS,
											  const TileKey& tile,
											  unsigned int tileSize,
											  cv::Mat& mat,
											  GDALProgressFunc progressFunc,
											  void* progressParam)
		{
			if (!hWarpedDS)
				return false;

			std::string src_wkt = GDALGetProjectionRef(hWarpedDS);

			double geoTransform[6] = { 0.0 };
			double invtTransfrom[6] = { 0.0 };
			int rasterWidth = GDALGetRasterXSize(hWarpedDS);
			int rasterHeight = GDALGetRasterYSize(hWarpedDS);
			GDALGetGeoTransform(hWarpedDS, geoTransform);
			GDALInvGeoTransform(geoTransform, invtTransfrom);

			double minX, minY, maxX, maxY; // 数据源的地理坐标范围
			pixelToGeo(geoTransform, 0.0, rasterHeight, minX, minY);
			pixelToGeo(geoTransform, rasterWidth, 0.0, maxX, maxY);

			osg::ref_ptr<SpatialReference> srs = SpatialReference::create(src_wkt);
			GeoExtent extents(srs, minX, minY, maxX, maxY); // 栅格数据源的地理范围
			GeoExtent tileExtent = tile.getExtent();

			// 根据目标分辨率计算 TileSize
			if (!extents.intersects(tileExtent))
				return false;

			// 获得目标 tile 的边界
			double xmin, ymin, xmax, ymax;
			tileExtent.getBounds(xmin, ymin, xmax, ymax);

			// 计算输入 tile边界与数据源边界的相交区域
			auto intersection = tileExtent.intersectionSameSRS(extents);

			// 相对于数据源，相交区域的像素窗口坐标
			double src_min_x, src_min_y, src_max_x, src_max_y;

			geoToPixel(invtTransfrom,
					   rasterWidth,
					   rasterHeight,
					   intersection.xMin(),
					   intersection.yMax(),
					   src_min_x,
					   src_min_y);

			geoToPixel(invtTransfrom,
					   rasterWidth,
					   rasterHeight,
					   intersection.xMax(),
					   intersection.yMin(),
					   src_max_x,
					   src_max_y);

			// 尽可能更大的窗口
			src_min_x = floor(src_min_x);
			src_min_y = floor(src_min_y);
			src_max_x = ceil(src_max_x);
			src_max_y = ceil(src_max_y);

			// 相交于数据源像素坐标系原点的偏移和宽、高
			int off_x = (int)(src_min_x);
			int off_y = (int)(src_min_y);
			int width = (int)(src_max_x - src_min_x);
			int height = (int)(src_max_y - src_min_y);

			if (off_x + width > rasterWidth || off_y + height > rasterHeight)
				PCS_WARN(_T("[CreateImageFromRastersInBoundary] 输入窗口超出数据源边界"));

			// 确定目标窗口
			// 计算 Tile 与数据源相交区域的地理坐标偏移(相对与 Tile 边界)
			double offset_left = intersection.xMin() - xmin;
			double offset_top = ymax - intersection.yMax();

			int target_width =
				(int)ceil((intersection.width() / tileExtent.width()) * (double)tileSize);
			int target_height =
				(int)ceil((intersection.height() / tileExtent.height()) * (double)tileSize);
			int tile_offset_left =
				(int)floor((offset_left / tileExtent.width()) * (double)tileSize);
			int tile_offset_top = (int)floor((offset_top / tileExtent.height()) * (double)tileSize);


			if (width <= 0 || height <= 0 || target_width <= 0 || target_height <= 0)
				return false;

			if (GDALGetRasterCount(hWarpedDS) < 3)
			{
				PCS_ERROR(_T("[CreateImageFromRastersInBoundary] 暂不支持非RGB图像"));
				return false;
			}

			GDALRasterBandH hBandRed = GDALGetRasterBand(hWarpedDS, 1);
			GDALRasterBandH hBandGreen = GDALGetRasterBand(hWarpedDS, 2);
			GDALRasterBandH hBandBlue = GDALGetRasterBand(hWarpedDS, 3);

			// 初始化内存
			unsigned char* rgb[3] = { new unsigned char[target_width * target_height],
									  new unsigned char[target_width * target_height],
									  new unsigned char[target_width * target_height] };

			for (int i = 0; i < 3; ++i)
				memset(rgb[i], 0, target_width * target_height);

			mat = cv::Mat(tileSize, tileSize, CV_8UC3, cv::Scalar(0, 0, 0));

			GDALRasterIOExtraArg extraArg;
			INIT_RASTERIO_EXTRA_ARG(extraArg);
			extraArg.eResampleAlg = GRIORA_NearestNeighbour;

			extraArg.bFloatingPointWindowValidity = TRUE;
			extraArg.dfXOff = off_x;
			extraArg.dfYOff = off_y;
			extraArg.dfXSize = width;
			extraArg.dfYSize = height;
			extraArg.pfnProgress = progressFunc;
			extraArg.pProgressData = progressParam;

			double NODATA[3] = { 0.0 };
			for (int i = 0; i < 3; ++i)
			{
				GDALRasterBand* pBand = (GDALRasterBand*)(GDALGetRasterBand(hWarpedDS, i + 1));
				NODATA[i] = pBand->GetNoDataValue();
			}

			// 从数据源读取像素
			for (int i = 0; i < 3; ++i)
			{
				GDALRasterBand* pBand = (GDALRasterBand*)GDALGetRasterBand(hWarpedDS, i + 1);

				// PC_DEBUG(_T("[CImageToolkit] band %d nodata: %lf"), i + 1, nodata);

				pBand->RasterIO(GF_Read,
								off_x,
								off_y,
								width,
								height,
								rgb[i],
								target_width,
								target_height,
								GDT_Byte,
								0,
								0,
								&extraArg);
			}

			size_t numNoDataPixels = 0;

			for (int src_row = 0, dst_row = tile_offset_top; src_row < target_height;
				 src_row++, dst_row++)
			{
				for (int src_col = 0, dst_col = tile_offset_left; src_col < target_width;
					 ++src_col, ++dst_col)
				{
					unsigned char r = rgb[0][src_col + src_row * target_width];
					unsigned char g = rgb[1][src_col + src_row * target_width];
					unsigned char b = rgb[2][src_col + src_row * target_width];

					if (r == NODATA[0] && g == NODATA[1] && b == NODATA[2])
					{
						numNoDataPixels++;
						r = g = b = 0;
					}

					mat.at<cv::Vec3b>(dst_row, dst_col) = cv::Vec3b(r, g, b);
				}
			}

			cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);

			// 销毁内存
			for (int i = 0; i < 3; ++i)
				delete[] rgb[i];

			return (numNoDataPixels != target_width * target_height);
		}

		void readDOMImage(std::shared_ptr<DOMDataset> ds,
						  const BoundingBox2D& bound,
						  cv::Mat& img,
						  std::string cachedir /*= ""*/)
		{
			if (!ds.get() || !ds->hDS)
				return;

			BoundingBox2D ds_bbox(ds->xmin, ds->ymin, ds->xmax, ds->ymax);

			if (!ds_bbox.intersects(bound))
				return;

			Timer timer;
			timer.Start();

			int EPSG = 0;
			std::string wkt = GDALGetProjectionRef(ds->hDS);

			OGRSpatialReference osr(wkt.c_str());
			if (osr.AutoIdentifyEPSG() == OGRERR_NONE)
			{
				std::string targetKey = osr.IsProjected() ? "PROJCS" : "GEOGCS";
				std::string epsgCode = osr.GetAuthorityCode(targetKey.c_str());
				EPSG = atoi(epsgCode.c_str());
			}

			if (EPSG == 0)
				return;

			int stepProgress = 0;
			double resolution = 0.1;
			unsigned int tileSize = 256;
			auto gridTiles = getTiles(EPSG, tileSize, resolution, bound);

			BoundingBox2D newBound = bound;
			cv::Mat mergedMat;

			for (int r = 0; r < (int)gridTiles.size(); ++r)
			{
				cv::Mat rowMat;
				const auto& rowTiles = gridTiles.at(r);

				for (int c = 0; c < (int)rowTiles.size(); ++c)
				{
					const auto& tile = rowTiles.at(c);

					// 更新合并瓦片的边界
					newBound.xMin() = std::min(newBound.xMin(), (float)tile.getExtent().xMin());
					newBound.yMin() = std::min(newBound.yMin(), (float)tile.getExtent().yMin());
					newBound.xMax() = std::max(newBound.xMax(), (float)tile.getExtent().xMax());
					newBound.yMax() = std::max(newBound.yMax(), (float)tile.getExtent().yMax());

					cv::Mat mat;
					bool fromCache = !cachedir.empty() && createImageFromCache(cachedir, tile, mat);

					if (!fromCache)
					{
						bool bSuccess = createImageFromRastersInBoundary(ds->hDS,
																		 tile,
																		 tileSize,
																		 mat,
																		 nullptr,
																		 nullptr);

						if (!bSuccess)
							mat = cv::Mat(tileSize, tileSize, CV_8UC3, cv::Scalar(0, 0, 0));
						else
						{
							// 写入到缓存
							if (!cachedir.empty())
								writeImage(cachedir, tile, mat);
						}
					}

					if (c == 0)
						rowMat = mat.clone();
					else
						cv::hconcat(rowMat, mat, rowMat);
				}

				if (r == 0)
					mergedMat = rowMat.clone();
				else
					cv::vconcat(mergedMat, rowMat, mergedMat);
			}

			// 从 newBound 截取bound部分
			int width = mergedMat.cols;
			int height = mergedMat.rows;
			img = mergedMat;

			if (width > 0 && height > 0)
			{
				double xRes = (newBound.xMax() - newBound.xMin()) / (double)width;
				double yRes = (newBound.yMin() - newBound.yMax()) / (double)height;

				int c0 = std::floor((bound.xMin() - newBound.xMin()) / xRes);
				int r0 = std::floor((bound.yMax() - newBound.yMax()) / yRes);
				int c1 = std::floor((bound.xMax() - newBound.xMin()) / xRes);
				int r1 = std::floor((bound.yMin() - newBound.yMax()) / yRes);

				if (c0 >= 0 && r0 >= 0 && c1 > c0 && r1 > r0) 
				{
					cv::Mat dst(r1 - r0, c1 - c0, CV_8UC3, cv::Scalar(255, 255, 255));
					cv::Rect roi(c0, r0, c1 - c0, r1 - r0);
					mergedMat(roi).copyTo(dst);
					img = dst;
				}
			}
			/*cv::imwrite(DebugDirectory + "image_from_box.jpg", mergedMat);
			cv::imwrite(DebugDirectory + "image_clip.jpg", img);*/

			// PCS_DEBUG("[readDomImage] 耗时 %lf ms", timer.ElapsedSeconds() * 1000.0);
		}
	}
}