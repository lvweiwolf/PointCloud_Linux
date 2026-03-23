#include <include/SpatialRef.h>
#include <include/StringToolkit.h>

#include <Tool/FileToolkit.h>
#include <Tool/LibToolkit.h>

#include <gdal_priv.h>
#include <gdalwarper.h>
#include <ogr_spatialref.h>
#include <ogr_srs_api.h>

namespace pc {
	namespace tool {

		std::map<CString, CSpatialRef*> CSpatialRef::s_Wkt2SpatialRefDic;
		std::map<int, CSpatialRef*> CSpatialRef::s_EPSG2SpatialRefDic;
		std::map<void*, CSpatialRef*> CSpatialRef::s_RefH2SpatialRefDic;

		LPCTSTR CSpatialRef::WebMercatorWkt =
			L"PROJCS[\"WGS_1984_Web_Mercator_Auxiliary_Sphere\",GEOGCS[\"GCS_WGS_1984\",\
DATUM[\"D_WGS_1984\",SPHEROID[\"WGS_1984\",6378137.0,298.257223563]],PRIMEM[\"Greenwich\",0.0],\
UNIT[\"Degree\",0.0174532925199433]],PROJECTION[\"Mercator_Auxiliary_Sphere\"],PARAMETER[\"False_Easting\",0.0],\
PARAMETER[\"False_Northing\",0.0],PARAMETER[\"Central_Meridian\",0.0],PARAMETER[\"Standard_Parallel_1\",0.0],\
PARAMETER[\"Auxiliary_Sphere_Type\",0.0],UNIT[\"Meter\",1.0]]";
		LPCTSTR CSpatialRef::SphereWgs84 =
			L"GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,\
AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],\
UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]";
		LPCTSTR CSpatialRef::CGCS2000 =
			L"GEOGCS[\"China Geodetic Coordinate System 2000\",DATUM[\"D_China_2000\",\
SPHEROID[\"CGCS2000\",6378137,298.257222101]],PRIMEM[\"Greenwich\",0],UNIT[\"Degree\",0.017453292519943295]]";
		int CSpatialRef::Wgs84EPSG = 4326;

		CSpatialRef* CSpatialRef::FindSpatialRef(LPCTSTR szWtk)
		{
			auto iter = s_Wkt2SpatialRefDic.find(szWtk);
			if (iter != s_Wkt2SpatialRefDic.end())
				return iter->second;

			CSpatialRef* pNewObj = new CSpatialRef(szWtk);
			s_Wkt2SpatialRefDic[szWtk] = pNewObj;
			s_RefH2SpatialRefDic[pNewObj->_sptRef] = pNewObj;
			return pNewObj;
		}

		void CSpatialRef::ClearAll()
		{
			for (auto iter : s_RefH2SpatialRefDic)
			{
				delete iter.second;
			}
			s_RefH2SpatialRefDic.clear();
			s_Wkt2SpatialRefDic.clear();
			for (auto iter : s_EPSG2SpatialRefDic)
			{
				delete iter.second;
			}
			s_EPSG2SpatialRefDic.clear();
		}

		CSpatialRef::CSpatialRef(LPCTSTR szWtk) : _strWktDescr(szWtk)
		{
			//_strWktDescr = szWtk;
			_sptRef = OSRNewSpatialReference(nullptr);

			std::string wktAStr = CStringToolkit::CStringToUTF8(szWtk);
			char* pHead = wktAStr.data();
			OSRImportFromWkt((OGRSpatialReferenceH)_sptRef, &pHead);
			_nEPSGCode = -1;

			if (OGRERR_NONE == OSRAutoIdentifyEPSG((OGRSpatialReferenceH)_sptRef))
			{
				std::string attriName =
					OSRIsProjected((OGRSpatialReferenceH)_sptRef) ? "PROJCS" : "GEOGCS";
				std::string strEPSG =
					OSRGetAuthorityCode((OGRSpatialReferenceH)_sptRef, attriName.c_str());

				_nEPSGCode = atoi(strEPSG.c_str());
			}
		}

		CSpatialRef::~CSpatialRef() { OSRRelease((OGRSpatialReferenceH)_sptRef); }

		CSpatialRef* CSpatialRef::FindSpatialRef(int nEPSG)
		{
			auto iter = s_EPSG2SpatialRefDic.find(nEPSG);
			if (iter != s_EPSG2SpatialRefDic.end())
				return iter->second;

			CSpatialRef* pNewObj = new CSpatialRef(nEPSG);
			s_EPSG2SpatialRefDic[nEPSG] = pNewObj;
			return pNewObj;
		}

		osg::Vec3d CSpatialRef::ToThisSpatialRef(const osg::Vec3d& srcPnt,
												 const CSpatialRef* pSrcSpatRef)
		{
			OGRCoordinateTransformationH xform_handle = nullptr;
			xform_handle =
				OCTNewCoordinateTransformation((OGRSpatialReferenceH)pSrcSpatRef->_sptRef,
											   (OGRSpatialReferenceH)_sptRef);

			double x = srcPnt.x(), y = srcPnt.y(), z = srcPnt.z();
			osg::Vec3d retPnt = srcPnt;
			// 转换坐标
			if (xform_handle != nullptr)
			{
				OCTTransform(xform_handle, 1, &retPnt.x(), &retPnt.y(), &retPnt.z());
				OCTDestroyCoordinateTransformation(xform_handle);
			}
			return retPnt;
		}

		void CSpatialRef::ConvertCoordinate(const std::vector<osg::Vec3d>& srcInPntList,
											std::vector<osg::Vec3d>& destOutPntList,
											const CSpatialRef* pSrcSpatRef)
		{
			if (srcInPntList.empty())
				return;

			OGRCoordinateTransformationH xform_handle = nullptr;
			xform_handle =
				OCTNewCoordinateTransformation((OGRSpatialReferenceH)pSrcSpatRef->_sptRef,
											   (OGRSpatialReferenceH)_sptRef);

			// 转换坐标
			if (xform_handle != nullptr)
			{
				// 创建3个数组存储x,y,z
				int nCount = srcInPntList.size();
				double* dx = new double[nCount];
				double* dy = new double[nCount];
				double* dz = new double[nCount];

				for (int i = 0; i < nCount; ++i)
				{
					dx[i] = srcInPntList[i].x();
					dy[i] = srcInPntList[i].y();
					dz[i] = srcInPntList[i].z();
				}

				OCTTransform(xform_handle, nCount, dx, dy, dz);

				for (int i = 0; i < nCount; ++i)
				{
					osg::Vec3d pt(dx[i], dy[i], dz[i]);
					destOutPntList.emplace_back(pt);
				}

				delete[] dx;
				delete[] dy;
				delete[] dz;
				OCTDestroyCoordinateTransformation(xform_handle);
			}
		}

		CSpatialRef::CSpatialRef(int nEPSG)
		{
			_sptRef = OSRNewSpatialReference(nullptr);
			OSRImportFromEPSG((OGRSpatialReferenceH)_sptRef, nEPSG);

			char* wktBuf = NULL;
			OSRExportToWkt((OGRSpatialReferenceH)_sptRef, &wktBuf);
			_strWktDescr = CStringToolkit::UTF8ToCString(wktBuf);
			OGRFree(wktBuf);
			_nEPSGCode = nEPSG;
		}

		class CDllInitor
		{
		public:
			CDllInitor()
			{
				// 加载GDAL驱动
				GDALAllRegister();
				CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
				CPLSetConfigOption("SHAPE_ENCODING", "");

				CString strModulePath = CLibToolkit::GetAppModuleFilename();
				CString strDir = CFileToolkit::GetFileDirectory(strModulePath);
				CString strGdalDataPath = strDir + L"system\\gdal_data\\";
				std::string gdalDataPath = CStringToolkit::CStringToUTF8(strGdalDataPath);
				
				CPLSetConfigOption("GDAL_DATA", gdalDataPath.c_str());
			};

			~CDllInitor() { CSpatialRef::ClearAll(); };
		};

		CDllInitor g_Initor;
	}
}