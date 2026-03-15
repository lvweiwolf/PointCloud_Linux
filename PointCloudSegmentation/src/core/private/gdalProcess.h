//////////////////////////////////////////////////////////////////////
// 文件名称：gdal_wrap.h
// 功能描述：GDAL 包装工具
// 创建标识：吕伟	2022/10/20
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef GDALPROCESS_H_
#define GDALPROCESS_H_

#include <src/algorithm/boundingbox2d.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <memory>

#ifndef WIN32
// Linux下，HANDLE通常用int或void*代替
typedef void* HANDLE;
// 或者使用更具体的类型
// typedef int HANDLE;  // 对于文件描述符
#endif

namespace osgEarth {
	class TileKey;
}

typedef osgEarth::TileKey TileKey;

namespace d3s {
	namespace pcs {

		struct VectorLineString
		{
			BoundingBox2D bound;
			std::vector<osg::Vec2> verts;

			// 数据读写
			void Write(std::ostream& stream);

			void Read(std::istream& stream);

			// 获取字节数
			size_t GetBytes();
		};

		struct DOMDataset
		{
			std::string srs;	 // 地理坐标投影系统描述字符串
			unsigned int width;	 // 像素宽度
			unsigned int height; // 像素高度
			double xmin;
			double ymin;
			double xmax;
			double ymax;
			double geotransform[6]; // GDAL 地理坐标变换矩阵

			HANDLE hDS = nullptr; // GDAL数据集句柄
			HANDLE hSourceDS = nullptr;
		};

		/**
		 *  @brief    投影坐标系从EPSG转换到WKT字符串
		 *
		 *  @param    unsigned int epsg
		 *
		 *  @return   std::string
		 */
		std::string getWktFromEPSGCode(unsigned int epsg);


		/**
		 *  @brief    坐标系转换
		 *
		 *  @prarm	 int epsg_src		原始坐标系EPSG
		 *  @prarm	 int epsg_dst		目标坐标系EPSG
		 *  @prarm	 double * x			坐标x
		 *  @prarm	 double * y			坐标y
		 *  @prarm	 double * z			坐标z
		 *
		 *  @return   void
		 */
		void coordinateSystemTransform(int epsg_src,
									   int epsg_dst,
									   double* x,
									   double* y,
									   double* z = nullptr);

		/**
		 *  @brief    读取道路矢量数据
		 *
		 *  @param    const std::string & shapefile					shp文件
		 *  @param    std::vector<VectorLineString * > & roadDS		道路矢量线串
		 *  @param    int & epsg									地理坐标EPSG
		 *
		 *  @return   void
		 */
		void readRoadDataset(const std::string& shapefile,
							 std::vector<VectorLineString*>& roadDS,
							 int& epsg);

		void readRoadDataset2(const std::string& shapefile,
							  std::vector<VectorLineString*>& roadDS,
							  int& epsg);

		/**
		 *  @brief    写入道路矢量数据
		 *
		 *  @param    const std::vector<VectorLineString * > & roadDS
		 *  @param    int epsg
		 *  @param    const std::string & filename
		 *
		 *  @return   void
		 */
		void writeRoadDataset(const std::vector<VectorLineString*>& roadDS,
							  int epsg,
							  const std::string& filename);

		/**
		 *  @brief	  创建DOM正射影像数据集
		 *
		 *  @param    const std::string & filename					DOM文件路径
		 *
		 *  @return   std::shared_ptr<d3s::pcs::DOMDataset>
		 */
		std::shared_ptr<DOMDataset> createDOMDataset(const std::string& filename,
													 bool shared = false);

		std::shared_ptr<DOMDataset> createDOMDataset(const std::vector<std::string>& files);

		/**
		 *  @brief    获得指定范围内的网格布局瓦片
		 *
		 *  @param    int EPSG										地理坐标系EPSG
		 *  @param    int tileSize									瓦片大小
		 *  @param    double resolution								指定分辨率
		 *  @param    const BoundingBox2D & bound					查询范围
		 *
		 *  @return   std::vector<std::vector<TileKey>>
		 */
		std::vector<std::vector<TileKey>> getTiles(int EPSG,
												   int tileSize,
												   double resolution,
												   const BoundingBox2D& bound);

		/**
		 *  @brief   关闭DOM正射影像数据集
		 *
		 *  @param    std::shared_ptr<DOMDataset> ds
		 *
		 *  @return   void
		 */
		void closeDOMDataset(std::shared_ptr<DOMDataset> ds);

		/**
		 *  @brief    读取指定范围内的DOM图像数据
		 *
		 *  @param    std::shared_ptr<DOMDataset> ds				DOM正射影像数据集
		 *  @param    BoundingBox2D bbox							地理坐标系下的数据范围
		 *  @param    cv::Mat& data									范围内图像数据
		 *
		 *  @return   void
		 */
		void readDOMImage_deprecated(std::shared_ptr<DOMDataset> ds,
									 BoundingBox2D bound,
									 cv::Mat& img);


		void readDOMImage(std::shared_ptr<DOMDataset> ds,
						  const BoundingBox2D& bound,
						  cv::Mat& img,
						  std::string cachedir = "");
	}
}

#endif // GDALPROCESS_H_