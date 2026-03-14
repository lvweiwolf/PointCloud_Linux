#ifndef POINTCLOUDSEGAPI_H_
#define POINTCLOUDSEGAPI_H_

#include <include/ICloudSegmentation.h>
#include <include/IGeoProjectionConvertor.h>
#include <include/IImageRecognition.h>
#include <include/PointCloudSegStruct.h>

namespace d3s {
	namespace pcs {
		/**
		 *  @brief    根据分类类别创建分类器
		 *
		 *  @param    SegmentationType type			点云分类类别
		 *
		 *  @return   d3s::pcs::ICloudSegmentation*		点云分类器接口对象
		 */
		d3s::pcs::ICloudSegmentation* SEG_EXPORT
		CreateSegmentation(d3s::pcs::SegmentationType eSegType);
		/**
		 *  @brief    根据配置文件内容创建分类参数
		 *
		 *  @param    const char * iniFile			从ini文件读取参数配置
		 *
		 *  @return   d3s::pcs::IOptions*
		 */
		d3s::pcs::IOptions* SEG_EXPORT CreateSegOption(const char* iniFile);


		/**
		 *  @brief    创建地理坐标转换接口
		 *
		 *  @param    int epsg						坐标系EPSG编码
		 *
		 *  @return   d3s::pcs::IGeoProjectionConvertor*
		 */
		d3s::pcs::IGeoProjectionConvertor* SEG_EXPORT CreateGeoProjectionConvertor(int epsg);

		/**
		 *  @brief    创建杆塔位置检测接口
		 *
		 *  @return   d3s::pcs::ICloudDetection*
		 */
		d3s::pcs::ICloudDetection* SEG_EXPORT CreateaPylonDetection();

		/**
		 *  @brief    创建杆塔点云重建接口
		 *
		 *
		 *  @return  d3s::pcs::ICloudReconstrct*
		 */
		d3s::pcs::ICloudReconstrct* SEG_EXPORT CreatePylonReconstruct();

		/**
		 *  @brief    创建电力线曲线拟合接口
		 *
		 *  @return   d3s::pcs::ICloudCurveFitting*
		 */
		d3s::pcs::ICloudCurveFitting* SEG_EXPORT CreatePowerlineFitting();

		/**
		 *  @brief    创建点云聚类接口
		 *
		 *  @return   d3s::pcs::ICloudClustering*
		 */
		d3s::pcs::ICloudClustering* SEG_EXPORT CreateCloudClustering();


		/**
		 *  @brief    创建点云栅格化接口
		 *
		 *  @return   d3s::pcs::ICloudRasterize*
		 */
		d3s::pcs::ICloudRasterize* SEG_EXPORT CreateCloudRasterize();

		/**
		 *  @brief    创建点云数据
		 *
		 *  @return   d3s::pcs::IPointCloud*
		 */
		d3s::pcs::IPointCloud* SEG_EXPORT CreatePointCloud();

		/**
		 *  @brief    创建道路矢量数据，设置到IOptions中，为道路分类提供约束
		 *			  对应字段名称为 "RoadVectorize" 例子：
		 *
		 *				IValueBuffer* buffer = service->ReadRoadVectorize("全国道路.zip");
		 *				{
		 *					IOptions* options = CreateOptions(..)
		 *					options->Set("RoadVectorize", (char*)buffer);
		 *				}
		 *
		 *
		 *  @param    const char * filename
		 *
		 *  @return   IValueBuffer*
		 */
		d3s::pcs::IValueBuffer* SEG_EXPORT CreateRoadVectorize(const std::string& filename);



		/// ImageRecognition
		/**
		 *  函数介绍： 根据配置文件内容创建识别参数
		 *
		 *  输入参数： const char * iniFile				配置文件
		 *  返回值：   d3s::pcs::IOptions*					返回参数配置
		 */
		IOptions* CreateOptions(const char* iniFile);

		/**
		 *  函数介绍： 根据类别创建识别器
		 *
		 *  输入参数： RecognitionType recognitionType		识别类别
		 *  返回值：   d3s::pcs::IImageRecognition*		识别器
		 */
		IImageRecognition* CreateRecogition(RecognitionType recognitionType);
	}
}

#endif // POINTCLOUDSEGAPI_H_