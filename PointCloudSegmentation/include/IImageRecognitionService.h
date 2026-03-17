/*-----------------------------------------------------   
* 文件名称：  IImageRecognitionService.H
* 功能描述：  影像识别服务
* 创建标识：  liangchao  2023/4/25 9:39
*
* 修改标识：
* 修改描述：           
*
* 修改标识：
* 修改描述：
-----------------------------------------------------*/
#ifndef IIMAGERECOGNITIONSERVICE_H_
#define IIMAGERECOGNITIONSERVICE_H_

#include <include/Export.h>
#include <include/share_ptr.h>
#include <include/ReferenceCountObj.h>


#define SERVICE_IMAGE_RECOGNITION _T("service_iamge_recognition")

namespace d3s {
	namespace pcs {
		class IOptions;
		class IImageRecognition;

		// 识别类型
		enum class RecognitionType
		{
			eCrop,						// 林木作物
		};
	}
}

namespace d3s {
	namespace pcs {

		class IImageRecognitionService 
		{
		public:
			virtual ~IImageRecognitionService() {}

		public:
			
			/**
			*  函数介绍： 根据配置文件内容创建识别参数
			*
			*  输入参数： const char * iniFile				配置文件
			*  返回值：   d3s::pcs::IOptions*					返回参数配置
			*/
			virtual IOptions* CreateOptions(const char* iniFile) = 0;
			
			/**
			*  函数介绍： 根据类别创建识别器
			*
			*  输入参数： RecognitionType recognitionType		识别类别
			*  返回值：   d3s::pcs::IImageRecognition*		识别器
			*/
			virtual IImageRecognition* CreateRecogition(RecognitionType recognitionType) = 0;
		};

	}
}

typedef d3s::share_ptr<d3s::pcs::IImageRecognitionService> IImageRecognitionServicePtr;

#endif // IIMAGERECOGNITIONSERVICE_H_