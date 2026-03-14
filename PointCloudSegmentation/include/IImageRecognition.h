/*-----------------------------------------------------
* 文件名称：  IImageRecognition.H
* 功能描述：  图像识别
* 创建标识：  liangchao  2023/4/25 9:54
*
* 修改标识：
* 修改描述：
*
* 修改标识：
* 修改描述：
-----------------------------------------------------*/

#ifndef IIMAGERECOGNITION_H_
#define IIMAGERECOGNITION_H_

#include <include/Export.h>
#include <include/share_ptr.h>
#include <include/ReferenceCountObj.h>

namespace d3s {
	namespace pcs {
		class IOptions;
	}
}

namespace d3s {
	namespace pcs {
		class SEG_EXPORT IImageRecognition : public d3s::ReferenceCountObj
		{
		public:
			virtual ~IImageRecognition() {}

		public:
			/**
			 *  函数介绍： 识别
			 *
			 *  输入参数： IOptions * pOption		识别参数
			 *  返回值：   void
			 */
			virtual bool Recognition(IOptions* pOption) = 0;
		};
	}
}

typedef d3s::share_ptr<d3s::pcs::IImageRecognition> IImageRecognitionPtr;

#endif // IIMAGERECOGNITION_H_