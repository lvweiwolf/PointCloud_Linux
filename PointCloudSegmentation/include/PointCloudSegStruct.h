#ifndef POINTCLOUDSEGSTRUCT_H_
#define POINTCLOUDSEGSTRUCT_H_

#include <include/Export.h>

#define MASK(x) (1 << (x))
// 自动分类方法掩码

namespace d3s {
	namespace pcs {
		enum RecognitionType
		{
			eCrop, // 林木作物
		};

		enum SegmentationType
		{
			GROUND,				 // 地面提取
			POWERCORRIDORS,		 // 电力线元素(铁塔、电力线)分类
			ENVIROMENTS,		 // 环境元素(植被、建筑)分类
			TREEINDIVIDUAL,		 // 单木分割
			TREESPECIESIDENTIFY, // 树种识别
			CROPS,				 // 农作物分类
			DANGER				 // 危险区域分割
		}; // 识别类型
	}
}

#endif // POINTCLOUDSEGSTRUCT_H_