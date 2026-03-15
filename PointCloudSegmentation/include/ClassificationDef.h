//////////////////////////////////////////////////////////////////////
// 文件名称：ClassificationDef.h
// 功能描述：输电线路分类类别定义
// 创建标识：吕伟	2022/10/13
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef CLASSIFICATION_DEF_H_
#define CLASSIFICATION_DEF_H_

namespace d3s {
	namespace pcs {

		enum Classification
		{
			eUnclassified = 0, // 未分类
			eGround,		   // 地面
			ePylon,			   // 杆塔
			ePowerline,		   // 电力线
			eLeadwire,		   // 引流线
			eIsulator,		   // 绝缘子
			eLowVegetation,	   // 低植被
			eHighVegetation,   // 高植被
			eBuilding,		   // 建筑
			eRoad,			   // 道路
			eWater,			   // 水域
			eOtherline,		   // 其他线路

			eCrop_Start = 50,
			eCrop_Other = 99, // 农作物-其他

			eTree_Start = 100,
			eTree_Other = 149, // 林地-其他

			eNumOfClassification
		};
	}
}

#endif // CLASSIFICATION_DEF_H_