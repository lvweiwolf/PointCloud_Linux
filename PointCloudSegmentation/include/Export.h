#ifndef EXPORT_H_
#define EXPORT_H_

#ifdef __cplusplus
#ifdef __linux__
#ifdef POINTCLOUDSEGMENTAPI
#define SEG_EXPORT __attribute__((visibility("default")))
// 构造函数 - 共享库加载时调用
__attribute__((constructor)) static void library_load()
{
	// 相当于DLL_PROCESS_ATTACH
	// 初始化代码
}
// 析构函数 - 共享库卸载时调用
__attribute__((destructor)) static void library_unload()
{
	// 相当于DLL_PROCESS_DETACH
	// 清理代码
}
#else
#define SEG_EXPORT
#endif

// #define MASK(x) (1 << (x))
//// 自动分类方法掩码
// POINTCLOUDSEGMENTAPI enum SegClassification
//{
//	eSegmentGround = MASK(0),		  // 地面分类
//	eSegmentPowerCorridors = MASK(1), // 电力线路分类
//	eSegmentEnviroments = MASK(2),	  // 环境分类
//	eSegmentTrees = MASK(3),		  // 单木分割
//	eSegmentTreeSpecies = MASK(4),	  // 树种识别
//	eSegmentCrops = MASK(5)			  // 农作物识别
// };
//
// class POINTCLOUDSEGMENTAPI PointCloudSegmentAPI
//{
// public:
//	PointCloudSegmentAPI(SegClassification eSegClassType);
//	~PointCloudSegmentAPI();
//	PointCloudSegmentAPI();
// public:
//	bool Segment();
//	class ICloudSegmentation;
//	ICloudSegmentation* _segmentation;
// };

#endif
#endif


#endif // EXPORT_H_