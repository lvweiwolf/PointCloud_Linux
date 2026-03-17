/////////////////////////////////////////////////////////////////////
// 文件名称：GroundSegmentation.h
// 功能描述：地面分割实现
// 创建标识：吕伟	2022/6/26
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

#include <src/service/CloudSegmentation.h>

#include <osg/Vec3>

namespace d3s {
	namespace pcs {

		// 地面点分类
		class CGroundSegmentation : public CCloudSegmentation
		{
		public:
			virtual ~CGroundSegmentation();

			/**
			 *  @brief    进行点云分割
			 *
			 *  @param    const IOptions * options					分割算法使用参数
			 *  @param    std::vector<int> & result					分割结果点云索引
			 *
			 *  @return   bool
			 */
			virtual bool Segment(const IOptions* options, std::vector<PointId>& result);
		};

		// 输电线路分类
		class CPowerCorridorSegmentaion : public CCloudSegmentation
		{
		public:
			virtual ~CPowerCorridorSegmentaion();

			/**
			 *  @brief    进行点云分割
			 *
			 *  @param    const IOptions * options					分割算法使用参数
			 *  @param    std::vector<int> & result					分割结果点云索引
			 *
			 *  @return   bool
			 */
			virtual bool Segment(const IOptions* options, std::vector<PointId>& result);
		};

		// 环境元素分类(植被、建筑、道路)
		class CEnviromentSegmentation : public CCloudSegmentation
		{
		public:
			virtual ~CEnviromentSegmentation();

			/**
			 *  @brief    进行点云分割
			 *
			 *  @param    const IOptions * options					分割算法使用参数
			 *  @param    std::vector<int> & result					分割结果点云索引
			 *
			 *  @return   bool
			 */
			virtual bool Segment(const IOptions* options, std::vector<PointId>& result);
		};

		// 农作物分类
		class CCropSegmentation : public CCloudSegmentation
		{
		public:
			virtual ~CCropSegmentation();

			/**
			 *  @brief    进行点云分割
			 *
			 *  @param    const IOptions * options					分割算法使用参数
			 *  @param    std::vector<int> & result					分割结果点云索引
			 *
			 *  @return   bool
			 */
			virtual bool Segment(const IOptions* options, std::vector<PointId>& result);
		};

		// 单木分割
		class CTreeIndividual : public CCloudSegmentation
		{
		public:
			CTreeIndividual(std::string method = "Dalponte2016");

			virtual ~CTreeIndividual();


			/**
			 *  @brief    进行点云分割
			 *
			 *  @param    const IOptions * options					分割算法使用参数
			 *  @param    std::vector<int> & result					分割结果点云索引
			 *
			 *  @return   bool
			 */
			virtual bool Segment(const IOptions* options, std::vector<PointId>& result);

		private:
			std::string _method;
		};

		// 树种识别
		class CTreeSpeciesIdentify : public CCloudSegmentation
		{
		public:
			CTreeSpeciesIdentify();

			virtual ~CTreeSpeciesIdentify();

			/**
			 *  @brief    进行点云分割
			 *
			 *  @param    const IOptions * options					分割算法使用参数
			 *  @param    std::vector<int> & result					分割结果点云索引
			 *
			 *  @return   bool
			 */
			virtual bool Segment(const IOptions* options, std::vector<PointId>& result);
		};

		// 危险区域识别
		class CDangerSegmentation : public CCloudSegmentation
		{
		public:
			CDangerSegmentation();

			virtual ~CDangerSegmentation();

			/**
			 *  @brief    进行点云分割
			 *
			 *  @param    const IOptions * options					分割算法使用参数
			 *  @param    std::vector<int> & result					分割结果点云索引
			 *
			 *  @return   bool
			 */
			virtual bool Segment(const IOptions* options, std::vector<PointId>& result);
		};


		// 杆塔自动检测
		class CPylonDetection : public CCloudDetection
		{
		public:
			virtual ~CPylonDetection();

			/**
			 *  @brief    进行点云分割
			 *
			 *  @param    const IOptions * options					分割算法使用参数
			 *  @param    std::vector<int> & result					分割结果点云索引
			 *
			 *  @return   bool
			 */
			virtual bool DetectPositions(const IOptions* options,
										 std::vector<osg::Vec3d>& positions);
		};

		// 点云聚类
		class CCloudClusteringImpl : public CCloudClustering
		{
		public:
			virtual ~CCloudClusteringImpl();

			/**
			 *  @brief    曲线拟合，点云聚类
			 *
			 *  @param    double tolerance		聚类距离阈值
			 *  @param    int minPts			类簇最小点数
			 *  @param    int maxPts			类簇最大点数
			 *  @param    Clusters & clusters	聚类结果
			 *
			 *  @return   void
			 */
			virtual void EuclideanCluster(double tolerance,
										  int minPts,
										  int maxPts,
										  Clusters& clusters);

			/**
			 *  @brief    曲线拟合，点云聚类 (高性能版本，但在点云类簇非常靠近的情况下会不太准确)
			 *
			 *  @param    double tolerance		聚类距离阈值
			 *  @param    int minPts			类簇最小点数
			 *  @param    int maxPts			类簇最大点数
			 *  @param    Clusters & clusters	聚类结果
			 *
			 *  @return   void
			 */
			virtual void EuclideanClusterFast(double tolerance,
											  int minPts,
											  int maxPts,
											  Clusters& clusters);
		};

		class CTiffRasterize : public CCloudRasterize
		{
		public:
			virtual ~CTiffRasterize();

			virtual void WriteTiff(const std::string& filepath,
								   const std::string& srs,
								   double resolution);
		};

		// 电力线点云抛物线拟合
		class CPowerlineCurveFitting : public CCloudCurveFitting
		{
		public:
			virtual ~CPowerlineCurveFitting();

			/**
			 *  @brief
			 *
			 *  @param    double expandLength		曲线延长线长度 [0.0, DBL_MAX)
			 *  @param    PARACURVE & params		输出曲线拟合参数
			 *
			 *  @return   bool
			 */
			virtual bool CurveFitting(double expandLength, PARACURVE& params);
		};


		// 铁塔点云重建
		class CPylonReconstruct : public CCloudReconstruct
		{
		public:
			virtual ~CPylonReconstruct();

			/**
			 *  @brief    点云重建
			 *
			 *  @param    const IOptions * options					点云重建算法参数
			 *
			 *  @return   bool
			 */
			virtual bool Reconstruct(const IOptions* options);
		};
	}
}

#endif // SEGMENTATION_H_
