//////////////////////////////////////////////////////////////////////
// 文件名称：ICloudSegmentation.h
// 功能描述：点云分割抽象接口
// 创建标识：吕伟	2022/6/21
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef ICLOUD_SEGMENTATION_H_
#define ICLOUD_SEGMENTATION_H_

#include <include/Export.h>
#include <include/ReferenceCountObj.h>
#include <include/share_ptr.h>

#include <osg/Vec3>
#include <osg/Matrix>
#include <osg/Array>
#include <osg/ref_ptr>

#include <vector>
#include <cstdint>
#include <string>

namespace d3s {
	namespace pcs {

		typedef int PointId;

		// 值内存缓冲区
		//////////////////////////////////////////////////////////////////////////
		class SEG_EXPORT IValueBuffer : public d3s::ReferenceCountObj
		{
		public:
			virtual ~IValueBuffer() {}

			/**
			 *  @brief    获得内存大小字节数
			 *
			 *  @return   size_t
			 */
			virtual size_t GetBytes() const = 0;

			/**
			 *  @brief    获得内存地址
			 *
			 *  @return   void*
			 */
			virtual void* GetPtr() const = 0;
		};


		// 点云分割参数接口
		//////////////////////////////////////////////////////////////////////////
		class SEG_EXPORT IOptions : public d3s::ReferenceCountObj
		{
		public:
			virtual ~IOptions() {}

			/**
			 *  @brief    设置参数值
			 *
			 *  @param    const char *						参数名称
			 *  @param    float/double/int/string		各种类型的参数值
			 *
			 *  @return   void
			 */
			virtual void Set(const char*, float) = 0;
			virtual void Set(const char*, double) = 0;
			virtual void Set(const char*, int) = 0;
			virtual void Set(const char*, bool) = 0;
			virtual void Set(const char*, const char*) = 0;
			virtual void Set(const char*, char*) = 0;

			/**
			 *  @brief    获得参数值
			 *
			 *  @param    const char *						参数名称
			 *
			 *  @return   float/double/int/string
			 */
			virtual float GetFloat(const char*) const = 0;
			virtual double GetDouble(const char*) const = 0;
			virtual int GetInt(const char*) const = 0;
			virtual bool GetBool(const char*) const = 0;
			virtual std::string GetStr(const char*) const = 0;
			virtual void* GetData(const char*) const = 0;
		};



		typedef struct SEG_EXPORT tagPOINT
		{
			tagPOINT() : x(0.0), y(0.0), z(0.0), treeId(0), label(0) {}
			double x;			 // x坐标
			double y;			 // y坐标
			double z;			 // z坐标
			unsigned int treeId; // 所属树 id
			char label;			 // 点类型
		} POINT;


		// 抛物线参数
		typedef struct SEG_EXPORT tagPARACURVE
		{
			tagPARACURVE() : xmean(0.0), ymin(-DBL_MAX), ymax(DBL_MAX), a(0.0), b(0.0), c(0.0) {}

			osg::Matrix rotate; // 导线方向对齐到 Y 轴的旋转矩阵
			osg::Vec3d origin;	// 曲线方程的坐标原点

			double xmean; // 曲线所在平面的 X 轴偏移
			double ymin;  // 曲线所在平面 Y 轴最小值
			double ymax;  // 曲线所在平面 Y 轴最大值
			double a, b, c;

		} PARACURVE;

		// 点云数据抽象接口
		//////////////////////////////////////////////////////////////////////////
		class SEG_EXPORT IPointCloud : public d3s::ReferenceCountObj
		{
		public:
			virtual ~IPointCloud() {}


			/**
			 *  @brief    点云数据写入
			 *
			 *  @param    const std::string & filename		点云文件路径
			 *
			 *  @return   void
			 */
			virtual void Write(const std::string& filename) = 0;

			/**
			 *  @brief    设置树ID
			 *
			 *  @param    uint32_t treeId
			 *
			 *  @return   void
			 */
			virtual void SetTreeId(PointId idx, uint32_t treeId) = 0;

			/**
			 *  @brief    获得树ID
			 *
			 *  @return   uint32_t
			 */
			virtual uint32_t GetTreeId(PointId idx) const = 0;


			/**
			 *  @brief    设置点云类别
			 *
			 *  @param    PointId idx						点云索引
			 *  @param    uint32_t label					类别
			 *
			 *  @return   void
			 */
			virtual void SetClassification(PointId idx, uint32_t label) = 0;

			/**
			 *  @brief    获取点云类别
			 *
			 *  @param    PointId idx						点云索引
			 *
			 *  @return   uint32_t
			 */
			virtual uint32_t GetClassification(PointId idx) = 0;

			/**
			 *  @brief    获得点云坐标
			 *
			 *  @param    PointId idx						点云索引
			 *
			 *  @return   osg::Vec3
			 */
			virtual osg::Vec3 GetXYZ(PointId idx) = 0;

			/**
			 *  @brief    从点云信息转换数据，传入参数符合以下条件：
			 *
			 *				data 存储点云地理坐标，以 UTM 坐标系为例，(316502.984  4720572.821
			 *715.914917)， epsg 为地理坐标系EPSG，以 UTM 50 坐标系为例，EPSG 为 32652，
			 *				对于无效的地理信息或没有地理信息的情况，EPSG 可以传入 -1
			 *
			 *
			 *  @param    const std::vector<POINT> & data	点云信息
			 *  @param    int epsg							地理坐标系EPSG
			 *
			 *  @return   void
			 */
			virtual void ConvertFrom(const std::vector<POINT>& data,
									 const osg::Vec3d& offset,
									 int epsg) = 0;

			/**
			 *  @brief    转换数据到点云信息
			 *
			 *  @param    std::vector<POINT> & data			点云信息
			 *  @param    int & epsg						地理坐标系EPSG
			 *
			 *  @return   void
			 */
			virtual void ConvertTo(std::vector<POINT>& data, osg::Vec3d& offset, int& epsg) = 0;

			/**
			 * 设置大小
			 * @param [in] nSize
			 * @return
			 **/
			virtual void SetSize(size_t size) = 0;

			/**
			 *  @brief    获得点云大小
			 *
			 *  @return   size_t
			 */
			virtual size_t GetSize() const = 0;

			/**
			 *  @brief    设置哦 GEO OGC 地理坐标信息
			 *
			 *  @param    const osg::Vec3d & offset			地理坐标偏移量
			 *  @param    int epsg							地理坐标系EPSG
			 *
			 *  @return   void
			 */
			virtual void SetGeoInfo(const osg::Vec3d& offset, int epsg) = 0;

			/**
			 *  @brief    点云进行降噪
			 *
			 *  @param    double radius						降噪检测范围
			 *  @param    unsigned int minPts				最少点数
			 *  @param    std::vector<int> & inliers		有效点索引
			 *  @param    std::vector<int> & outliers		噪声点索引
			 *
			 *  @return   void
			 */
			virtual void Denoise(double radius,
								 unsigned int minPts,
								 std::vector<int>& inliers,
								 std::vector<int>& outliers) = 0;


			virtual void DenoiseSOR(int meanK,
									double stdMul,
									std::vector<int>& inliers,
									std::vector<int>& outliers) = 0;

			/**
			 * 填充OSG数据
			 * @param [in] vertArray
			 * @param [in] matrix
			 * @param [in] nIndex
			 * @return
			 **/
			virtual void FillOsgPoint(osg::ref_ptr<osg::Vec3Array> vertArray,
									  osg::Matrix& matrix,
									  int nIndex) = 0;
			/**
			 * 填充OSG数据
			 * @param [in] vertArray
			 * @param [in] matrix
			 * @param [in] nIndex
			 * @return
			 **/
			virtual void FillOsgPoint(const osg::Vec3& vert, int nIndex) = 0;
			virtual void FillOsgPoint(const osg::Vec3& vert, uint32_t label, int nIndex) = 0;
		};


		typedef d3s::share_ptr<IPointCloud> IPointCloudPtr;

		// 点云处理接口
		//////////////////////////////////////////////////////////////////////////
		class SEG_EXPORT ICloudProcess : public d3s::ReferenceCountObj
		{
		public:
			virtual ~ICloudProcess() {}

			/**
			 *  @brief   设置点云数据
			 *
			 *  @prarm	 IPointCloudPtr cloud						处理的点云数据
			 *
			 *  @return   bool
			 */
			virtual void SetCloudInput(IPointCloudPtr cloud) = 0;


			/**
			 *  @brief    设置需要处理点云的索引
			 *
			 *  @param    const std::vector<int> & indices			实际处理点的索引
			 *
			 *  @return   bool
			 */
			virtual void SetIndices(const std::vector<PointId>& indices) = 0;
		};


		// 点云分割抽象接口
		//////////////////////////////////////////////////////////////////////////
		class SEG_EXPORT ICloudSegmentation : public ICloudProcess
		{
		public:
			virtual ~ICloudSegmentation() {}

			/**
			 *  @brief    进行点云分割
			 *
			 *  @param    const IOptions * options					分割算法使用参数
			 *  @param    std::vector<int> & result					分割结果点云索引
			 *
			 *  @return   bool
			 */
			virtual bool Segment(const IOptions* options, std::vector<PointId>& result) = 0;
		};

		// 点云检测接口
		//////////////////////////////////////////////////////////////////////////
		class SEG_EXPORT ICloudDetection : public ICloudProcess
		{
		public:
			virtual ~ICloudDetection() {}

			/**
			 *  @brief    检测坐标位置
			 *
			 *  @prarm	 const IOptions * options					检测算法参数
			 *  @prarm	 std::vector<osg::Vec3d> & positions		检测结果
			 *
			 *  @return   bool
			 */
			virtual bool DetectPositions(const IOptions* options,
										 std::vector<osg::Vec3d>& positions) = 0;
		};

		class SEG_EXPORT ICloudReconstrct : public ICloudProcess
		{
		public:
			virtual ~ICloudReconstrct() {}

			/**
			 *  @brief    点云重建
			 *
			 *  @param    const IOptions * options					点云重建算法参数
			 *
			 *  @return   bool
			 */
			virtual bool Reconstruct(const IOptions* options) = 0;
		};

		// 曲线拟合
		//////////////////////////////////////////////////////////////////////////
		class SEG_EXPORT ICloudCurveFitting : public ICloudProcess
		{
		public:
			virtual ~ICloudCurveFitting() {};

			/**
			 *  @brief
			 *
			 *  @param    double expandLength		曲线延长线长度 [0.0, DBL_MAX)
			 *  @param    PARACURVE & params		输出曲线拟合参数
			 *
			 *  @return   bool
			 */
			virtual bool CurveFitting(double expandLength, PARACURVE& params) = 0;
		};

		// 点云聚类
		//////////////////////////////////////////////////////////////////////////
		class SEG_EXPORT ICloudClustering : public ICloudProcess
		{
		public:
			typedef std::vector<std::vector<int>> Clusters;

			virtual ~ICloudClustering() {}

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
										  Clusters& clusters) = 0;

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
											  Clusters& clusters) = 0;
		};

		// 点云栅格化
		//////////////////////////////////////////////////////////////////////////
		class SEG_EXPORT ICloudRasterize : public ICloudProcess
		{
		public:
			virtual ~ICloudRasterize() {}


			/**
			 *  @brief    写入TIFF文件
			 *
			 *  @param    const std::string & filepath		文件路径
			 *  @param    const std::string & srs			TIFF地理投影坐标系
			 *  @param    double resolution					分辨率类似：1.0 代表 1px = 1m
			 *
			 *  @return   void
			 */
			virtual void WriteTiff(const std::string& filepath,
								   const std::string& srs,
								   double resolution) = 0;
		};
	}
}


typedef d3s::pcs::IPointCloudPtr IPointCloudPtr;
typedef d3s::share_ptr<d3s::pcs::IValueBuffer> IValueBufferPtr;
typedef d3s::share_ptr<d3s::pcs::IOptions> IOptionsPtr;
typedef d3s::share_ptr<d3s::pcs::ICloudProcess> ICloudProcessPtr;

#endif // ICLOUD_SEGMENTATION_H_