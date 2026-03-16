/////////////////////////////////////////////////////////////////////
// 文件名称：PointCloud.h
// 功能描述：点云数据实现
// 创建标识：吕伟	2022/6/26
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef POINTCLOUD_H_
#define POINTCLOUD_H_

#include <src/core/pointTypes.h>

#include <include/ICloudSegmentation.h>

#include <mutex>

namespace d3s {
	namespace pcs {

		// 点云数据模型内部实现
		class CPointCloud : public IPointCloud
		{
		public:
			CPointCloud();

			CPointCloud(PointCloudView<PointPCLH>& pcv);

			virtual ~CPointCloud();

			/**
			 *  @brief    写入点云到文件
			 *
			 *  @param    const std::string & filename		文件名称
			 *
			 *  @return   void
			 */
			virtual void Write(const std::string& filename);

			/**
			 *  @brief    设置点云中点字段值
			 *
			 *  @param    const char * pszField				字段名称
			 *  @param    PointId idx						点云索引
			 *  @param    const void * val					设置值
			 *
			 *  @return   void
			 */
			void SetFieldInternel(const char* pszField, PointId idx, const void* val);

			/**
			 *  @brief    获取点云中点字段值
			 *
			 *  @param    const char * pszField				字段名称
			 *  @param    PointId idx						点云字段
			 *  @param    void * val						获取值
			 *
			 *  @return   void
			 */
			void GetFieldInternel(const char* pszField, PointId idx, void* val);

			/**
			 *  @brief    设置树ID
			 *
			 *  @param    uint32_t treeId
			 *
			 *  @return   void
			 */
			virtual void SetTreeId(PointId idx, uint32_t treeId);

			/**
			 *  @brief    获得树ID
			 *
			 *  @return   uint32_t
			 */
			virtual uint32_t GetTreeId(PointId idx) const;

			/**
			 *  @brief    设置点云类别
			 *
			 *  @param    PointId idx						点云索引
			 *  @param    uint32_t label					类别
			 *
			 *  @return   void
			 */
			virtual void SetClassification(PointId idx, uint32_t label);

			/**
			 *  @brief    获取点云类别
			 *
			 *  @param    PointId idx						点云索引
			 *
			 *  @return   uint32_t
			 */
			virtual uint32_t GetClassification(PointId idx);

			/**
			 *  @brief    获得点云坐标
			 *
			 *  @param    PointId idx						点云索引
			 *
			 *  @return   osg::Vec3
			 */
			virtual osg::Vec3 GetXYZ(PointId idx);

			/**
			 * 设置大小
			 * @param [in] nSize
			 * @return
			 **/
			virtual void SetSize(size_t size);


			/**
			 *  @brief    获得点云点数量
			 *
			 *  @return   size_t
			 */
			virtual size_t GetSize() const;

			/**
			 * 设置坐标信息
			 * @param [in] offset
			 * @param [in] epsg
			 * @return
			 **/
			virtual void SetGeoInfo(const osg::Vec3d& offset, int epsg);

			/**
			 *  @brief    获得点云内存数据
			 *
			 *  @return   d3s::pcs::PointCloudViewPtr
			 */
			PointCloudViewPtr Data();

			PointCloudViewConstPtr Data() const;

			/**
			 *  @brief    从点云信息转换数据，传入参数符合以下条件：
			 *
			 *				data 存储点云地理坐标，以 UTM 坐标系为例，
			 *				(316502.984  4720572.821 715.914917)， epsg 为地理坐标系EPSG，
			 *				以 UTM 50 坐标系为例，EPSG 为 32652，对于无效的地理信息或没有
			 *				地理信息的情况，EPSG 可以传入 -1
			 *
			 *
			 *  @param    const std::vector<POINT> & data	点云信息
			 *  @param    int epsg							地理坐标系EPSG
			 *
			 *  @return   void
			 */
			virtual void ConvertFrom(const std::vector<POINT>& data,
									 const osg::Vec3d& offset,
									 int epsg);

			/**
			 *  @brief    转换数据到点云信息
			 *
			 *  @param    std::vector<POINT> & data			点云信息
			 *  @param    int & epsg						地理坐标系EPSG
			 *
			 *  @return   void
			 */
			virtual void ConvertTo(std::vector<POINT>& data, osg::Vec3d& offset, int& epsg);

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
								 std::vector<int>& outliers);

			virtual void DenoiseSOR(int meanK,
									double stdMul,
									std::vector<int>& inliers,
									std::vector<int>& outliers);

			/**
			 * 填充OSG数据
			 * @param [in] vertArray
			 * @param [in] matrix
			 * @param [in] nIndex
			 * @return
			 **/
			virtual void FillOsgPoint(osg::ref_ptr<osg::Vec3Array> vertArray,
									  osg::Matrix& matrix,
									  int nIndex);
			/**
			 * 填充OSG数据
			 * @param [in] vertArray
			 * @param [in] matrix
			 * @param [in] nIndex
			 * @return
			 **/
			virtual void FillOsgPoint(const osg::Vec3& vert, int nIndex);
			virtual void FillOsgPoint(const osg::Vec3& vert, uint32_t label, int nIndex);

		private:
			PointCloudView<PointPCLH> _pcv; // 点云内存视图，点云数据
			std::mutex _mutex;
		};
	}
}

typedef d3s::share_ptr<d3s::pcs::CPointCloud> PointCloudPtr;

#endif // POINTCLOUD_H_