//////////////////////////////////////////////////////////////////////
// 文件名称：pylonDecompose.h
// 功能描述：铁塔点云分解铁塔部件算法
// 创建标识：吕伟	2022/12/19
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef PYLON_DECOMPOSE_H_
#define PYLON_DECOMPOSE_H_

#include <src/core/pointTypes.h>

#include <osg/Matrix>
#include <osg/BoundingBox>

#include <vector>

namespace d3s {
	namespace pcs {

		class GenericMesh;

		struct PylonDecomposeOptions
		{
			double head_length;
			double z_step;
			double x_step;
			double window_size;
			double fillrate;
			double s1_ratio;
			double aspect_error;
		};


		class PylonDecomposer
		{
		public:
			PylonDecomposer(const PylonDecomposeOptions& options,
							GenericMesh* mesh,
							PointCloudViewPtr input,
							const std::vector<int>& indices);

			~PylonDecomposer();

			/**
			 *  @brief    横担对齐到X轴方向后的旋转矩阵
			 *
			 *  @return   const osg::Matrix&
			 */
			const osg::Matrix& GetTransform() const;

			/**
			 *  @brief    获得塔头点云索引
			 *
			 *  @return   std::vector<int>
			 */
			std::vector<int> GetPylonHeadIndices() const;

			/**
			 *  @brief    获得塔体点云索引
			 *
			 *  @return   std::vector<int>
			 */
			std::vector<int> GetPylonBodyIndices() const;

			/**
			 *  @brief    获得塔脚点云索引
			 *
			 *  @return   std::vector<int>
			 */
			std::vector<int> GetPylonFootIndices() const;


			/*osg::BoundingBox GetPylonHeadBBox() const;

			osg::BoundingBox GetPylonBodyBBox() const;

			osg::BoundingBox GetPylonFootBBox() const;*/

			/**
			 *  @brief    分解杆塔为：塔头、塔体、塔脚三个部分
			 *
			 *  @return   void
			 */
			void Decompose();

		private:
			/**
			 *  @brief    获得塔头点云索引
			 *
			 *  @param    const osg::BoundingBox & bbox					杆塔边界
			 *  @param    double length									塔头长度
			 *
			 *  @return   std::vector<int>
			 */
			std::vector<int> GetPylonHead(const osg::BoundingBox& bbox, double length);

			/**
			 *  @brief    获得按 Z 轴方向的切分单元
			 *
			 *  @param    double zmin									最小Z值
			 *  @param    double zmax									最大Z值
			 *  @param    double step									单元长度
			 *  @param    std::vector<std::vector<int>> & slices		点云且分层
			 *
			 *  @return   void
			 */
			void GetSlices(double zmin,
						   double zmax,
						   double step,
						   std::vector<std::vector<int>>& slices);


			/**
			 *  @brief    计算关键位置
			 *
			 *  @param    const std::vector<std::vector<int>> & slices	点云切分层
			 *  @param    int size										滑动窗口大小
			 *  @param    std::vector<int> & kp							关键位置
			 *
			 *  @return   void
			 */
			void ComputeKeyPositions(const std::vector<std::vector<int>>& slices,
									 int size,
									 std::vector<int>& kp);


			/**
			 *  @brief    过滤无效的关键位置
			 *
			 *  @param    const std::vector<std::vector<int>> & slices	点云且分层
			 *  @param    std::vector<int> & kp							关键位置
			 *
			 *  @return   void
			 */
			void KeyPositionsFiltering(const std::vector<std::vector<int>>& slices,
									   std::vector<int>& kp);

			/**
			 *  @brief    聚类相邻的关键位置，确保关键位置为一层
			 *
			 *  @param    std::vector<int> & kp							关键位置
			 *
			 *  @return   void
			 */
			void KeyPositionsClustering(std::vector<int>& kp);


			/**
			 *  @brief    计算关键分割位置
			 *
			 *  @param    const std::vector<std::vector<int>> & slices	点云切分层
			 *  @param    const std::vector<int> & kp					关键位置
			 *  @param    std::vector<int> & ksp						关键分割位置
			 *
			 *  @return   void
			 */
			void ComputeKeySegmentPositions(const std::vector<std::vector<int>>& slices,
											const std::vector<int>& kp,
											std::vector<int>& ksp);


		private:
			PylonDecomposeOptions _options;
			GenericMesh* _mesh;
			PointCloudViewPtr _input;  // 输入点云数据
			std::vector<int> _indices; // 处理点云的索引列表
			osg::Matrix _transform;	   // 铁塔重定向旋转矩阵

			std::vector<int> _pylonHeadIndices;
			std::vector<int> _pylonBodyIndices;
			std::vector<int> _pylonFootIndices;
		};
	}
}

#endif // PYLON_DECOMPOSE_H_
