//////////////////////////////////////////////////////////////////////
// 文件名称：pylonLocation.h
// 功能描述：杆塔快速定位
// 创建标识：吕伟	2022/9/14
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once

#include "gridCell.h"

namespace d3s {
	namespace pcs {

		struct PylonLocationOptions
		{
			double location_cellsize; // 格网单元大小
			double location_zstep;	// 计算fv的垂直间隙

			double location_th1; // T_h1，用于过滤包含低位点的格网单元高度阈值
			double location_th2; // T_h2，用于格网区域生长聚类的相对高度阈值
			double location_tv;	 // T_V，垂直分布系数阈值
			double location_tc;	 // T_C，凸包系数阈值
			double location_th;	 // T_H，高程分布系数阈值
			double location_outlier_coeff; // T_o，杆塔检测点云密度异常值系数
			double location_linearity; // T_LC，线性形状系数阈值
			double location_min_proj;  // T_L，投影长度阈值
			double location_max_span;	// 最大档距
			double location_corridor_width;	// 通道宽度
			int location_neighbors;	   // 格网邻域范围
			
			int max_span_error = 1;		 // 档中最大错误尝试次数
			double min_span_angle_theta; // 档方向与导线方向最小夹角阈值
		};

		// LocationGrid
		//////////////////////////////////////////////////////////////////////////
		class LocationGrid : public Grid2D
		{
		public:
			typedef std::array<int, 2> Index;

			struct CellFeature
			{
				double h = 0.0;	 // 格网高差(最大hag)
				double Fv = 0.0; // 垂直分布系数
				double Fc = 0.0; // 凸包系数
				double Fh = 0.0; // 高程分布系数

				double Fn = 0.0; // 格网超过阈值的点数

				int clusterId = -1;
				int label = -1;
			};

		public:
			LocationGrid();

			LocationGrid(int width, int height);

			LocationGrid(const LocationGrid& rhs);

			/**
			 *  @brief    调整2D网格尺寸
			 *
			 *  @param    int width			宽度
			 *  @param    int height		高度
			 *
			 *  @return   void
			 */
			void resize(int width, int height);

			/**
			 *  @brief    计算格网特征值
			 *
			 *  @prarm	 double tolerance	垂直聚类距离
			 *  @prarm	 int neighbors		格网领域数
			 *
			 *  @return   void
			 */
			void ComputeCellFeatures(double height_threshold, double tolerance, int neighbors);

			/**
			 *  @brief    基于个网格内点云的Z坐标进行欧式聚类
			 *
			 *  @param    const GridCell & cell					格网单元
			 *  @param    double distance						聚类半径
			 *  @param    std::vector<std::vector<int>> & clusters	聚类结果
			 *
			 *  @return   void
			 */
			void ClusteringOnZAxis(const GridCell& cell,
								   double distance,
								   std::vector<std::vector<int>>& clusters);


			/**
			 *  @brief    获得格网特征值
			 *
			 *  @param    int row			行号
			 *  @param    int col			列号
			 *
			 *  @return   std::shared_ptr<d3s::pcs::LocationGrid::CellFeature>
			 */
			std::shared_ptr<CellFeature>& GetFeatureAt(int row, int col);
			std::shared_ptr<CellFeature>& GetFeatureAt(int index);

			const std::shared_ptr<CellFeature>& GetFeatureAt(int row, int col) const;
			const std::shared_ptr<CellFeature>& GetFeatureAt(int index) const;

		protected:
			std::vector<std::shared_ptr<CellFeature>> _cellFeatures;
		};

		typedef std::vector<std::vector<LocationGrid::Index>> LocalClusters;


		// PylonLocation
		//////////////////////////////////////////////////////////////////////////
		class PylonLocation
		{
		public:
			typedef std::shared_ptr<LocationGrid> GridPtr;
			typedef std::vector<osg::Vec3d> Positions;

			PylonLocation(const PylonLocationOptions& options,
						  PointCloudViewPtr input,
						  const std::vector<int>& indices);

			~PylonLocation();

			/**
			 *  @brief    获得格网划分
			 *
			 *  @return   std::shared_ptr<d3s::pcs::LocationGrid>
			 */
			GridPtr GetGrid() const;


			void Location(Positions& pylonPositions);

		private:
			/**
			 *  @brief    杆塔坐标校验
			 *
			 *  @prarm	 PylonPositions & pylonPositions
			 *
			 *  @return   void
			 */
			void PylonPositionValidate(GridPtr grid, Positions& pylonPositions);

			/**
			 *  @brief    杆塔坐标按照输电线路方向排序
			 *
			 *  @prarm	 PylonPositions & pylonPositions
			 *
			 *  @return   void
			 */
			void PylonPositionSorting(Positions& pylonPositions);


			/**
			 *  @brief    获得可能的电力线点云
			 *
			 *  @param    GridPtr grid						点云格网
			 *  @param    const osg::Vec3d & p0				起始点
			 *  @param    const osg::Vec3d & p1				终止点
			 *  @param    double offset						偏移
			 *  @param    double width						走廊宽度
			 *  @param    std::vector<int> & indices		范围内点云
			 *
			 *  @return   void
			 */
			void GetPowerlineIndices(GridPtr grid,
									 const osg::Vec3d& p0,
									 const osg::Vec3d& p1,
									 double offset,
									 double width,
									 std::vector<int>& indices);


			/**
			 *  @brief    获得格网的中心点坐标
			 *
			 *  @prarm	 GridPtr grid	格网
			 *  @prarm	 const std::vector<LocationGrid::Index> & locations	格网位置
			 *
			 *  @return   d3s::pcs::PylonLocation::Positions
			 */
			Positions GetLocationPositions(GridPtr grid,
										   const std::vector<LocationGrid::Index>& locations);

			/**
			 *  @brief    计算杆塔精确坐标
			 *
			 *  @prarm	 const std::vector<LocationGrid::Index> & locations			杆塔定位
			 *
			 *  @return   d3s::pcs::PylonLocation::PylonPositions
			 */
			Positions ComputePylonPositions(const Positions& positions, double cellsize);


			/**
			 *  @brief    定位杆塔，计算杆塔粗略位置
			 *
			 *  @prarm	 const LocalClusters & clusters	格网粗类
			 *  @prarm	 std::vector<LocationGrid::Index> & locations				杆塔定位
			 *
			 *  @return   void
			 */
			/*void ComputePylonLocations(GridPtr grid,
									   const LocalClusters& clusters,
									   std::vector<LocationGrid::Index>& locations);*/

			void ComputePylonLocations(GridPtr grid,
										const LocalClusters& clusters,
										std::vector<LocationGrid::Index>& locations);

			/**
			 *  @brief    移除非候选的类簇
			 *
			 *  @param    LocalClusters & clusters	格网类簇
			 *
			 *  @return   void
			 */
			void Filtering(GridPtr grid, LocalClusters& clusters);

			/**
			 *  @brief    格网聚类
			 *
			 *  @param    LocalClusters & clusters	聚类结果
			 *
			 *  @return   void
			 */
			void Clustering(GridPtr grid, LocalClusters& clusters);

			/**
			 *  @brief    删除低点位格网
			 *
			 *  @return   void
			 */
			void RemoveLowHeightCells(GridPtr grid);


			/**
			 *  @brief    绘制杆塔位置效果
			 *
			 *  @prarm	 const std::vector<LocationGrid::Index> & locations	 杆塔定位
			 *
			 *  @return   void
			 */
			void RenderPylonLocations(GridPtr grid,
									  const LocalClusters& clusters,
									  const std::vector<LocationGrid::Index>& locations);

		private:
			GridPtr _grid;

			PylonLocationOptions _options; // 铁塔点云提取参数
			PointCloudViewPtr _input;	   // 输入点云数据
			std::vector<int> _indices;	   // 处理点云的索引列表

			Positions _pylonPostions; // 铁塔位置
		};

		/**
		 *  @brief    创建格网
		 *
		 *  @return   void
		 */
		void ComputeLocationGridCells(double cellsize,
									  PointCloudViewPtr input,
									  const std::vector<int>& indices,
									  LocationGrid& grid);
	}
}
