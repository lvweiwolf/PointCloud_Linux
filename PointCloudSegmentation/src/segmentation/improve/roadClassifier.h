//////////////////////////////////////////////////////////////////////
// 文件名称：roadClassifier.h
// 功能描述：道路分类算法
// 创建标识：吕伟	2022/10/13
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once

#include "../../../include/ICloudSegmentation.h"

#include "../../algorithm/boundingbox2d.h"

#include "../../segmentation/gridCell.h"


namespace d3s {
	namespace pcs {

		// 虚拟格网
		class VirtualGrid
		{
		public:
			typedef std::array<int, 2> Index;
			typedef std::vector<Index> Cluster;
			typedef std::vector<Cluster> Clusters;
			typedef GridCell* CellPtr;

			/*	typedef std::unordered_map<int, std::shared_ptr<GridCell>> SparseGrid;
				typedef SparseGrid::iterator iterator;*/

			VirtualGrid();

			VirtualGrid(int width, int height);

			VirtualGrid(const VirtualGrid& rhs);

			virtual ~VirtualGrid();

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
			 *  @brief    是否为空
			 *
			 *  @return   bool
			 */
			inline bool empty() const { return _cells.empty(); }

			/**
			 *  @brief    单元数量
			 *
			 *  @return   size_t
			 */
			inline size_t size() const { return _cells.size(); }

			/**
			 *  @brief    获得行数
			 *
			 *  @return   int
			 */
			inline int getNumRows() const { return _nRows; }

			/**
			 *  @brief    获得列数
			 *
			 *  @return   int
			 */
			inline int getNumColumns() const { return _nColumns; }

			/**
			 *  @brief    获得指定位置网格单元
			 *
			 *  @param    int row
			 *  @param    int col
			 *
			 *  @return   d3s::pcs::GridCell&
			 */
			CellPtr& at(int row, int col);
			const CellPtr& at(int row, int col) const;

			CellPtr& at(int npos);
			const CellPtr& at(int npos) const;

			CellPtr& operator[](int npos);
			const CellPtr& operator[](int npos) const;

		protected:
			int _nRows;	   // 行数
			int _nColumns; // 列数

			std::vector<CellPtr> _cells; // 格网单元
		};


		/**
		 *  @brief    创建虚拟格网单元
		 *
		 *  @param    double cellsize						单元大小
		 *  @param    PointCloudViewPtr input				点云数据
		 *  @param    const std::vector<int> & indices		点云索引
		 *  @param    VirtualGrid2D & grid					虚拟格网
		 *
		 *  @return   void
		 */
		void ComputeVirtualGridCells(double cellsize,
									 PointCloudViewPtr input,
									 const std::vector<int>& indices,
									 VirtualGrid& grid);



		struct RoadClassifyOptions
		{
			double cell_size; // 格网单元大小

			double height_threshold; // 高程差阈值
			double slope_threshold;	 // 坡度高差阈值
			double min_area;	 // 最小面积阈值
			double max_area;	// 最大面积
			double shape_threshold;	 // 形状系数阈值
			double linearity_threshold;	// 线性系数阈值
		};

		// 道路点云分类器
		class RoadClassifier
		{
		public:
			typedef std::shared_ptr<VirtualGrid> GridPtr;
			typedef VirtualGrid::Clusters Clusters;


			RoadClassifier(const RoadClassifyOptions& options,
						   PointCloudViewPtr input,
						   const std::vector<int>& indices);

			~RoadClassifier();

			/**
			 *  @brief    设置道路矢量数据
			 *
			 *  @param    d3s::share_ptr<IValueBuffer> roadVectorize
			 *
			 *  @return   void
			 */
			void SetRoadVectorize(d3s::share_ptr<IValueBuffer> roadVectorize);

			void Segment();

		private:

			/**
			*  @brief    根据道路矢量进行约束，分类道路点云
			*
			*  @return   void
			*/
			void GetRoadCandiateIndices(std::vector<int>& indices);

			/**
			*  @brief    常规道路点云分类
			*
			*  @return   void
			*/
			void SegmentNormal();

			/**
			 *  @brief    删除低点位格网
			 *
			 *  @return   void
			 */
			void RemoveLowHeightCells(GridPtr grid);

			/**
			 *  @brief    虚拟格网区域生长聚类
			 *
			 *  @param    GridPtr grid				点云格网
			 *  @param    Clusters & clusters		格网聚类
			 *
			 *  @return   void
			 */
			void Clustering(GridPtr grid, Clusters& clusters);

			/**
			 *  @brief    移除非候选的类簇
			 *
			 *  @param    LocalClusters & clusters	格网类簇
			 *
			 *  @return   void
			 */
			void Filtering(GridPtr grid, Clusters& clusters);

			/**
			*  @brief    通过边界裁剪道路矢量
			*
			*  @prarm	 const BoundingBox2D & bound	矩形边界
			*  @prarm	 std::vector<std::vector<osg::Vec3d>> & roads	道路矢量
			*
			*  @return   void
			*/
			void ClipingWithBound(const BoundingBox2D& bound,
								  std::vector<std::vector<osg::Vec3d>>& roads);



			/**
			 *  @brief    计算格网类簇的面积
			 *
			 *  @param    GridPtr grid				点云格网
			 *  @param    const std::vector<VirtualGrid::Index> & cluster	格网类簇
			 *
			 *  @return   double
			 */
			double ComputeArea(GridPtr grid, const VirtualGrid::Cluster& cluster);


			/**
			 *  @brief    计算格网类簇的形状系数
			 *
			 *  @param    GridPtr grid				点云网格
			 *  @param    const VirtualGrid::Cluster & cluster				格网类簇
			 *
			 *  @return   double
			 */
			double ComputeShapeness(GridPtr grid, const VirtualGrid::Cluster& cluster);

			
			/**
			 *  @brief    计算格网类簇线性系数
			 *
			 *  @param    GridPtr grid
			 *  @param    const VirtualGrid::Cluster & cluster
			 *
			 *  @return   double
			 */
			double ComputeLinearity(GridPtr grid, const VirtualGrid::Cluster& cluster);
						
			/**
			 *  @brief    绘制虚拟格网聚类
			 *
			 *  @param    GridPtr grid				点云格网
			 *  @param    const Clusters & clusters 格网聚类
			 *
			 *  @return   void
			 */
			void RenderVirtualGridClusters(GridPtr grid, const Clusters& clusters);


		private:
			RoadClassifyOptions _options; // 铁塔点云提取参数
			PointCloudViewPtr _input;	  // 输入点云数据
			std::vector<int> _indices;	  // 处理点云的索引列表

			d3s::share_ptr<IValueBuffer> _roadVectorize;
		};
	}
}