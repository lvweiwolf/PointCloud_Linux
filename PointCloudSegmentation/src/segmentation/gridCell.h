//////////////////////////////////////////////////////////////////////
// 文件名称：gridCell.h
// 功能描述：网格单元
// 创建标识：吕伟	2022/4/11
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once

#include "../algorithm/hash.h"
#include "../core/pointTypes.h"
#include <vector>
#include <functional>
#include <unordered_map>
#include <osg/Vec2i>
#include <osg/BoundingBox>
#include <osg/Vec3>
#include <memory>
#define kMinPointsInlinerCell 0
#define kMaxNoEmptyCell 5


namespace d3s {
	namespace pcs {

		extern osg::Vec2i NEIGHBORS_8[8];

		class GridCell : public osg::BoundingBox
		{
		public:
			GridCell();

			GridCell(const osg::Vec3& pmin, const osg::Vec3& pmax);

			GridCell(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax);

			GridCell(const GridCell& rhs);

			GridCell& operator=(const GridCell& rhs);

			// 点云原始数据
			inline void SetInput(PointCloudViewConstPtr input) { _input = input; }
			inline PointCloudViewConstPtr GetInput() const { return _input; }

			// 获得单元内点云数量
			inline size_t GetSize() const { return _indices.size(); }

			// 获得单元内点云索引
			inline const std::vector<int>& GetIndices() const { return _indices; }

			/**
			 *  @brief    向单元添加点
			 *
			 *  @param    int pointIndex				点索引
			 *
			 *  @return   void
			 */
			void AddPoint(int pointIndex);

			/**
			 *  @brief    清空
			 *
			 *  @return   void
			 */
			void Clear();
		
			ClassificationType label;

		protected:
			PointCloudViewConstPtr _input;		// 格网划分输入点云数据
			std::vector<int> _indices;			// 待处理点云索引
		};

		// Grid2D 2D网格
		//////////////////////////////////////////////////////////////////////////
		class Grid2D
		{
		public:
			Grid2D();

			Grid2D(int width, int height);

			Grid2D(const Grid2D& rhs);

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
			GridCell& at(int row, int col);
			const GridCell& at(int row, int col) const;

			GridCell& at(int npos);
			const GridCell& at(int npos) const;

			GridCell& operator[](int npos);
			const GridCell& operator[](int npos) const;

		protected:
			int _nRows;				// 行数
			int _nColumns;			// 列数

			std::vector<GridCell> _cells;		// 网格单元
		};

		// Grid3D 3D网格
		class Grid3D
		{
		public:
			typedef std::array<int, 3> Index;

			Grid3D();

			Grid3D(int xSize, int ySize, int zSize);

			Grid3D(const Grid3D& rhs);

			void resize(int xSize, int ySize, int zSize);

			bool empty() const { return _sparse_cells.empty(); }

			size_t size() const { return _sparse_cells.size(); }

			int getXSize() const { return _xSize; }

			int getYSize() const { return _ySize; }

			int getZSize() const { return _zSize; }

			std::shared_ptr<GridCell>& getOrCreate(int xi, int yi, int zi);
			std::shared_ptr<GridCell>& getOrCreate(const Index& index);

			std::shared_ptr<GridCell> get(int xi, int yi, int zi) const;
			std::shared_ptr<GridCell> get(const Index& index) const;

			void Visit(std::function<void(const std::shared_ptr<GridCell>&)> visitor) const;
			void Visit(std::function<void(Index, const std::shared_ptr<GridCell>&)> visitor) const;

		protected:
			int _xSize;
			int _ySize;
			int _zSize;

			std::unordered_map<Index, std::shared_ptr<GridCell>, hash_eigen<Index>> _sparse_cells;
		};

		//////////////////////////////////////////////////////////////////////////
		/**
		 *  @brief    计算划分网格单元
		 *
		 *  @param    double cellsize							单元大小
		 *  @param    PointCloudViewPtr input					点云数据
		 *  @param    std::vector<GridCell> & grid				格网
		 *
		 *  @return   void
		 */
		void ComputeGridCells(double cellsize,
							  PointCloudViewPtr input,
							  const std::vector<int>& indices,
							  Grid2D& grid);

		/**
		 *  @brief    计算划分网格单元(局部)
		 *
		 *  @param    double cellsize						单元大小
		 *  @param    PointCloudViewPtr input				点云数据
		 *  @param    const std::vector<int> & indices		点云索引
		 *  @param    GridBlock & grid						局部2D格网
		 *
		 *  @return   void
		 */
		void ComputeLocalGridCells(double cellsize,
								   PointCloudViewPtr input,
								   const std::vector<int>& indices,
								   Grid2D& grid);

		/**
		 *  @brief    计算划分网格单元(局部)
		 *
		 *  @param    double cellsize						单元大小
		 *  @param    PointCloudViewPtr input				点云数据
		 *  @param    const std::vector<int> & indices		点云索引
		 *  @param    Grid3D & grid							3D格网
		 *
		 *  @return   void
		 */
		void ComputeLocalGridCells(double cellsize,
								   PointCloudViewPtr input,
								   const std::vector<int>& indices,
								   ClassificationType label,
								   Grid3D& grid);

		/**
		*  @brief    获得单元邻近单元(不包括自身)
		*
		*  @prarm	 Grid2D & gird								2D网格
		*  @prarm	 int r										行号
		*  @prarm	 int c										列号
		*  @prarm	 std::vector<int> & indicesNeighbor			邻近单元点云索引
		*
		*  @return   void
		*/
		void GetCellNeighbors(const Grid2D& gird, int r, int c, std::vector<int>& indicesNeighbor);

		/**
		 *  @brief    基于体素的欧式聚类
		 *
		 *  @param    PointCloudViewPtr input				点云数据
		 *  @param    const std::vector<int> & indices		点云索引
		 *  @param    double tolerance						邻近查找半径
		 *  @param    unsigned int minPts					类簇最小点数
		 *  @param    unsigned int maxPts					类簇最大点数
		 *  @param    std::vector<std::vector<int>> & clusters	聚类结果
		 *
		 *  @return   void
		 */
		void EuclideanVoxelCluster(PointCloudViewPtr input,
								   const std::vector<int>& indices,
								   double tolerance,
								   unsigned int minPts,
								   unsigned int maxPts,
								   std::vector<std::vector<int>>& clusters);
		
	}
}
