#ifndef PLOT_HANDLE_H_
#define PLOT_HANDLE_H_

#include <include/ClassificationDef.h>
#include <src/core/pointTypes.h>

#include <osg/Node>

#include <string>
#include <vector>
#include <unordered_map>

namespace d3s {
	namespace pcs {

		class Grid2D;
		class Grid3D;
		class LocationGrid;
		class VirtualGrid;
		class PylonGrid3D;

		class PlotHandler
		{
		public:
			static PlotHandler* GetInst();

			/**
			 *  @brief    添加节点到渲染树，并为它起名字
			 *
			 *  @param    const std::string & name			节点名称
			 *  @param    osg::ref_ptr<osg::Node> node		节点对象
			 *  @param    bool dynamic						是否插入到动态渲染树
			 *
			 *  @return   bool
			 */
			bool AddNode(const std::string& name,
						 osg::ref_ptr<osg::Node> node,
						 bool dynamic = true);

			/**
			 *  @brief    删除指定名字的节点
			 *
			 *  @param    const std::string & name			节点名称
			 *  @param    bool dynamic						是否从动态树下删除
			 *
			 *  @return   bool
			 */
			bool RemoveNode(const std::string& name, bool dynamic = true);

			/**
			 *  @brief    获得节点
			 *
			 *  @param    const std::string & name			节点名称
			 *  @param    bool dynamic						是否从动态树下检索
			 *
			 *  @return   osg::ref_ptr<osg::Node>
			 */
			osg::ref_ptr<osg::Node> GetNode(const std::string& name, bool dynamic = true);

		protected:
			PlotHandler();

			~PlotHandler() {};

			static PlotHandler* inst;

			std::map<std::string, osg::ref_ptr<osg::Node>> _nodeNamed;
		};

		struct ColorManager
		{
			static osg::Vec4ub colorTable[eNumOfClassification];
			static osg::Vec4 colorMap[256];							   // 高程着色，颜色映射表
			static std::unordered_map<int, int> powerlineClusterLabel; // 导线按回路着色
			static std::vector<int> treeSegments;					   // 点云索引与树木ID的映射
			static std::vector<int> treeSpecies;					   // 点云索引与树种类别的映射
			static std::vector<int> debugTags;
			static std::unordered_map<int, osg::Vec4> treeColorMap;
			static std::unordered_map<int, osg::Vec4> treeSpeciesColorMap;
			static std::unordered_map<int, osg::Vec4> debugColorMap;

			static void CreateColorMap();
		};

		/**
		 *  @brief    更新坐标轴节点
		 *
		 *  @return   void
		 */
		void UpdateAxisPlot();

		/**
		 *  @brief    根据点云类别渲染颜色
		 *
		 *  @param    const std::string& name					节点名称
		 *  @param    PointCloudViewConstPtr pointCloudPtr		点云数据
		 *
		 *  @return   void
		 */
		void RenderColorFromClassifcation(const std::string& name,
										  PointCloudViewConstPtr pointCloudPtr);


		/**
		 *  @brief    根据点云原始RGB渲染颜色
		 *
		 *  @param    const std::string & name					节点名称
		 *  @param    PointCloudViewConstPtr pointCloudPtr		点云数据
		 *
		 *  @return   void
		 */
		void RenderColorFromRGB(const std::string& name, PointCloudViewConstPtr pointCloudPtr);


		/**
		 *  @brief    根据点云高程渲染颜色
		 *
		 *  @param    const std::string & name					节点名称
		 *  @param    PointCloudViewConstPtr pointCloudPtr		点云数据
		 *
		 *  @return   void
		 */
		void RenderColorFromElevation(const std::string& name,
									  PointCloudViewConstPtr pointCloudPtr);

		/**
		 *  @brief    根据归一化点云渲染颜色
		 *
		 *  @param    const std::string & name					节点名称
		 *  @param    PointCloudViewConstPtr pointCloudPtr		点云数据
		 *
		 *  @return   void
		 */
		void RenderColorFromNormalizeElevation(const std::string& name,
											   PointCloudViewConstPtr pointCloudPtr);


		/**
		 *  @brief    根据导线聚类渲染颜色
		 *
		 *  @param    const std::string & name					节点名称
		 *  @param    PointCloudViewConstPtr pointCloudPtr		点云数据
		 *
		 *  @return   void
		 */
		void RenderColorFromCluster(const std::string& name, PointCloudViewConstPtr pointCloudPtr);


		/**
		 *  @brief    根据单木分割渲染颜色
		 *
		 *  @param    const std::string & name					节点名称
		 *  @param    PointCloudViewConstPtr pointCloudPtr		点云数据
		 *
		 *  @return   void
		 */
		void RenderColorFromTreeSegmentation(const std::string& name,
											 PointCloudViewConstPtr pointCloudPtr);

		/**
		 *  @brief    根据树种分类渲染颜色
		 *
		 *  @param    const std::string & name					节点名称
		 *  @param    PointCloudViewConstPtr pointCloudPtr		点云数据
		 *
		 *  @return   void
		 */
		void RenderColorFromTreeSpceiesClassification(const std::string& name,
													  PointCloudViewConstPtr pointCloudPtr);

		void RenderColorFromDebugTag(const std::string& name, PointCloudViewConstPtr pointCloudPtr);

		/**
		 *  @brief    显示指定类别的点云，其他类别隐藏
		 *
		 *  @param    const std::string & name					节点名称
		 *  @param    PointCloudViewConstPtr pointCloudPtr		点云数据
		 *  @param    int classfication							类别
		 *
		 *  @return   void
		 */
		void ShowExceptClassfication(const std::string& name,
									 PointCloudViewConstPtr pointCloudPtr,
									 int classfication);


		/**
		 *  @brief    渲染网格单元
		 *
		 *  @prarm	 double cellsize							单元大小
		 *  @prarm	 double step								垂直步长
		 *  @prarm	 PointCloudViewPtr pointCloudPtr			点云数据
		 *
		 *  @return   void
		 */
		void RenderGridCells(double cellsize, double step, PointCloudViewPtr pointCloudPtr);

		void RenderGridCells(const Grid3D& grid, PointCloudViewPtr pointCloudPtr);

		void RenderGridCells(const LocationGrid& grid,
							 const std::vector<osg::Vec4>& colorMap,
							 PointCloudViewPtr pointCloudPtr);

		void RenderGridCells(const VirtualGrid& grid,
							 const std::vector<std::vector<std::array<int, 2>>>& clusters,
							 const std::vector<osg::Vec4>& colorMap,
							 PointCloudViewPtr pointCloudPtr);

		void RenderBlocks(const std::vector<double>& xlist,
						  const std::vector<double>& ylist,
						  double blockSize,
						  PointCloudViewPtr pointCloudPtr);


		/**
		 *  @brief    绘制杆塔格网
		 *
		 *  @prarm	 const PylonGrid3D & grid					杆塔点云格网
		 *  @prarm	 PointCloudViewPtr pointCloudPtr			原始点云数据
		 *
		 *  @return   void
		 */
		void RenderPylonGrid(const PylonGrid3D& grid, PointCloudViewPtr pointCloudPtr);


		/**
		 *  @brief	  绘制杆塔坐标标记
		 *
		 *  @param    const std::vector<osg::Vec3d> & positions	杆塔坐标数组
		 *  @param    double radius								标记大小
		 *  @param    PointCloudViewPtr pointCloudPtr			点云数据
		 *
		 *  @return   void
		 */
		void RenderPylonPositions(const std::vector<osg::Vec3d>& positions,
								  double radius,
								  PointCloudViewPtr pointCloudPtr);

		/**
		 *  @brief    绘制位置标记
		 *
		 *  @prarm	 const std::string & name				名称
		 *  @prarm	 const osg::Vec3d & position			坐标
		 *  @prarm	 double radius							半径
		 *  @prarm	 PointCloudViewPtr pointCloudPtr		点云数据
		 *
		 *  @return   void
		 */
		void RenderPosition(const std::string& name,
							const osg::Vec3d& position,
							double radius,
							PointCloudViewPtr pointCloudPtr,
							osg::Vec4 color = osg::Vec4(1, 0, 0, 1));

		/**
		 *  @brief    绘制铁塔路径走向
		 *
		 *  @param    const std::vector<osg::Vec3d> & path		路径顶点
		 *  @param    PointCloudViewPtr pointCloudPtr			点云数据
		 *
		 *  @return   void
		 */
		void RenderLinePath(const std::string& name,
							const std::vector<osg::Vec3d>& path,
							PointCloudViewPtr pointCloudPtr,
							osg::Vec4 color = osg::Vec4(0.f, 1.f, 0.f, 0.5f));

		void RenderPowerlineCurve(const std::string& name,
								  const std::vector<osg::Vec3d>& verts,
								  PointCloudViewPtr pointCloudPtr,
								  osg::Vec4 color);


		void RenderPylonMask(const std::vector<osg::Vec3d>& path,
							 PointCloudViewPtr pointCloudPtr,
							 std::string name = "pylonmask0");


		void RenderPowerlineMask(const std::vector<osg::Vec3d>& path,
								 PointCloudViewPtr pointCloudPtr,
								 std::string name = "powerlinemask0",
								 osg::Vec4 color = osg::Vec4(1.f, 0.f, 0.f, 1.f));


		void RenderAABB(const osg::BoundingBox& aabb,
						PointCloudViewPtr pointCloudPtr,
						std::string name = "aabb",
						osg::Vec4 fill = osg::Vec4(0.f, 1.f, 0.f, 0.f),
						osg::Vec4 border = osg::Vec4(0.f, 1.f, 0.f, 1.f));

		void RenderOBB(const osg::BoundingBox& aabb,
					   const osg::Vec3d& position,
					   const osg::Matrix& rotation,
					   PointCloudViewPtr pointCloudPtr,
					   std::string name = "obb");


		void RenderAddMatrix(const std::string& name, const osg::Matrix& matrix);


		void RenderKeyPositions(const std::vector<int>& kp,
								const osg::Vec3d& minPt,
								const osg::Vec3d& maxPt,
								const osg::Matrix& inverseTransform,
								double step,
								PointCloudViewPtr pointCloudPtr,
								std::string name = "kp");

	}
}

#endif // PLOT_HANDLE_H_
