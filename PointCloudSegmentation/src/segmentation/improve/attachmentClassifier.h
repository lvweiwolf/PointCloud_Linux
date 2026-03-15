//////////////////////////////////////////////////////////////////////
// 文件名称：deviceClassifier.h
// 功能描述：绝缘子串分类器
// 创建标识：吕伟	2022/11/3
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once

#include <set>

#include "../../segmentation/gridCell.h"
#include "../../segmentation/powerlineCurve.h"

namespace d3s {
	namespace pcs {

		class Grid2D;

		class PylonGrid3D : public Grid3D
		{
		public:
			struct CellFeature
			{
				int cluster_id;
				bool has_pylon;
				bool has_powerline;

				double x_length;	// X方向长度
				double y_length;	// Y方向长度
				double x_fillrate;	// X方向填充率
				double y_fillrate;	// Y方向填充率
				double xy_fillrate; // XY平面邻域范围内填充率
				double xz_fillrate; // XZ平面邻域范围内填充率
			};

			typedef std::shared_ptr<CellFeature> CellFeaturePtr;

			double cellsize;
			osg::Vec3d center;
			osg::BoundingBox obb;
			osg::Matrix rotation;

		public:
			PylonGrid3D();

			PylonGrid3D(int xSize, int ySize, int zSize);

			PylonGrid3D(const PylonGrid3D& rhs);

			CellFeaturePtr& GetOrCreateFeature(int xi, int yi, int zi);
			CellFeaturePtr& GetOrCreateFeature(const Index& index);

			CellFeaturePtr GetFeature(int xi, int yi, int zi) const;
			CellFeaturePtr GetFeature(const Index& index) const;

			void Visit(std::function<void(const CellFeaturePtr&)> visitor) const;
			void Visit(std::function<void(Index)> visitor) const;

			/**
			 *  @brief    计算格网单元特征值
			 *
			 *  @return   void
			 */
			void ComputeCellFeatures();

		private:
			std::unordered_map<Index, CellFeaturePtr, hash_eigen<Index>> _feature_cells;
		};

		typedef std::vector<PylonGrid3D::Index> AttachmentCluster;
		typedef std::vector<AttachmentCluster> AttachmentClusters;

		// 绝缘子分类选项
		struct AttachmentClassifyOptions
		{
			bool enable;
			double head_cellsize;
			double head_length;

			double voxel_size;	   // 3D格网划分单元尺寸
			double search_radius;  // 邻域搜索半径
			double x_length;	   // X轴方向长度阈值，用于区分塔体
			double y_length;	   // Y轴方向长度阈值，用于区分悬垂串格网类簇
			double z_length;	   // Z轴方向长度阈值，用于区分悬垂串格网类簇
			double x_fillrate;	   // X轴方向格网单元填充率
			double y_fillrate;	   // Y方向格网内点云覆盖率
			double xy_fillrate;	   // XY平面半径邻域格网单元填充率
			double xz_fillrate;	   // XZ平面半径邻域格网单元填充率
			double leadwire_x_fr;  // 引流线X轴方向格网单元填充率
			double leadwire_x_len; // 引流线X轴方向长度阈值
			double suspension_thr; // 悬垂绝缘子特征阈值
			double strain_thr;	   // 耐张绝缘子特征阈值

			std::string suspension_method;	// 悬垂绝缘子特征计算方法
			std::string strain_method;		// 耐张绝缘子特征计算方法

			/* 内参数 */
			int connection_mask_distance = 4;
			int ignore_mask_distance = 5;
			double subsample_size = 0.05;
		};

		// 绝缘子分类器
		class AttachmentClassifier
		{
		public:
			typedef std::vector<osg::Vec3d> Positions;
			typedef std::vector<osg::Vec3d> Vectors;
			typedef std::vector<osg::Vec3d> Polygon;
			typedef std::vector<Polygon> Polygons;
			typedef std::vector<std::array<int, 3>> GrowDirection;

			typedef std::function<bool(const AttachmentClassifyOptions&,
									   const PylonGrid3D&,
									   const PylonGrid3D::Index&)>
				GrowCondition;

			typedef std::function<bool(PylonGrid3D::CellFeaturePtr)> FeatureCondition;

			AttachmentClassifier(const AttachmentClassifyOptions& options,
								 PointCloudViewPtr input,
								 const std::vector<int>& indices);

			~AttachmentClassifier();

			/**
			 *  @brief    设置铁塔位置
			 *
			 *  @prarm	 const Positions & pylonPositions
			 *
			 *  @return   void
			 */
			void SetPylonPositions(const Positions& pylonPositions);

			/**
			 *  @brief    设置铁塔边界
			 *
			 *  @prarm	 const Polygons & pylonPolygons
			 *
			 *  @return   void
			 */
			void SetPylonPolygons(const Polygons& pylonPolygons);

			/**
			 *  @brief    设置杆塔横担方向
			 *
			 *  @param    const Vectors & pylonSides
			 *
			 *  @return   void
			 */
			void SetPylonSides(const Vectors& pylonSides);

			/**
			 *  @brief    设置杆塔连接点
			 *
			 *  @param    const Connections & pylonConnections
			 *
			 *  @return   void
			 */
			void SetPylonConnections(const PylonConnections& pylonConnections);

			/**
			 *  @brief    执行分割程序
			 *
			 *  @return   void
			 */
			void Segment();

		private:

			void SegmentStrain(const PylonGrid3D& grid,
							   const PylonGrid3D::Index& index,
							   AttachmentCluster& strain,
							   AttachmentCluster& leadwire,
							   std::vector<AttachmentCluster>& suspensions,
							   std::set<int>& strainMask);

			/**
			 *  @brief    设置类簇标签
			 *
			 *  @prarm	 const PylonGrid3D & grid						杆塔点云格网
			 *  @prarm	 const AttachmentCluster & cluster				绝缘子格网单元类簇
			 *  @prarm	 int label										标签
			 *
			 *  @return   void
			 */
			void SetClusterLabel(const PylonGrid3D& grid,
								 const AttachmentCluster& cluster,
								 int label);

			/**
			 *  @brief    获得杆塔边界范围内杆塔与电力线点云索引
			 *
			 *  @prarm	 const Grid2D & grid							点云格网
			 *  @prarm	 const Polygon & polygon						杆塔边界范围
			 *  @prarm	 std::vector<int> & pylonIndices				杆塔点云索引
			 *  @prarm	 std::vector<int> & powerlineIndices			电力线点云索引
			 *
			 *  @return   void
			 */
			void GetIndices(const Grid2D& grid,
							const Polygon& polygon,
							std::vector<int>& pylonIndices,
							std::vector<int>& powerlineIndices);


			std::vector<int> GetIndices(const PylonGrid3D& grid, const AttachmentCluster& cluster);

			/**
			 *  @brief    获得挂点附近电力线点云索引
			 *
			 *  @prarm	 const Grid2D & grid							点云格网
			 *  @prarm	 const PylonConnection & connection				杆塔挂点位置
			 *  @prarm	 double radius									格网单元检索半径
			 *
			 *  @return   void
			 */
			std::vector<int> GetPowerlineSurround(const PylonGrid3D& grid,
												  const PylonConnection& connection);
			
			/**
			 *  @brief    获得杆塔OBB包围盒
			 *
			 *  @prarm	 const std::vector<int> & pylonIndices			杆塔点云
			 *  @prarm	 const std::vector<int> & powerlineIndices		电力线点云
			 *  @prarm	 int i											杆塔线路索引，帮助计算偏转
			 *  @prarm	 osg::BoundingBox & obb							OBB包围盒
			 *  @prarm	 osg::Vec3d & center							中心点
			 *  @prarm	 osg::Matrix & rotation							旋转矩阵
			 *
			 *  @return   void
			 */
			void GetPylonOBB(const std::vector<int>& pylonIndices,
							 const std::vector<int>& powerlineIndices,
							 int,
							 osg::BoundingBox& obb,
							 osg::Vec3d& center,
							 osg::Matrix& rotation);

			/**
			 *  @brief    移除需要忽略的导线连接点
			 *
			 *  @param    const osg::Vec3d & pos							杆塔坐标
			 *  @param    PylonConnection & connection						导线连接挂点
			 *
			 *  @return   void
			 */
			void RemoveConnectionIgnored(const osg::Vec3d& pos, PylonConnection& connection);


			AttachmentCluster RemoveClusterInRadius(const AttachmentCluster& cluster,
													const PylonGrid3D::Index& index,
													int distance);

			/**
			 *  @brief    创造领域单元
			 *
			 *  @param    const PylonGrid3D & grid						杆塔点云格网
			 *  @param    int numNeighbor								邻域搜索范围
			 *  @param    PylonGrid3D::Index & index					查找位置
			 *
			 *  @return   bool
			 */
			bool FindNeighbors(const PylonGrid3D& grid,
							   int numNeighbor,
							   PylonGrid3D::Index& index,
							   FeatureCondition condition);


			/**
			 *  @brief    获得杆塔点云格网中距离连接点最近单元
			 *
			 *  @prarm	 const PylonGrid3D & grid						杆塔点云格网
			 *  @prarm	 const osg::Vec3d & connectPoint				挂点坐标
			 *  @prarm	 PylonGrid3D::Index & index						格网单元索引
			 *
			 *  @return   bool
			 */
			bool GetNearestIndex(const PylonGrid3D& grid,
								 const osg::Vec3d& connectPoint,
								 PylonGrid3D::Index& index,
								 FeatureCondition condition);


			bool GetNearestIndex(const PylonGrid3D& grid,
								 const PylonGrid3D::Index& index,
								 PylonGrid3D::Index& nearestIndex,
								 FeatureCondition condition);


			/**
			 *  @brief    在类簇中获得距离挂点最远端的格网单元
			 *
			 *  @param    const PylonGrid3D & grid						杆塔点云格网
			 *  @param    const AttachmentCluster & cluster				绝缘子候选格网单元类簇
			 *  @param    const osg::Vec3d & connectPoint				导线连接点位置
			 *  @param    PylonGrid3D::Index & farthestIndex			最远端格网单元索引
			 *
			 *  @return   bool
			 */
			bool GetFarthestIndex(const PylonGrid3D& grid,
								  const AttachmentCluster& cluster,
								  const osg::Vec3d& connectPoint,
								  PylonGrid3D::Index& farthestIndex);

			/**
			*  @brief    修复引流线格网类簇
			*
			*  @prarm	 const PylonGrid3D & grid						杆塔点云格网
			*  @prarm	 AttachmentCluster & cluster					绝缘子格网类簇
			*
			*  @return   void
			*/
			void GetSuspensionSeeds(const PylonGrid3D& grid,
							  AttachmentCluster& cluster,
							  AttachmentCluster& suspension);

			/**
			 *  @brief
			 *
			 *  @prarm	 const PylonGrid3D & grid						杆塔点云格网
			 *  @prarm	 const PylonGrid3D::Index & seedIndex			生长种子格网单元
			 *  @prarm	 const GrowDirection & growDirect				生长方向
			 *  @prarm	 const GrowCondition & condition				生长条件
			 *
			 *  @return   d3s::pcs::AttachmentCluster					绝缘子候选格网单元类簇
			 */
			AttachmentCluster Growing(const PylonGrid3D& grid,
									  const PylonGrid3D::Index& seedIndex,
									  const GrowDirection& growDirect,
									  const GrowCondition& condition);

			/**
			 *  @brief    引流线区域生长
			 *
			 *  @prarm	 const PylonGrid3D & grid						杆塔点云格网
			 *  @prarm	 const AttachmentCluster & seeds				格网单元种子
			 *  @prarm	 const GrowDirection & growDirect				生长方向
			 *  @prarm	 const GrowCondition & condition				生长条件
			 *  @prarm	 bool  ignoreSeed								是否忽略种子
			 *
			 *  @return   d3s::pcs::AttachmentCluster					引流线候选格网类簇
			 */
			AttachmentCluster Growing(const PylonGrid3D& grid,
									  const AttachmentCluster& seeds,
									  const GrowDirection& growDirect,
									  const GrowCondition& condition,
									  bool ignoreSeed);



			/**
			 *  @brief    是否为悬垂绝缘子串
			 *
			 *  @prarm	 const PylonGrid3D & grid						杆塔点云格网
			 *  @prarm	 const AttachmentCluster & cluster				绝缘子格网单元类簇
			 *  @prarm	 const osg::Vec3d & connectPoint				挂点坐标
			 *
			 *  @return   bool
			 */
			bool IsSuspension(const PylonGrid3D& grid,
							  const AttachmentCluster& cluster,
							  const osg::Vec3d& connectPoint) const;

			bool IsSuspension(const PylonGrid3D& grid, const AttachmentCluster& cluster) const;


			/**
			 *  @brief    是否为耐张绝缘子串
			 *
			 *  @prarm	 const PylonGrid3D & grid						杆塔点云格网
			 *  @prarm	 const AttachmentCluster & cluster				绝缘子格网单元类簇
			 *
			 *  @return   bool
			 */
			bool IsStrain(const PylonGrid3D& grid, const AttachmentCluster& cluster);


			/**
			 *  @brief    从电力线点云中修复悬垂绝缘子串
			 *
			 *  @prarm	 const PylonGrid3D & grid						杆塔点云格网
			 *  @prarm	 const std::vector<int> & powerlines			电力线点云
			 *  @prarm	 std::vector<int> & attachments					输出悬垂串点云
			 *
			 *  @return   void
			 */
			void FixSuspensionAttachment(const PylonGrid3D& grid,
										 const std::vector<int>& powerlines,
										 std::vector<int>& attachments);

			/**
			 *  @brief    修复耐张绝缘子串格网类簇
			 *
			 *  @param    const PylonGrid3D & grid						杆塔点云格网
			 *  @param    AttachmentCluster & cluster					绝缘子格网类簇
			 *
			 *  @return   void
			 */
			void FixStrainAttachment(const PylonGrid3D& grid, AttachmentCluster& cluster);
			

			/**
			 *  @brief    修复挂点附近“过度”分类为电力线的杆塔点云
			 *
			 *  @param    const PylonGrid3D & grid						杆塔点云格网
			 *  @param    const osg::Vec3d & connectPoint				挂点坐标
			 *  @param    double radius									挂点附近搜索半径
			 *  @param    std::vector<int> & powerlines					错误电力线点云
			 *
			 *  @return   void
			 */
			void FixPowerlineNearConnectPoint(const PylonGrid3D& grid,
											  const osg::Vec3d& connectPoint,
											  double radius,
											  std::vector<int>& powerlines);


			void FixLeadwires(const PylonGrid3D& grid,
							  const PylonGrid3D::Index& connectIndex,
							  const AttachmentCluster& leadwire,
							  const std::vector<AttachmentCluster>& suspensions,
							  const std::set<int>& strainMask);


			void FixSuspensionInLeadwire(const PylonGrid3D& grid,
										 const std::vector<int>& leadwireIndices,
										 const std::vector<AttachmentCluster>& suspensions);


			void AttachmentClustering(const PylonGrid3D& grid,
									  const AttachmentCluster& seeds,
									  std::vector<AttachmentCluster>& clusters);

		private:
			AttachmentClassifyOptions _options; // 杆塔点云细化参数
			PointCloudViewPtr _input;			// 输入点云数据
			std::vector<int> _indices;			// 处理点云的索引列表

			Positions _pylonPositions;
			Polygons _pylonPolygons;
			Vectors _pylonSides;

			PylonConnections _pylonConnections;

			GrowDirection _GROW_UP;
			GrowDirection _GROW_FRONT;
			GrowDirection _GROW_BACK;
			GrowDirection _GROW_ALL;
		};


		/**
		 *  @brief    创建杆塔格网
		 *
		 *  @return   void
		 */
		void ComputePylonGrid(double cellsize,
							  const osg::Vec3d& center,
							  const osg::BoundingBox& obb,
							  const osg::Matrix& rotation,
							  PointCloudViewPtr input,
							  const std::vector<int>& indices,
							  PylonGrid3D& grid);
	}
}