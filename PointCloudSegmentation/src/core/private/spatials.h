/////////////////////////////////////////////////////////////////////
// 文件名称：spatial_structure.h
// 功能描述：空间接口相关接口
// 创建标识：吕伟	2022/6/14
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef SPATIALS_H_
#define SPATIALS_H_

#include <src/core/pointTypes.h>

#include <osg/Vec3>

#include <pcl/octree/octree.h>

namespace d3s {
	namespace pcs {

		typedef pcl::octree::OctreePointCloudSearch<PointPCLH> Octree;
		typedef pcl::octree::OctreeNode OctreeNode;
		typedef pcl::octree::OctreeKey OctreeKey;

		typedef Octree::BranchNode BranchNode;
		typedef Octree::LeafNode LeafNode;
		typedef Octree::LeafContainer LeafContainer;
		typedef Octree::BranchContainer BranchContainer;

		struct OctreeKeyCompare
		{
			bool operator()(const OctreeKey& lhs, const OctreeKey& rhs) const
			{
				return (!(lhs == rhs) && (lhs <= rhs));
			}
		};

		typedef std::map<OctreeKey, std::vector<int>, OctreeKeyCompare> OctreeNodeSamples;

		struct OctreeTraverseState
		{
			const Octree* octree;
			const OctreeNode* node;
			OctreeKey key;
			unsigned char depth;
		};


		/**
		 *  @brief    根据八叉树索引 key, 获取对应边界范围
		 *
		 *  @param    const Octree * octree								八叉树
		 *  @param    const OctreeKey & key								八叉树体素索引
		 *  @param    unsigned int depth								深度
		 *  @param    osg::Vec3 & pmin									边界最小值
		 *  @param    osg::Vec3 & pmax									边界最大值
		 *
		 *  @return   void
		 */
		void genVoxelBoundsFromOctreeKey(const Octree* octree,
										 const OctreeKey& key,
										 unsigned int depth,
										 osg::Vec3& pmin,
										 osg::Vec3& pmax);

		/**
		 *  @brief    点云加载，并导出为LOD
		 *
		 *  @prarm	 const std::string & input						输入文件路径
		 *  @prarm	 const std::string & outdir						输出文件目录
		 *
		 *  @return   bool
		 */
		bool generatePointCloudLOD_KDTree(const std::string& input, const std::string& outdir);

		bool generatePointCloudLOD_Octree(const std::string& input, const std::string& outdir);

		/**
		 *  @brief   使用八叉树构建 LOD
		 *
		 *  @param    const PointCloudView<PointPCLH> & pcv				点云原始数据
		 *  @param    double resolution									分辨率，最大级别体素的尺寸
		 *  @param    double samplePrecent								采样比例
		 *
		 *  @return   bool
		 */
		bool buildOctreeLOD(const PointCloudView<PointPCLH>& pcv,
							double resolution,
							double samplePrecent,
							osg::ref_ptr<osg::Node>& result);


		/**
		 *  @brief    构建点云节点
		 *
		 *  @param    const PointCloudView<PointPCLH> & pcv				点云原始数据
		 *  @param    osg::ref_ptr<osg::Node> & result
		 *
		 *  @return   bool
		 */
		bool buildCloudNode(const PointCloudView<PointPCLH>& pcv, osg::ref_ptr<osg::Node>& result);


		/**
		 *  @brief    八叉树生成 OpenSceneGraph 的LOD节点
		 *
		 *  @param    OctreeTraverseState state							八叉树遍历状态
		 *  @param    osg::Node * parentNode							父节点
		 *  @param    double samplePrecent								采样比例
		 *  @param    bool pageLOD										是否生成PageLOD
		 *  @param    std::vector<int> & sampleSource					父级从子级采样的点云索引
		 *
		 *  @return   void
		 */
		void buildOpenSceneGraphLOD(OctreeTraverseState state,
									osg::Node* parentNode,
									double samplePrecent,
									OctreeNodeSamples& samples);

		void buildOpenSceneGraphPagedLOD(OctreeTraverseState state,
										 osg::Node* parentNode,
										 double samplePrecent,
										 OctreeNodeSamples& samples);

	}
}

#endif // SPATIALS_H_