//stdafx.h
#include "spatials.h"

#include <numeric>

#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/LOD>

#include "filters.h"
#include "../../plot/geomCreator.h"
#include "../../utils/logging.h"
#include "../../utils/timer.h"

#define kMaximumVisiableDistanceOfTopLevel 1e30

namespace d3s {
	namespace pcs {

		namespace {

			inline float unifRand()
			{
			
				return (static_cast<float>(rand() / double(RAND_MAX)));
				// return (((214013 * seed_ + 2531011) >> 16) & 0x7FFF);
			}

			static unsigned int SEED(static_cast<unsigned int>(time(NULL)));
		}


		void genVoxelBoundsFromOctreeKey(const Octree* octree,
										 const OctreeKey& key,
										 unsigned int depth,
										 osg::Vec3& pmin,
										 osg::Vec3& pmax)
		{
			CHECK_MSG(octree, "参数传递错误！");

			if (!octree)
				return;

			// 获得点云边界
			double min_x_;
			double max_x_;
			double min_y_;
			double max_y_;
			double min_z_;
			double max_z_;
			octree->getBoundingBox(min_x_, min_y_, min_z_, max_x_, max_y_, max_z_);

			// 计算当前树深度的体素大小
			double voxel_side_len = octree->getResolution() *
									static_cast<double>(1 << (octree->getTreeDepth() - depth));



			// 计算体素边界
			pmin[0] = static_cast<float>(static_cast<double>(key.x) * voxel_side_len + min_x_);
			pmin[1] = static_cast<float>(static_cast<double>(key.y) * voxel_side_len + min_y_);
			pmin[2] = static_cast<float>(static_cast<double>(key.z) * voxel_side_len + min_z_);
			pmax[0] = static_cast<float>(static_cast<double>(key.x + 1) * voxel_side_len + min_x_);
			pmax[1] = static_cast<float>(static_cast<double>(key.y + 1) * voxel_side_len + min_y_);
			pmax[2] = static_cast<float>(static_cast<double>(key.z + 1) * voxel_side_len + min_z_);
		}


		bool buildOctreeLOD(const PointCloudView<PointPCLH>& pcv,
							double resolution,
							double samplePrecent,
							osg::ref_ptr<osg::Node>& result)
		{
			Timer timer;
			timer.Start();

			Octree octree(resolution);

			// 初始化八叉树
			{
				pcl::PointCloud<PointPCLH>::Ptr cloud(new pcl::PointCloud<PointPCLH>);
				cloud->points = pcv.points;
				cloud->width = cloud->points.size();
				cloud->height = 1;

				octree.setInputCloud(cloud);	  // 设置点云数据
				octree.defineBoundingBox();		  // 自动更新边界框
				octree.addPointsFromInputCloud(); // 从点云数据添加点
			}

			osg::Vec3d bbmin, bbmax;
			octree.getBoundingBox(bbmin[0], bbmin[1], bbmin[2], bbmax[0], bbmax[1], bbmax[2]);

			// 八叉树深度
			PCS_INFO("[buildOctreeLOD] octree depth: %d.",
									  octree.getTreeDepth());

			unsigned int numNodes = 0;
			const OctreeNode* root = (*octree.begin());

			if (!root)
			{
				PCS_ERROR("octree root node is null.");
				return false;
			}

			OctreeTraverseState entry;
			entry.octree = &octree;
			entry.node = root;
			entry.depth = 0;
			entry.key.x = entry.key.y = entry.key.z = 0;

			// 检索八叉树
			OctreeNodeSamples samples;
			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			osg::ref_ptr<osg::LOD> rootLod = new osg::LOD;
			buildOpenSceneGraphLOD(entry, rootLod.get(), samplePrecent, samples);

			// 添加地理坐标偏移量
			transform->setMatrix(osg::Matrixd::translate(pcv.offset_xyz));
			transform->addChild(rootLod);

			result = transform;

			PCS_INFO("[buildOctreeLOD] 生成八叉树LOD结构  %.3f s.", timer.ElapsedSeconds());

			return true;
		}


		bool buildCloudNode(const PointCloudView<PointPCLH>& pcv, osg::ref_ptr<osg::Node>& result)
		{
			Timer timer;
			timer.Start();

			// 拷贝点
			pcl::PointCloud<PointPCLH>::Ptr cloud(new pcl::PointCloud<PointPCLH>);
			cloud->points = pcv.points;
			cloud->width = cloud->points.size();
			cloud->height = 1;

			// 添加地理坐标偏移量
			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			transform->setMatrix(osg::Matrixd::translate(pcv.offset_xyz));

			std::vector<int> indices(cloud->size());
			std::iota(indices.begin(), indices.end(), 0);

			osg::ref_ptr<osg::Geode> geode = CreatePointCloud(*cloud, indices);
			transform->addChild(geode);
			result = transform;

			PCS_INFO("[buildCloudNode] 构建点云节点  %.3f s.",
									  timer.ElapsedSeconds());

			return true;
		}

		void buildOpenSceneGraphLOD(OctreeTraverseState entry,
									osg::Node* parentNode,
									double samplePrecent,
									OctreeNodeSamples& samples)
		{
			if (!entry.node)
				return;

			if (!parentNode)
				return;

			// 更新当前深度
			entry.depth++;

			// 当前八叉树索引
			OctreeKey& currentKey = entry.key;

			// 当前节点不是叶节点，判断是否已经生成了LOD，如果没有生成，则生成它
			if (entry.node->getNodeType() == pcl::octree::BRANCH_NODE)
			{
				bool bret = true;
				unsigned int numLeaf = 0;
				const BranchNode* branch = static_cast<const BranchNode*>(entry.node);

				osg::ref_ptr<osg::Group> group = new osg::Group;
				std::vector<int> samplesOfDepth;

				// 遍历子节点
				for (unsigned char childIdx = 0; childIdx < 8; ++childIdx)
				{
					const OctreeNode* childNode = branch->getChildPtr(childIdx);

					if (childNode)
					{
						currentKey.pushBranch(childIdx);
						entry.node = childNode;

						if (childNode->getNodeType() == pcl::octree::BRANCH_NODE)
						{
							osg::ref_ptr<osg::LOD> childLod = new osg::LOD();
							group->addChild(childLod);

							//  处理子节点
							buildOpenSceneGraphLOD(entry, childLod, samplePrecent, samples);
						}
						else
						{
							//  处理子节点
							buildOpenSceneGraphLOD(entry, group, samplePrecent, samples);
							++numLeaf;
						}

						std::vector<int>& temp = samples[currentKey];
						samplesOfDepth.insert(samplesOfDepth.end(), temp.begin(), temp.end());

						currentKey.popBranch();
					}
				}

				// 从子节点采样点云
				if (!samplesOfDepth.empty())
				{

					/*PCS_INFO(
					"Branch节点: depth:%d, 子级点云数: %d, 本级采样数: %d",
					entry.depth,
					samplesOfDepth.size(),
					sampleSize);*/

					std::vector<int> temp;

					if (samplesOfDepth.size() > 10)
					{
						samplePrecent = std::max(0.0, std::min(1.0, samplePrecent));

						unsigned int sampleSize =
							(unsigned int)((double)samplesOfDepth.size() * samplePrecent);

						//randomSample(samplesOfDepth, sampleSize, temp);
						temp = randomSampling(samplesOfDepth, sampleSize);
					}
					else
						temp = samplesOfDepth;


					// 如果子节点是叶子节点，则直接将叶子节点组成 osg::Group 添加即可，
					// 否则，添加随机采样生成点云
					if (!numLeaf)
					{
						osg::ref_ptr<osg::Geode> geode =
							CreatePointCloud(*(entry.octree->getInputCloud()), temp);

						parentNode->asGroup()->addChild(geode);
					}

					std::swap(samples[currentKey], temp);
				}

				parentNode->asGroup()->addChild(group);
			}
			else if (entry.node->getNodeType() == pcl::octree::LEAF_NODE)
			{
				const LeafNode* leaf = static_cast<const LeafNode*>(entry.node);
				const LeafContainer* leafContainer = leaf->getContainerPtr();

				if (!leafContainer)
					return;

				std::vector<int> temp;
				// 叶子节点获得包含的点云索引
				leafContainer->getPointIndices(temp);

				// 创建点云 Geode
				osg::ref_ptr<osg::Geode> geode =
					CreatePointCloud(*(entry.octree->getInputCloud()), temp);

				parentNode->asGroup()->addChild(geode);

				/*PCS_INFO("Leaf节点: depth: %d, 点云数: %d",
										  entry.depth,
										  temp.size());*/

				std::swap(samples[currentKey], temp);
			}

			// 还原深度
			entry.depth--;

			osg::LOD* parentLod = dynamic_cast<osg::LOD*>(parentNode);

			if (parentLod)
			{
				int depth = entry.depth;

				// 设置LOD相关显示范围
				/*osg::Vec3 pmin, pmax;
				genVoxelBoundsFromOctreeKey(entry.octree, entry.key, depth, pmin, pmax);
				osg::BoundingBox bbox(pmin - *entry.offset, pmax - *entry.offset);
				float radius = bbox.radius();*/

				float radius = parentLod->getBound().radius();
				float cutOffDistance = 2.0f * radius;
				float farDistance = kMaximumVisiableDistanceOfTopLevel;

				/*PCS_INFO("depth: %d, radius: %lf, cutOffDistance: %lf",
										  depth,
										  radius,
										  cutOffDistance);*/

				parentLod->setRange(1, 0, cutOffDistance);
				parentLod->setRange(0, cutOffDistance, farDistance);
				// parentLod->setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);

				if (parentLod->getNumChildren() > 0)
				{
					parentLod->setCenter(parentLod->getBound().center());
					parentLod->setRadius(radius);
				}
			}
		} // buildOpenSceneGraphLOD

		void buildOpenSceneGraphPagedLOD(OctreeTraverseState state,
										 osg::Node* parentNode,
										 double samplePrecent,
										 OctreeNodeSamples& samples)
		{
			throw std::runtime_error("Not Implement Yet");
		} // buildOpenSceneGraphPagedLOD


	}
}