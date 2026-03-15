//////////////////////////////////////////////////////////////////////
// 文件名称：individualTreeSegment.h
// 功能描述：单木分割
// 创建标识：吕伟	2022/5/23
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef TREE_SEGMENTATION_H_
#define TREE_SEGMENTATION_H_

#include <src/utils/threading.h>
#include <src/core/pointTypes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>

namespace cv {
	class Mat;
}

namespace d3s {
	namespace pcs {


		struct TreeSegmentationOptions
		{
			bool green_color_style; // 绿色系风格
			int min_pts;			// 单木分割聚类的最少点数
			double min_height;		// 最小树高
			double max_height;		// 最大树高
			double min_radius;		// 最小树间距
			double max_radius;		// 最大树间距
			double max_crown;		// 最大冠幅
			double th_tree;			// 树高阈值
			double th_seed;			// 种子阈值
			double th_crown;		// 树冠阈值
			double resolution;		// 表面网格采样分辨率

			// Li2012
			double dt1;
			double dt2;
			double Zu;
			double R;
			double radius;

			// 打印参数
			void Print();
		};

		struct TreeMetrics
		{
			int id;				// 树ID
			double x;			// x 坐标
			double y;			// y 坐标
			double z;			// z 坐标
			double height;		// 树高
			double crownSize;	// 冠幅直径
			double crownArea;	// 冠幅面积
			double crownVolume; // 冠幅体积
		};

		// 单木分割算法 Dalponte2016
		//////////////////////////////////////////////////////////////////////////
		class Dalponte2016 : public Thread
		{
		public:
			Dalponte2016(const TreeSegmentationOptions& options,
						 PointCloudViewPtr input,
						 const std::vector<int>& indices);

			virtual ~Dalponte2016();

			std::vector<TreeMetrics> GetTreeMetrics() const;

		private:
			virtual void Run() override;

			/**
			 *  @brief    计算几步最高点，作为单木分割的种子
			 *
			 *  @param    const cv::Mat & mat						CHM树冠模型
			 *  @param    cv::Mat & treetops						树顶种子
			 *
			 *  @return   void
			 */
			void ComputeLocalMaxima(const cv::Mat& mat,
								 cv::Mat& treetops,
								 std::function<double(double)> dynamic_radius);


			/**
			 *  @brief    分割树木算法
			 *
			 *  @param    const cv::Mat & chm						CHM冠层墨香
			 *  @param    const cv::Mat & treetops					树顶种子点
			 *  @param    cv::Mat & result							树木分割后的label分布
			 *  @param    double th_seed							种子点生长阈值1，如果某个区域的高度大于树高乘以该值，
			 *														则会将该像素添到该区域。它应该介于0和1之间。默认值为0.45。
			 *
			 *  @param    double th_crown							种子点生长阈值2，如果某个像素的高度大于该区域当前的
			 *														平均高度乘以该值，则该像素将添加到该区域。它应该介于0和1之间。
			 *														默认值为0.55。
			 * 
			 *  @param    double th_tree							低于该阈值的像素不会被分类为树
			 *  @param    double distance
			 检测到的树的树冠直径的最大值（以像素为单位）。默认值为10。
			 *
			 *  @return   void
			 */
			void SegmentTree(const cv::Mat& chm,
							 const cv::Mat& treetops,
							 cv::Mat& result,
							 double th_seed = 0.45,
							 double th_crown = 0.55,
							 double th_tree = 2,
							 double distance = 10);


			/**
			 *  @brief    统计树木冠幅属性
			 *
			 *  @param    const std::map<int, std::vector<int>> & treeIndicesMap
			 *
			 *  @return   void
			 */
			void ComputeTreeMetrics(const std::map<int, std::vector<int>>& treeIndicesMap);

			/**
			 *  @brief    创建2D点云
			 *
			 *  @param    PointCloudViewPtr input					3D点云数据
			 *  @param    const std::vector<int> & indices			点云索引
			 *
			 *  @return   pcl::PointCloud<pcl::PointXY>::Ptr
			 */
			pcl::PointCloud<pcl::PointXY>::Ptr CreateCloud2D(PointCloudViewPtr input,
															 const std::vector<int>& indices);


			TreeSegmentationOptions _options; // 单木分割参数
			PointCloudViewPtr _input;		  // 输入点云数据

			std::vector<int> _indices;			   // 待处理点云索引
			std::vector<TreeMetrics> _treeMetrics; // 单木分割结果信息
		};


		// 单木分割算法 Li2012
		//////////////////////////////////////////////////////////////////////////
		class Li2012 : public Thread
		{
		public:
			Li2012(const TreeSegmentationOptions& options,
				   PointCloudViewPtr input,
				   const std::vector<int>& indices);

			virtual ~Li2012();

			std::vector<TreeMetrics> GetTreeMetrics() const;

		private:
			virtual void Run() override;
		
			/**
			 *  @brief    计算局部最大值点
			 *
			 *  @param    const std::vector<double> & ws		窗口大小列表
			 *  @param    double min_height						最小高度
			 *  @param    bool circular							是否为圆形窗口
			 *
			 *  @return   void
			 */
			std::vector<bool> ComputeLocalMaxima(const std::vector<double>& ws,
												 double min_height,
												 bool circular);

			/**
			 *  @brief    分割树木
			 *
			 *  @param    double dt1		阈值1, 参见Li等人(2012)的参考文献第79页. 默认值为 1.5
			 *  @param    double dt2		阈值2, 参见Li等人(2012)的参考文献第79页. 默认值为 2
			 *  @param    double Zu			如果点高程大于Zu，则使用 dt2, 否则使用 dt1, 见Li等人(2012)第79页. 
											默认值为 15。
			 *  @param    double R			搜索半径, 见Li等人(2012)第79页。默认值为2. 如果 R=0, 则所有点都自
											动被视为局部最大值, 并跳过搜索步骤(更快).
			 *  @param    double th_tree	检测到的树的最小高度, 默认值为2.
			 *  @param    double radius		冠层的最大半径吗, 任何大于冠层的值都是好的, 因为该参数不会影响结果, 
											然而，它通过限制要执行的比较次数, 极大地影响了计算速度. 值越低, 方
											法越快, 默认值为10.
			 *
			 *  @return   void
			 */
			void SegmentTree(std::vector<int>& idtree,
							 double dt1 = 1.5,
							 double dt2 = 2.0,
							 double Zu = 15.0,
							 double R = 2.0,
							 double th_tree = 2.0,
							 double radius = 10.0);


			
			std::vector<double> SquareDistance(const std::vector<int>& indices,
											   int index,
											   const PointPCLH& dummy);


			TreeSegmentationOptions _options; // 单木分割参数
			PointCloudViewPtr _input;		  // 输入点云数据

			std::vector<int> _indices;			   // 待处理点云索引
			std::vector<TreeMetrics> _treeMetrics; // 单木分割结果信息
		};


		void WriteTreeMetrics(const std::vector<TreeMetrics> & treeMetrics, const std::string& path);
	}
}

#endif // TREE_SEGMENTATION_H_