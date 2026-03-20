/////////////////////////////////////////////////////////////////////
// 文件名称：CloudSegmentation.h
// 功能描述：点云分割基类
// 创建标识：吕伟	2022/6/26
// 修改标识：
// 修改描述：
// 文件版权：江博微西新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef CLOUD_SEGMENTATION_H_
#define CLOUD_SEGMENTATION_H_

#include <include/ICloudSegmentation.h>

#include <src/utils/logging.h>

namespace d3s {
	namespace pcs {

		class CCloudProcess
		{
		public:
			CCloudProcess();

			virtual ~CCloudProcess();

			/**
			 *  @brief   设置点云数据
			 *
			 *  @prarm	 IPointCloudPtr cloud						处理的点云数据
			 *
			 *  @return   bool
			 */
			void SetCloudInput(IPointCloudPtr cloud);


			/**
			 *  @brief    设置需要处理点云的索引
			 *
			 *  @param    const std::vector<int> & indices			实际处理点的索引
			 *
			 *  @return   bool
			 */
			void SetIndices(const std::vector<PointId>& indices);

			/**
			 *  @brief    设置进度处理回调
			 *
			 *  @prarm	 ISegmentProgressPtr progress				进度处理接口
			 *
			 *  @return   void
			 */
			// void SetProgressCallback(IProgressPtr progress);

			/**
			 *  @brief    设置分类任务进度范围
			 *
			 *  @prarm	 double start								进度开始值
			 *  @prarm	 double end									进度结束值
			 *
			 *  @return   void
			 */
			// void SetProgressRange(double start, double end);

		protected:
			/**
			 *  @brief    任务局部进度
			 *
			 *  @prarm	 double precent								局部进度百分比(0, 100)
			 *  @prarm	 const char * pszMessage					任务消息
			 *
			 *  @return   void
			 */
			// void Progress(double precent, const char* pszMessage);

			IPointCloudPtr _cloud;		   // 点云数据对象
			std::vector<PointId> _indices; // 处理的点云索引

		private:
			// IProgressPtr _progressCallback; // 进度处理回调
			double _start; // 进度起始值
			double _end;   // 进度结束值
		};

		class CCloudSegmentation : public ICloudSegmentation, public CCloudProcess
		{
		public:
			CCloudSegmentation() : CCloudProcess() {};

			virtual ~CCloudSegmentation() {};

			virtual void SetCloudInput(IPointCloudPtr cloud)
			{
				CCloudProcess::SetCloudInput(cloud);
			}

			virtual void SetIndices(const std::vector<PointId>& indices)
			{
				CCloudProcess::SetIndices(indices);
			}

			// virtual void SetProgressCallback(IProgressPtr progress = nullptr)
			//{
			//	CCloudProcess::SetProgressCallback(progress);
			// }

			virtual bool Segment(const IOptions* options, std::vector<PointId>& result)
			{
				PCS_ERROR("暂未实现!");
				return false;
			}
		};

		class CCloudDetection : public ICloudDetection, public CCloudProcess
		{
		public:
			CCloudDetection() : CCloudProcess() {};

			virtual ~CCloudDetection() {};

			virtual void SetCloudInput(IPointCloudPtr cloud)
			{
				CCloudProcess::SetCloudInput(cloud);
			}

			virtual void SetIndices(const std::vector<PointId>& indices)
			{
				CCloudProcess::SetIndices(indices);
			}

			virtual bool DetectPositions(const IOptions* options,
										 std::vector<osg::Vec3d>& positions)
			{
				PCS_ERROR("暂未实现!");
				return false;
			}
		};

		class CCloudCurveFitting : public ICloudCurveFitting, public CCloudProcess
		{
		public:
			CCloudCurveFitting() : CCloudProcess() {};

			virtual ~CCloudCurveFitting() {};

			virtual void SetCloudInput(IPointCloudPtr cloud)
			{
				CCloudProcess::SetCloudInput(cloud);
			}

			virtual void SetIndices(const std::vector<PointId>& indices)
			{
				CCloudProcess::SetIndices(indices);
			}

			// virtual void SetProgressCallback(IProgressPtr progress = nullptr)
			//{
			//	CCloudProcess::SetProgressCallback(progress);
			// }

			virtual bool CurveFitting(double expandLength, PARACURVE& params)
			{
				PCS_ERROR("暂未实现!");
				return false;
			}
		};

		class CCloudClustering : public ICloudClustering, public CCloudProcess
		{
		public:
			CCloudClustering() : CCloudProcess() {};

			virtual ~CCloudClustering() {};

			virtual void SetCloudInput(IPointCloudPtr cloud)
			{
				CCloudProcess::SetCloudInput(cloud);
			}

			virtual void SetIndices(const std::vector<PointId>& indices)
			{
				CCloudProcess::SetIndices(indices);
			}

			// virtual void SetProgressCallback(IProgressPtr progress = nullptr)
			//{
			//	CCloudProcess::SetProgressCallback(progress);
			// }

			virtual void EuclideanCluster(double tolerance,
										  int minPts,
										  int maxPts,
										  Clusters& clusters)
			{
				PCS_ERROR("暂未实现!");
			}

			virtual void EuclideanClusterFast(double tolerance,
											  int minPts,
											  int maxPts,
											  Clusters& clusters)
			{
				PCS_ERROR("暂未实现!");
			}
		};

		class CCloudRasterize : public ICloudRasterize, public CCloudProcess
		{
		public:
			CCloudRasterize() : CCloudProcess() {};

			virtual ~CCloudRasterize() {};

			virtual void SetCloudInput(IPointCloudPtr cloud)
			{
				CCloudProcess::SetCloudInput(cloud);
			}

			virtual void SetIndices(const std::vector<PointId>& indices)
			{
				CCloudProcess::SetIndices(indices);
			}

			// virtual void SetProgressCallback(IProgressPtr progress = nullptr)
			//{
			//	CCloudProcess::SetProgressCallback(progress);
			// }

			virtual void WriteTiff(const std::string& filepath,
								   const std::string& srs,
								   double resolution)
			{
				PCS_ERROR("暂未实现!");
			}
		};

		class CCloudReconstruct : public ICloudReconstrct, public CCloudProcess
		{
		public:
			CCloudReconstruct() : CCloudProcess() {};

			virtual ~CCloudReconstruct() {};

			virtual void SetCloudInput(IPointCloudPtr cloud)
			{
				CCloudProcess::SetCloudInput(cloud);
			}

			virtual void SetIndices(const std::vector<PointId>& indices)
			{
				CCloudProcess::SetIndices(indices);
			}

			// virtual void SetProgressCallback(IProgressPtr progress = nullptr)
			//{
			//	CCloudProcess::SetProgressCallback(progress);
			// }

			virtual bool Reconstruct(const IOptions* options)
			{
				PCS_ERROR("暂未实现!");
				return false;
			}
		};
	}
}

typedef d3s::share_ptr<d3s::pcs::CCloudSegmentation> CCloudSegmentationPtr;
typedef d3s::share_ptr<d3s::pcs::CCloudDetection> CCloudDetectionPtr;
typedef d3s::share_ptr<d3s::pcs::CCloudCurveFitting> CCloudCurveFittingPtr;
typedef d3s::share_ptr<d3s::pcs::CCloudClustering> CCloudClusteringPtr;
typedef d3s::share_ptr<d3s::pcs::CCloudRasterize> CCloudRasterizePtr;
typedef d3s::share_ptr<d3s::pcs::CCloudReconstruct> CCloudReconstructPtr;


#endif // CLOUD_SEGMENTATION_H_