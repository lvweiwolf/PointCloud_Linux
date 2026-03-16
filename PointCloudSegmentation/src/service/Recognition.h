/*-----------------------------------------------------
* 文件名称：  Recognition.H
* 功能描述：  识别
* 创建标识：  liangchao  2023/4/25 10:12
*
* 修改标识：
* 修改描述：
*
* 修改标识：
* 修改描述：
-----------------------------------------------------*/
#ifndef RECOGNITION_H_
#define RECOGNITION_H_

#include <include/IImageRecognition.h>
#include <include/ICloudSegmentation.h>

#include <src/core/deeplearning/inference.h>
#include <src/core/private/gdalProcess.h>

#include <ncnn/net.h>

namespace d3s {
	namespace pcs {

		// 林木、作物识别
		class CCropRecognition : public IImageRecognition
		{
		public:
			CCropRecognition();
			virtual ~CCropRecognition() {}

		public:
			/**
			 *  函数介绍： 识别
			 *
			 *  输入参数： IOptions * pOption		识别参数
			 *  返回值：   void
			 */
			virtual bool Recognition(IOptions* pOption) override;

		protected:
			/**
			 * 初始化参数
			 * @param [in] pOption 参数集合
			 * @return
			 */
			bool InitParam(IOptions* pOption);

			/**
			 * 识别指定区域
			 * @param [in] regionBox 区域
			 * @param [in] nIndex 区域计数
			 * @return
			 */
			void RecognitionByRegion(const d3s::pcs::BoundingBox2D& regionBox, int& nBoxCount);

			/**
			 * 像素转真实坐标
			 * @param [in] regionBox 真实坐标包围盒
			 * @param [in] regionPts 像素转坐标
			 * @param [in] img_size 像素分辨率
			 * @return
			 */
			std::vector<std::vector<osg::Vec3d>> Pixel2Real(
				const d3s::pcs::BoundingBox2D& regionBox,
				const std::vector<std::vector<cv::Point>>& regionPts,
				const cv::Size& img_size);

			/**
			 * 简化多边形（抗锯齿）
			 * @param [in/out] polygons	多边形
			 * @param [in] dEpsilon	容差
			 * @return
			 */
			void SimplifyPolygon(std::vector<std::vector<osg::Vec3d>>& polygons, double dEpsilon);

			/**
			 * 合并相交的多边形
			 * @param [in/out] polygons	多边形
			 * @param [in] dEpsilon	容差
			 * @return
			 */
			void MergeIntersectPolygon(std::vector<std::vector<osg::Vec3d>>& polygons,
									   double dEpsilon);

			/**
			 * 多边形求交判断
			 * @param [in] polygon1	多边形
			 * @param [in] polygon2	多边形
			 * @return
			 */
			bool IntersectsPolygon(const std::vector<osg::Vec3d>& polygon1,
								   const std::vector<osg::Vec3d>& polygon2);

			/**
			 * 合并多边形
			 * @param [in] polygon1	多边形
			 * @param [in] polygon2	多边形
			 * @return
			 */
			std::vector<osg::Vec3d> MergePolygons(const std::vector<osg::Vec3d>& polygon1,
												  const std::vector<osg::Vec3d>& polygon2);

		private:
			std::shared_ptr<DOMDataset> _pDs;				   // 影像对象
			std::string _outFilePath;						   // 输出文件路径
			ncnn::Net _ncnn_model;							   // 训练模型
			std::vector<std::vector<osg::Vec3d>>* _pRegionPts; // 识别到的区域点击
		};

	}
}

#endif // !RECOGNITION_H_
