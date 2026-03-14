/////////////////////////////////////////////////////////////////////
// 文件名称：rasterProcess.h
// 功能描述：光栅化数据处理接口
// 创建标识：吕伟	2022/6/14
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once

#include "../../algorithm/math.h"
#include "../../algorithm/raster.h"
#include <osg/Vec2d>
#include <osg/Vec2i>
#include <osg/Vec4>
#include <vector>
#include <string>
#include <array>
#include "../pointTypes.h"
#define PI 3.14159265358979323846
namespace d3s {
	namespace pcs {
		
		typedef std::vector<std::array<osg::Vec2d, 2>> LineSegmnets;

		struct ImageProcessParameter
		{
			int morph_fill_size = 3;
			int morph_open_size = 6;

			double hough_rho = 1;
			double hough_theta = PI / 180.0;
			double hough_threshold = 50;
			double hough_min_length = 50;
			double hough_max_gap = 25;
			double line_thickness = 1.0;

			double line_lsd_scale = 0.7; // LSD算法缩放值默认0.7，分辨率很低时适当提高缩放稀疏
		};


		/**
		 *  @brief    地面点优化
		 *
		 *  @param    const PointCloudView<PointPCLH> & pcv				点云数据
		 *  @param    const std::vector<int> & indices					点云索引
		 *  @param    double resolution									转换到图像分辨率
		 *  @param    int morphSize										形态学操作卷积核大小
		 *  @param    std::vector<int> & out							过滤后的点云索引
		 *
		 *  @return   void
		 */
		void pointCloudMorph(const PointCloudView<PointPCLH>& pcv,
							 const std::vector<int>& indices,
							 double resolution,
							 int morphSize,
							 std::vector<int>& out);

		/**
		 *  @brief    创建点云三角网光栅数据
		 *
		 *  @prarm	 const PointCloudView<PointPCLH> & pcv				点云数据
		 *  @prarm	 const std::vector<int> & ground					地面点
		 *  @prarm	 double resolution									栅格图像分辨率
		 *  @prarm	 Rasterd & result									光栅数据
		 *
		 *  @return   void
		 */
		void createCloudRaster(const PointCloudView<PointPCLH>& pcv,
							   const std::vector<int>& ground,
							   double resolution,
							   Rasterd& result);

		void createCloudRaster(const PointCloudView<PointPCLH>::PointCloud& points,
							   const osg::BoundingBox& bbox,
							   double resolution,
							   Rasterd& result);

		/**
		*  @brief    创建道路栅格数据
		*
		*  @prarm	 const PointCloudView<PointPCLH> & pcv				点云数据
		*  @prarm	 const std::vector<std::vector<osg::Vec3d>> & roads	道路矢量数据
		*  @prarm	 Rasterd & result
		*
		*  @return   void
		*/
		void createRoadRaster(const PointCloudView<PointPCLH>& pcv,
							  const std::vector<std::vector<osg::Vec3d>>& roads,
							  Rasterd& result);

		/**
		 *  @brief    按地面点归一化高程
		 *
		 *  @prarm	 const std::string & dtmfile					地形文件 tiff
		 *  @prarm	 PointCloudView<PointPCLH> & pcv				点云数据
		 *
		 *  @return  bool
		 */
		bool normalizeByGround(const std::string& dtmfile, PointCloudView<PointPCLH>& pcv);

		/**
		 *  @brief    按地面点归一化高程
		 *
		 *  @param    const Rasterd & raster						光栅数据
		 *  @param    PointCloudView<PointPCLH> & pcv				点云数据
		 *
		 *  @return   bool
		 */
		bool normalizeByGround(const Rasterd& raster, PointCloudView<PointPCLH>& pcv);

		/**
		 *  @brief    生成二值掩码图像
		 *
		 *  @param    const PointCloudView<PointPCLH> & pcv				点云数据
		 *  @param    const std::vector<int> & indices					遮罩部分点云索引
		 *  @param    const osg::BoundingBox & bbox						局部边界范围
		 *  @param    double resolution									分辨率
		 *  @param    Rasterd & result									遮罩
		 *
		 *  @return   void
		 */
		void createBinaryMask(const PointCloudView<PointPCLH>& pcv,
							  const std::vector<int>& indices,
							  const osg::BoundingBox& bbox,
							  double resolution,
							  Rasterd& result);


		void createHagRaster(const PointCloudView<PointPCLH>& pcv,
							 const std::vector<int>& indices,
							 const osg::BoundingBox& bbox,
							 double resolution,
							 Rasterd& result);

		void createHeightRaster(const PointCloudView<PointPCLH>& pcv,
								const std::vector<int>& indices,
								const osg::BoundingBox& bbox,
								double resolution,
								Rasterd& result);

		/**
		 *  @brief    写入二值图像到本地
		 *
		 *  @param    const Rasterd & binary							二值图像数据
		 *  @param    const std::string & filepath						文件路径
		 *
		 *  @return   void
		 */
		void writeTiff(const Rasterd& binary, const std::string& filepath, std::string srs = "");


		void writeImage(const Rasterd& src, const std::string& filepath, bool binary = false);

		void writeImage(const Rasterd& src,
						const std::string& filepath,
						const std::vector<osg::Vec4>& colormap);

		/**
		 *  @brief	 创建铁塔二值图像掩码
		 *
		 *  @prarm	 const Rasterd & src								铁塔候选点二值图像
		 *  @prarm	 const ImageProcessParameter & p					图像处理参数
		 *  @prarm	 Rasterd & result									铁塔识别区域掩码
		 *  @prarm	 int & x											铁塔中心像素坐标 x
		 *  @prarm	 int & y											铁塔中心像素坐标	y
		 *
		 *  @return   bool
		 */
		bool createPylonMask(const Rasterd& src,
							 const ImageProcessParameter& p,
							 Rasterd& result,
							 int& x,
							 int& y);

		/**
		 *  @brief    创建电力线二值图像有掩码
		 *
		 *  @prarm	 const Rasterd & mask								电力线候选点二值图像
		 *  @prarm	 const ImageProcessParameter & p					图像处理参数
		 *  @prarm	 Rasterd & result									电力线识别区域掩码
		 *
		 *  @return   void
		 */
		void createPowerlineMask(const Rasterd& src,
								 const ImageProcessParameter& p,
								 Rasterd& result);

		/**
		 *  @brief    创建植被掩码
		 *
		 *  @param    const Rasterd & mask								点云候选二值图像
		 *  @param    const ImageProcessParameter & p					图像处理参数
		 *  @param    Rasterd & result									植被掩码
		 *  @param    double areaThreshold								植被面积阈值
		 *
		 *  @return   void
		 */
		void createVegetationMask(const Rasterd& mask,
								  const ImageProcessParameter& p,
								  Rasterd& result,
								  double threshold = 100);

		/**
		 *  @brief    生成塔体掩码
		 *
		 *  @param    const PointCloudView<PointPCLH> & pcv				点云数据
		 *  @param    const std::vector<int> & mask						点云索引
		 *  @param    const osg::BoundingBox & bbox						点云范围边界
		 *  @param    double resolution									分辨率
		 *  @param    double morphRatio									形态处理系数
		 *  @param    int post											后缀
		 *  @param    Rasterd & result									输出掩码
		 *
		 *  @return   void
		 */
		void createBodyMask(const PointCloudView<PointPCLH>& pcv,
							const std::vector<int>& mask,
							const osg::BoundingBox& bbox,
							double resolution,
							double morphRatio,
							int post,
							Rasterd& result);


		/**
		 *  @brief   检测异常值
		 *
		 *  @param    const Rasterd & raster							光栅数据
		 *  @param    Rasterd & binary									二值图像
		 *
		 *  @return   void
		 */
		void detectOutliers(const Rasterd& raster,
							double multi,
							Rasterd& result,
							std::vector<osg::Vec2i>& pixels);



		/**
		 *  @brief    使用Hough变换算法检测直线
		 *
		 *  @param    const Rasterd & mask						图像掩码
		 *  @param    const ImageProcessParameter & p			图像处理参数
		 *  @param    std::vector<std::array<osg::Vec2d, 2>> & lines 检测出的直线
		 *
		 *  @return   void
		 */
		void detectLines(const Rasterd& mask,
						 const ImageProcessParameter& p,
						 std::vector<std::array<osg::Vec2d, 2>>& lines);

		void detectLines(const Rasterd& mask, const ImageProcessParameter& p, Rasterd& result);


		/**
		 *  @brief    使用LSD算法检测直线
		 *
		 *  @param    const Rasterd & mask						图像掩码
		 *  @param    const ImageProcessParameter & p			图像处理参数
		 *  @param    std::vector<std::array<osg::Vec2d, 2>> & lines	检测出的直线
		 *
		 *  @return   void
		 */
		void detectLinesLSD(const Rasterd& mask,
							const ImageProcessParameter& p,
							std::vector<std::array<osg::Vec2d, 2>>& lines);

		/**
		 *  @brief    移除较小长度的直线
		 *
		 *  @param    LineSegmnets & lines	线段数组
		 *  @param    double threshold		长度阈值
		 *
		 *  @return   void
		 */
		void removeSmallLengths(LineSegmnets& lines, double threshold);


		void createLineMask(const Rasterd& mask,
							const ImageProcessParameter& p,
							const std::vector<std::array<osg::Vec2d, 2>>& lines,
							Rasterd& result);

	
		/**
		 *  @brief    检测连通区域
		 *
		 *  @param    const Rasterd & mask						图像掩码
		 *  @param    std::vector<std::vector<osg::Vec2i>> & components		连同区域
		 *
		 *  @return   void
		 */
		void findConnectComponents(const Rasterd& mask,
									 std::vector<std::vector<osg::Vec2i>>& components);

		/**
		 *  @brief    连通区域求交
		 *
		 *  @param    const Rasterd & mask0						掩码0
		 *  @param    Rasterd & mask1							掩码1
		 *  @param    int post									后缀
		 *
		 *  @return   void
		 */
		void intersectConnectComponents(const Rasterd& mask0, Rasterd& mask1, int post);

		/**
		 *  @brief    膨胀算法
		 *
		 *  @param    Rasterd & mask							图像掩码
		 *  @param    double morphRatio							膨胀比例
		 *  @param    int post									后缀
		 *
		 *  @return   void
		 */
		void dilate(Rasterd& mask, double morphRatio, int post);

		/**
		*  @brief    填充凸包
		*
		*  @prarm	 Rasterd & mask								图像掩码
		*  @prarm	 double morphRatio							膨胀比例
		*  @prarm	 int post									后缀
		*
		*  @return   void
		*/
		void convexHull(Rasterd& mask, double morphRatio, int post);


		/**
		 *  @brief    图像缩放
		 *
		 *  @param    Rasterd & mask							图像掩码
		 *  @param    double scale								缩放比例
		 *  @param    int post									后缀
		 *
		 *  @return   void
		 */
		void scale(Rasterd& mask, double scale, int post);


		/**
		 *  @brief    细化算法
		 *
		 *  @param    Rasterd & mask
		 *
		 *  @return   void
		 */
		void thinning(Rasterd& mask);

		/**
		 *  @brief    计算图像轮廓的形状系数
		 *
		 *  @param    const Rasterd & mask						图像掩码
		 *
		 *  @return   double
		 */
		double compuateShapeness(const Rasterd& mask);


		/**
		 *  @brief    创建一组随机颜色
		 *
		 *  @param    int colorSize
		 *
		 *  @return   std::vector<osg::Vec4>
		 */
		std::vector<osg::Vec4> createColors(int colorSize);


		/**
		 *  @brief    创建为彩色颜色
		 *
		 *  @param    int numClasses
		 *
		 *  @return   std::vector<osg::Vec4>
		 */
		std::vector<osg::Vec4> createPseudoColors(int numClasses);


		void gaussianBlur(Rasterd& raster, int kx, int ky, int sigmaX);

		Rasterd computeSlope(const Rasterd& raster);

		Rasterd computeGrad(const Rasterd& raster);

		Rasterd computeLaplacian(const Rasterd& raster);

		void morphology(Rasterd& raster, int morphType, int morphShape, int xsize, int ysize);


		Rasterd filterWithArea(const Rasterd& raster, double areaThr);
	}
}
