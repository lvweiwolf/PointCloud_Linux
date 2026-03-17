#include <src/service/Recognition.h>
#include <src/utils/logging.h>
#include <src/utils/misc.h>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <ncnn/gpu.h>

#include <algorithm>

using namespace d3s::pcs;
const double gl_dStepLenth = 100.0;	   // tif切割比例
const size_t gl_nPolygonMinPtSize = 3; // 多边形最少点个数
typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point;
typedef boost::geometry::model::polygon<point> polygon;


CCropRecognition::CCropRecognition() { _pRegionPts = nullptr; }

bool CCropRecognition::Recognition(IOptions* pOption)
{
	// 准备基本参数
	if (!InitParam(pOption))
		return false;

	// 切片识别
	int nColTiffCnt = (_pDs->xmax - _pDs->xmin) / gl_dStepLenth + 1;
	int nRowTiffCnt = (_pDs->ymax - _pDs->ymin) / gl_dStepLenth + 1;
	int nBoxCount = 1;
	for (int i = 0; i < nRowTiffCnt; ++i)
	{
		double dTempYmin = gl_dStepLenth * i + _pDs->ymin;
		double dTempYmax = gl_dStepLenth + dTempYmin;
		if (i == nRowTiffCnt - 1)
			dTempYmax = _pDs->ymax;

		for (int j = 0; j < nColTiffCnt; ++j)
		{
			double dTempXmin = gl_dStepLenth * j + _pDs->xmin;
			double dTempXmax = gl_dStepLenth + dTempXmin;
			if (j == nColTiffCnt - 1)
				dTempXmax = _pDs->xmax;

			BoundingBox2D regionBox(dTempXmin, dTempYmin, dTempXmax, dTempYmax);
			RecognitionByRegion(regionBox, nBoxCount);
		}
	}

	double dSrcX = fabs(_pDs->geotransform[1]);
	double dSrcY = fabs(_pDs->geotransform[5]);
	double dMaxSrc = std::max(dSrcX, dSrcY);
	// 简化多边形（抗锯齿）
	SimplifyPolygon(*_pRegionPts, dMaxSrc);
	// 合并区域（使用地理坐标与像素比例作为容差，补偿切片造成的区域误差）
	MergeIntersectPolygon(*_pRegionPts, dMaxSrc);

	closeDOMDataset(_pDs);
	return true;
}

bool CCropRecognition::InitParam(IOptions* pOption)
{
	if (NULL == pOption)
		return false;
	int gpu_count = ncnn::get_gpu_count();
	if (gpu_count <= 0)
	{
		PCS_ERROR("[unet_inference_ncnn] 推理失败，未能检测到GPU设备.");
		return false;
	}
	if (nullptr == pOption->GetData("regionPts"))
		return false;
	_pRegionPts = (std::vector<std::vector<osg::Vec3d>>*)pOption->GetData("regionPts");

	std::string dom_filename = pOption->GetStr("dom_filepath");
	if (dom_filename.empty() || !ExistsFile(dom_filename))
		return false;
	_pDs = createDOMDataset(dom_filename);
	if (nullptr == _pDs)
		return false;
	std::string ncnn_param_path = pOption->GetStr("ncnn_param");
	std::string ncnn_model_path = pOption->GetStr("ncnn_model");

	_outFilePath = GetParentDir(dom_filename) + "\\Output";
	_ncnn_model.opt.use_vulkan_compute = 1;
	_ncnn_model.set_vulkan_device(gpu_count - 1);
	_ncnn_model.load_param(ncnn_param_path.c_str());
	_ncnn_model.load_model(ncnn_model_path.c_str());

	return true;
}

void CCropRecognition::RecognitionByRegion(const d3s::pcs::BoundingBox2D& regionBox, int& nBoxCount)
{
	cv::Mat img, score;
	readDOMImage_deprecated(_pDs, regionBox, img);
	if (img.empty())
		return;
	cv::Size img_size(img.cols, img.rows);

	// 预测数据及处理
	pp_stdnet2_inference_ncnn(_ncnn_model, 512, img, score);
	cv::Mat mask(score.rows, score.cols, CV_8UC1, cv::Scalar(0));
	for (int r = 0; r < score.rows; ++r)
	{
		for (int c = 0; c < score.cols; ++c)
		{
			if (score.at<uint8_t>(r, c) > 0)
				mask.at<uint8_t>(r, c) = 255;
		}
	}

	// 调整大小到原始尺寸，将识别区域还原原本颜色
	cv::resize(mask, mask, img_size, cv::INTER_NEAREST);

	// 轮廓提取(轮廓地区参考代码)
	std::vector<std::vector<cv::Point>> regionPts;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(mask, regionPts, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	// 像素坐标转真实坐标
	std::vector<std::vector<osg::Vec3d>> realPts = Pixel2Real(regionBox, regionPts, img_size);
	_pRegionPts->insert(_pRegionPts->end(), realPts.begin(), realPts.end());

	//// 测试代码
	// if (!regionPts.empty())
	//{
	//	// 写入RGB图像
	//	std::string directory = _outFilePath + StringPrintf(R"(%d\)", nBoxCount);
	//	CreateDirIfNotExists(directory);
	//	cv::imwrite(directory + "实际图片.png", img);
	//	// 写入区域可视识别图像（验证是否存在识别物种）
	//	std::string outMaskFile = _outFilePath + StringPrintf(R"(%d\)", nBoxCount)
	//		+ "识别灰度图" + "testout.png";
	//	cv::imwrite(outMaskFile, mask);
	//	osg::ref_ptr<osg::Image> image = osgDB::readImageFile(directory + "实际图片.png");
	//	// 区域提取图像
	//	for (size_t i = 0; i < realPts.size(); ++i)
	//	{
	//		d3s::pcs::BoundingBox2D box;
	//		for (const auto& iterJ : realPts[i])
	//			box.expandBy(osg::Vec2d(iterJ.x(), iterJ.y()));
	//		if (box.valid())
	//		{
	//			cv::Mat img;
	//			readDOMImage(_pDs, box, img);
	//			std::string outMaskFile = _outFilePath + StringPrintf(R"(%d\)", nBoxCount)
	//				+ StringPrintf(R"(识别区域图片%d.png)", i);
	//			cv::imwrite(outMaskFile, img);
	//		}
	//	}
	//	// 区域描边图像
	//	if (nullptr != image)
	//	{
	//		cv::Point* pPrePt = nullptr;
	//		for (auto& iterI : regionPts)
	//		{
	//			for (auto& iterJ : iterI)
	//			{
	//				if (nullptr != pPrePt && (pPrePt->x != iterJ.x || pPrePt->y != iterJ.y))
	//				{
	//					int x_min = min(pPrePt->x, iterJ.x);
	//					int x_max = max(pPrePt->x, iterJ.x);
	//					int y_min = min(pPrePt->y, iterJ.y);
	//					int y_max = max(pPrePt->y, iterJ.y);

	//					std::vector<osg::Vec2i> pts;
	//					if (pPrePt->x == iterJ.x)
	//					{
	//						for (int dy = y_min; dy < y_max; ++dy)
	//						{
	//							pts.push_back(osg::Vec2i(iterJ.x, dy));
	//						}
	//					}
	//					else
	//					{
	//						double k = (pPrePt->y - iterJ.y) / (pPrePt->x - iterJ.x);
	//						double b = pPrePt->y - k * pPrePt->x;
	//						for (int dx = x_min; dx < x_max; ++dx)
	//						{
	//							double dy = k * dx + b;
	//							if (dy < 0)
	//								dy = 0;
	//							else if (dy >= image->t())
	//								dy = image->t() - 1;

	//							// image坐标原点在左下角
	//							pts.push_back(osg::Vec2i(dx, dy));
	//						}
	//					}
	//					for (auto iter : pts)
	//					{
	//						int nx = iter.x();
	//						// image坐标原点在左下角
	//						int ny = image->t() - iter.y();
	//						if (nx < 0)
	//							nx = 0;
	//						else if (nx >= image->s())
	//							nx = image->s() - 1;
	//						if (ny < 0)
	//							ny = 0;
	//						else if (ny >= image->t())
	//							ny = image->t() - 1;
	//						image->setColor(osg::Vec4(1, 0, 0, 1), nx, ny);
	//					}
	//				}
	//				pPrePt = &iterJ;
	//			}
	//			pPrePt = nullptr;
	//		}
	//		osgDB::writeImageFile(*image, directory + "区域描边图片.png");
	//	}
	//}
	++nBoxCount;
}

std::vector<cv::Point> random_sampling(const cv::Point pPrePt, const cv::Point& iterJ)
{
	std::vector<cv::Point> pts;
	if (fabs(pPrePt.x - iterJ.x) < 10e-5 || fabs(pPrePt.y - iterJ.y) < 10e-5)
		return pts;
	double k = (pPrePt.y - iterJ.y) / (pPrePt.x - iterJ.x);
	double b = pPrePt.y - k * pPrePt.x;
	double x_min = std::min(pPrePt.x, iterJ.x);
	double x_max = std::max(pPrePt.x, iterJ.x);
	double y_min = std::min(pPrePt.y, iterJ.y);
	double y_max = std::max(pPrePt.y, iterJ.y);
	for (double dx = x_min; dx < x_max; ++dx)
	{
		double dy = k * dx + b;
		pts.push_back(cv::Point(dx, dy));
	}
	return pts;
}

std::vector<std::vector<osg::Vec3d>> CCropRecognition::Pixel2Real(
	const d3s::pcs::BoundingBox2D& regionBox,
	const std::vector<std::vector<cv::Point>>& regionPts,
	const cv::Size& img_size)
{
	std::vector<std::vector<osg::Vec3d>> regionPt;
	if (!regionBox.valid() || regionPts.empty() || img_size.empty() || nullptr == _pRegionPts)
		return regionPt;

	double dRealW = regionBox.xMax() - regionBox.xMin();
	double dRealH = regionBox.yMax() - regionBox.yMin();
	double dPixelW = img_size.width;
	double dPixelH = img_size.height;
	double dScrW = dRealW / dPixelW;
	double dScrH = dRealH / dPixelH;

	for (size_t i = 0; i < regionPts.size(); ++i)
	{
		std::vector<osg::Vec3d> pts;
		for (auto const& iterJ : regionPts[i])
		{
			double dX = (double)iterJ.x * dScrW + regionBox.xMin();
			double dY = (double)(dPixelH - iterJ.y) * dScrH + regionBox.yMin();
			pts.push_back(osg::Vec3d(dX, dY, 0));
		}
		regionPt.push_back(pts);
	}
	return regionPt;
}

std::vector<osg::Vec3d> CCropRecognition::MergePolygons(const std::vector<osg::Vec3d>& polygon1,
														const std::vector<osg::Vec3d>& polygon2)
{
	std::vector<osg::Vec3d> result;
	if (polygon1.empty() || polygon2.empty())
		return result;

	std::vector<point> boostPolygonPts1;
	std::vector<point> boostPolygonPts2;
	for (auto iter : polygon1)
		boostPolygonPts1.emplace_back(point(iter.x(), iter.y()));
	for (auto iter : polygon2)
		boostPolygonPts2.emplace_back(point(iter.x(), iter.y()));

	// boost库合并相交多边形
	polygon boostPolygon1, boostPolygon2;
	boost::geometry::append(boostPolygon1.outer(), boostPolygonPts1);
	boost::geometry::append(boostPolygon2.outer(), boostPolygonPts2);
	boost::geometry::correct(boostPolygon1);
	boost::geometry::correct(boostPolygon2);
	std::vector<polygon> polys;
	boost::geometry::union_(boostPolygon1, boostPolygon2, polys);
	if (!polys.empty())
	{
		const polygon& p = polys.front();
		for (auto it = p.outer().begin(); it != p.outer().end(); ++it)
			result.push_back(osg::Vec3d(it->get<0>(), it->get<1>(), 0));
	}
	return result;
}

bool CCropRecognition::IntersectsPolygon(const std::vector<osg::Vec3d>& polygon1,
										 const std::vector<osg::Vec3d>& polygon2)
{
	if (polygon1.empty() || polygon2.empty())
		return false;

	std::vector<point> boostPolygonPts1;
	std::vector<point> boostPolygonPts2;
	for (auto iter : polygon1)
		boostPolygonPts1.emplace_back(point(iter.x(), iter.y()));
	for (auto iter : polygon2)
		boostPolygonPts2.emplace_back(point(iter.x(), iter.y()));

	// boost库多边形求交
	polygon boostPolygon1, boostPolygon2;
	boost::geometry::append(boostPolygon1.outer(), boostPolygonPts1);
	boost::geometry::append(boostPolygon2.outer(), boostPolygonPts2);
	boost::geometry::correct(boostPolygon1);
	boost::geometry::correct(boostPolygon2);
	return boost::geometry::intersects(boostPolygon1, boostPolygon2);
}

void CCropRecognition::MergeIntersectPolygon(std::vector<std::vector<osg::Vec3d>>& polygons,
											 double dEpsilon)
{
	// 多边形个数小于2无意义
	if (polygons.size() < 2)
		return;

	// 将多边形放大
	for (auto& iter : polygons)
	{
		osg::BoundingBox box;
		for (const auto& pt : iter)
			box.expandBy(pt);
		for (auto& pt : iter)
		{
			auto dir = pt - box.center();
			double dLength = dir.length() + dEpsilon;
			dir.normalize();
			pt = dir * dLength + box.center();
		}
		// 闭合
		if (!iter.empty())
			iter.push_back(iter.front());
	}

	// 判断相交，相交则合并
	for (size_t i = 0; i < polygons.size();)
	{
		bool bIntersect = false;
		for (size_t j = i + 1; j < polygons.size(); ++j)
		{
			if (!IntersectsPolygon(polygons[i], polygons[j]))
				continue;

			polygons[i] = MergePolygons(polygons[i], polygons[j]);
			bIntersect = true;
			// 已合并，删除当前多边形
			polygons.erase(polygons.begin() + j);
			break;
		}

		// 如果存在相交多边形，则当前多边形继续冒泡对比
		if (!bIntersect)
			++i;
	}

	// 还原被放大后多边形
	for (size_t i = 0; i < polygons.size(); ++i)
	{
		osg::BoundingBox box;
		for (const auto& pt : polygons[i])
			box.expandBy(pt);
		for (auto& pt : polygons[i])
		{
			auto dir = pt - box.center();
			double dLength = dir.length() - dEpsilon;
			dir.normalize();
			pt = dir * dLength + box.center();
		}
	}
}

void CCropRecognition::SimplifyPolygon(std::vector<std::vector<osg::Vec3d>>& polygons,
									   double dEpsilon)
{
	for (auto& polygon : polygons)
	{
		auto curPolygon = polygon;
		for (size_t i = 0; i < curPolygon.size(); ++i)
		{
			for (size_t j = i + 1; j < curPolygon.size();)
			{
				if ((curPolygon[i] - curPolygon[j]).length() < dEpsilon)
					curPolygon.erase(curPolygon.begin() + j);
				else
					break;
			}
		}
		if (curPolygon.size() >= gl_nPolygonMinPtSize)
			polygon = curPolygon;
	}
}
