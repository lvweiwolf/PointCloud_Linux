//stdafx.h
#include "powerlineCurve.h"

#include <osg/Geode>
#include <osg/MatrixTransform>

#include "../algorithm/nonelinear_least_squares.h"
#include "../algorithm/geometry2d_op.h"

#include "../core/private/cloudProcess.h"

#include "../plot/plotHandle.h"
#include "../plot/geomCreator.h"
#include "../utils/stringutil.h"

namespace d3s {
	namespace pcs {

		double PowerlineCurve::DistanceTo(const osg::Vec3d& point) const
		{
			osg::Vec3d xyz = point - origin;
			osg::Vec3d xyz_prime = rotate.preMult(xyz);

			double xdist = fabs(xyz_prime.x() - xmean);

			// X 轴距离
			double y = xyz_prime.y();
			double z = xyz_prime.z();

			double ydist = 0.0;

			if (y < ymin)
				ydist = ymin - y;
			else if (y > ymax)
				ydist = y - ymax;
			else
				ydist = 0.0;

			double sdist = std::sqrt(xdist * xdist + ydist * ydist);

			// Z 轴距离
			double zdist = fabs((a * y * y + b * y + c) - z);
			double dist = std::sqrt(sdist * sdist + zdist * zdist);

			return dist;
		}


		double PowerlineCurve::NolimitDistanceTo(const osg::Vec3d& point) const
		{
			osg::Vec3d xyz = point - origin;
			osg::Vec3d xyz_prime = rotate.preMult(xyz);

			double xdist = fabs(xyz_prime.x() - xmean);

			// X 轴距离
			double y = xyz_prime.y();
			double z = xyz_prime.z();

			double ydist = 0.0;
			double sdist = std::sqrt(xdist * xdist + ydist * ydist);

			// Z 轴距离
			double zdist = fabs((a * y * y + b * y + c) - z);
			double dist = std::sqrt(sdist * sdist + zdist * zdist);

			return dist;
		}

		osg::Vec3d PowerlineCurve::GetMin() const 
		{ 
			double z = (a * ymin * ymin + b * ymin + c); 
			
			osg::Vec3d local(xmean, ymin, z);
			osg::Vec3d world = osg::Matrix::inverse(rotate).preMult(local);

			return world;
		}

		osg::Vec3d PowerlineCurve::GetMax() const 
		{
			double z = (a * ymax * ymax + b * ymax + c);

			osg::Vec3d local(xmean, ymax, z);
			osg::Vec3d world = osg::Matrix::inverse(rotate).preMult(local);

			return world;
		}

		// 全局函数
		//////////////////////////////////////////////////////////////////////////
		PowerlineCurvePtr PowerlineCurveFitting(PointCloudViewPtr input,
												const std::vector<int>& cluster,
												double expandLength,
												osg::Vec4 color,
												bool debug)
		{
			if (cluster.size() < 3)
				return nullptr;

			osg::Vec3d major, middle, minor;
			std::vector<osg::Vec3d> pts;

			for (int i = 0; i < (int)cluster.size(); ++i)
			{
				const auto& p = input->points[cluster[i]];
				osg::Vec3d v = osg::Vec3d(p.x, p.y, p.z);
				v.z() = 0.0;

				pts.push_back(v);
			}

			getEigenVectors(pts, major, middle, minor);
			osg::Vec3d curveVec = major;
			curveVec.normalize();

			const auto& o = input->points[cluster[0]]; // 第一个点为起点
			osg::Vec3d origin(o.x, o.y, o.z);

			// 导线方向对齐到 Y 轴
			osg::Matrix rotate = osg::Matrix::rotate(curveVec, osg::Vec3d(0.0, 1.0, 0.0));

			double x_mean = 0.0;
			std::vector<double> data;
			data.resize(cluster.size() * 2);

			for (int i = 0; i < (int)cluster.size(); ++i)
			{
				const auto& p = input->points[cluster[i]];
				osg::Vec3d v(p.x, p.y, p.z);
				osg::Vec3d xyz = v - origin;

				osg::Vec3d xyz_prime = rotate.preMult(xyz);

				x_mean += xyz_prime.x();
				data[i * 2] = xyz_prime.y();
				data[i * 2 + 1] = xyz_prime.z();
			}

			x_mean /= (double)cluster.size();

			double a = 0.0;
			double b = 0.0;
			double c = 1.0;

			curveFitting(&data[0], data.size(), a, b, c);

			// 点在Y轴上的最大/最小值
			double ymin = DBL_MAX;
			double ymax = -DBL_MAX;

			for (int i = 0; i < (int)data.size(); i += 2)
			{
				ymin = std::min(ymin, data[i]);
				ymax = std::max(ymax, data[i]);
			}

			// Y轴前后扩充一个单元的长度
			ymin -= expandLength;
			ymax += expandLength;

			// 创建导线曲线
			PowerlineCurvePtr wire =
				std::make_shared<PowerlineCurve>(rotate, origin, x_mean, ymin, ymax, a, b, c);

			if (debug)
			{
				std::vector<osg::Vec3d> verts;
				osg::Matrix rotateInverse = osg::Matrix::inverse(rotate);

				// 按 0.5m 间隔生成点
				for (double y = ymin; y < ymax; y += 0.5)
				{
					double z = a * y * y + b * y + c;
					verts.push_back(osg::Vec3d(x_mean, y, z));
				}

				// 转换到线路坐标空间
				for (int i = 0; i < (int)verts.size(); ++i)
				{
					auto& xyz = verts[i];
					xyz = rotateInverse.preMult(xyz);
					xyz += origin;
				}

				static int iCurve = 0;
				RenderPowerlineCurve(StringPrintf("curve%d", iCurve++), verts, input, color);
			}

			return wire;
		}

		PowerlineCurvePtr PowerlineCurveFitting(PointCloudViewPtr input,
												const std::vector<int>& cluster,
												const osg::Vec3d& pos,
												const osg::Vec3d& dir,
												double expandLength,
												osg::Vec4 color,
												bool debug)
		{
			if (cluster.size() < 3)
				return nullptr;

			osg::Vec3d major, middle, minor;
			std::vector<osg::Vec3d> pts;

			for (int i = 0; i < (int)cluster.size(); ++i)
			{
				const auto& p = input->points[cluster[i]];
				osg::Vec3d v = osg::Vec3d(p.x, p.y, p.z);
				v.z() = 0.0;

				pts.push_back(v);
			}

			getEigenVectors(pts, major, middle, minor);
			osg::Vec3d curveVec = major;
			curveVec.normalize();

			const auto& o = input->points[cluster[0]]; // 第一个点为起点
			osg::Vec3d origin(o.x, o.y, o.z);

			// 导线方向对齐到 Y 轴
			osg::Matrix rotate = osg::Matrix::rotate(curveVec, osg::Vec3d(0.0, 1.0, 0.0));

			double x_mean = 0.0;
			std::vector<double> data;
			data.resize(cluster.size() * 2);

			for (int i = 0; i < (int)cluster.size(); ++i)
			{
				const auto& p = input->points[cluster[i]];
				osg::Vec3d v(p.x, p.y, p.z);
				osg::Vec3d xyz = v - origin;

				osg::Vec3d xyz_prime = rotate.preMult(xyz);

				x_mean += xyz_prime.x();
				data[i * 2] = xyz_prime.y();
				data[i * 2 + 1] = xyz_prime.z();
			}

			x_mean /= (double)cluster.size();

			double a = 0.0;
			double b = 0.0;
			double c = 1.0;

			curveFitting(&data[0], data.size(), a, b, c);

			// 点在Y轴上的最大/最小值
			double ymin = DBL_MAX;
			double ymax = -DBL_MAX;

			for (int i = 0; i < (int)data.size(); i += 2)
			{
				ymin = std::min(ymin, data[i]);
				ymax = std::max(ymax, data[i]);
			}

			// 计算边界交点
			{
				osg::Vec2d A(x_mean, ymin);
				osg::Vec2d B(x_mean, ymax);

				osg::Vec3d vec = dir;
				vec.z() = 0.0;
				vec.normalize();

				osg::Vec3d pos_xyz = pos - origin;
				osg::Vec3d pos_prime = rotate.preMult(pos_xyz);
				osg::Vec3d vec_prime = rotate.preMult(vec);

				osg::Vec2d C(pos_prime.x(), pos_prime.y());
				osg::Vec2d D = C + osg::Vec2d(vec_prime.x(), vec_prime.y());

				double x = x_mean;
				double y = ymax;

				if (lineIntersection(A.x(), A.y(), B.x(), B.y(), C.x(), C.y(), D.x(), D.y(), x, y))
				{
					if (y < ymin)
						ymin = std::max(ymin - expandLength, y);

					if (y > ymax)
						ymax = std::min(ymax + expandLength, y);
				}
			}

			// 创建导线曲线
			PowerlineCurvePtr wire =
				std::make_shared<PowerlineCurve>(rotate, origin, x_mean, ymin, ymax, a, b, c);

			if (debug)
			{
				std::vector<osg::Vec3d> verts;
				osg::Matrix rotateInverse = osg::Matrix::inverse(rotate);

				// 按 0.5m 间隔生成点
				for (double y = ymin; y < ymax; y += 0.5)
				{
					double z = a * y * y + b * y + c;
					verts.push_back(osg::Vec3d(x_mean, y, z));
				}

				// 转换到线路坐标空间
				for (int i = 0; i < (int)verts.size(); ++i)
				{
					auto& xyz = verts[i];
					xyz = rotateInverse.preMult(xyz);
					xyz += origin;
				}

				static int iCurve = 0;
				RenderPowerlineCurve(StringPrintf("curve%d", iCurve++), verts, input, color);
			}

			return wire;
		}
	}
}
