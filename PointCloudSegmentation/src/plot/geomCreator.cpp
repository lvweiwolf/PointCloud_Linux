//stdafx.h
#include "geomCreator.h"
#include "../Platform.h"
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/Point>
#include <osg/PolygonMode>
#include <osg/LineWidth>

#include <osgText/Text>
#include "../core/pointTypes.h"	

namespace d3s {
	namespace pcs {

		namespace {
			osg::ref_ptr<osg::StateSet> pointStateSet;

			osg::ref_ptr<osg::StateSet> getOrCreatePointStateSet()
			{
				if (!pointStateSet.valid())
				{
					pointStateSet = new osg::StateSet;

					osg::ref_ptr<osg::Point> point = new osg::Point;
					point->setSize(3);

					pointStateSet->setAttribute(point);
					pointStateSet->setMode(GL_POINT_SMOOTH, osg::StateAttribute::ON);
					pointStateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
				}

				return pointStateSet;
			}
		}

		osg::Geode* CreateAxis(osg::BoundingBox bbox)
		{
			osg::ref_ptr<osg::Geode> pGeode = new osg::Geode();
			// 设置节点掩码，使其不被掩码为 0xfffffff0 的 NodeVisitor捕获
			pGeode->setNodeMask(0x00000001);

			pGeode->setDataVariance(osg::Object::STATIC);

			osg::ref_ptr<osg::Vec4Array> blue = new osg::Vec4Array;
			blue->push_back(osg::Vec4(0.2, 0.2, 1, 1));
			osg::ref_ptr<osg::Vec4Array> green = new osg::Vec4Array;
			green->push_back(osg::Vec4(0.2, 1, 0.2, 1));
			osg::ref_ptr<osg::Vec4Array> red = new osg::Vec4Array;
			red->push_back(osg::Vec4(1, 0.2, 0.2, 1));

			const osg::Vec3& min = bbox._min;
			const osg::Vec3& max = bbox._max;

			osg::ref_ptr<osg::Vec3Array> xAxisVertex = new osg::Vec3Array(2);
			(*xAxisVertex)[0] = min;
			(*xAxisVertex)[1] = osg::Vec3(max.x(), min.y(), min.z());

			osg::ref_ptr<osg::Vec3Array> yAxisVertex = new osg::Vec3Array(2);
			(*yAxisVertex)[0] = min;
			(*yAxisVertex)[1] = osg::Vec3(min.x(), max.y(), min.z());

			osg::ref_ptr<osg::Vec3Array> zAxisVertex = new osg::Vec3Array(2);
			(*zAxisVertex)[0] = min;
			(*zAxisVertex)[1] = osg::Vec3(min.x(), min.y(), max.z());

			osg::ref_ptr<osg::Geometry> pXGeometry = new osg::Geometry;
			pXGeometry->setColorArray(red);
			pXGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
			pXGeometry->setVertexArray(xAxisVertex);
			pXGeometry->addPrimitiveSet(
				new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, xAxisVertex->size()));
			pGeode->addDrawable(pXGeometry);

			osg::ref_ptr<osg::Geometry> pYGeometry = new osg::Geometry;
			pYGeometry->setColorArray(green);
			pYGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
			pYGeometry->setVertexArray(yAxisVertex);
			pYGeometry->addPrimitiveSet(
				new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, xAxisVertex->size()));
			pGeode->addDrawable(pYGeometry);

			{
				osg::ref_ptr<osg::Geometry> pZGeometry = new osg::Geometry;
				pZGeometry->setColorArray(blue);
				pZGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
				pZGeometry->setVertexArray(zAxisVertex);
				pZGeometry->addPrimitiveSet(
					new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, xAxisVertex->size()));
				pGeode->addDrawable(pZGeometry);
			}

			osg::ref_ptr<osgText::Text> textX = new osgText::Text;
			// textX->setUseDisplayList(false);
			textX->setFont("Fonts/msyh.ttc");
			textX->setColor(osg::Vec4(1, 1, 1, 1));
			textX->setCharacterSize(10);
			textX->setPosition(osg::Vec3(max.x(), min.y(), min.z()));
			textX->setText(_T("X"));
			textX->setAutoRotateToScreen(true);
			textX->setFontResolution(256, 256);
			pGeode->addDrawable(textX);

			osg::ref_ptr<osgText::Text> textY = new osgText::Text;
			// textY->setUseDisplayList(false);
			textY->setFont("Fonts/msyh.ttc");
			textY->setColor(osg::Vec4(1, 1, 1, 1));
			textY->setCharacterSize(10);
			textY->setPosition(osg::Vec3(min.x(), max.y(), min.z()));
			textY->setText(_T("Y"));
			textY->setAutoRotateToScreen(true);
			textY->setFontResolution(256, 256);
			pGeode->addDrawable(textY);


			osg::ref_ptr<osgText::Text> textZ = new osgText::Text;
			// textZ->setUseDisplayList(false);
			textZ->setFont("Fonts/msyh.ttc");
			textZ->setColor(osg::Vec4(1, 1, 1, 1));
			textZ->setCharacterSize(10);
			textZ->setPosition(osg::Vec3(min.x(), min.y(), max.z()));
			textZ->setText(_T("Z"));
			textZ->setAutoRotateToScreen(true);
			textZ->setFontResolution(256, 256);
			pGeode->addDrawable(textZ);

			pGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

			osg::ref_ptr<osg::PolygonMode> polyModeObj = new osg::PolygonMode;
			polyModeObj->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);
			pGeode->getOrCreateStateSet()->setAttributeAndModes(polyModeObj,
																osg::StateAttribute::ON |
																	osg::StateAttribute::PROTECTED |
																	osg::StateAttribute::OVERRIDE);

			return pGeode.release();
		}

		// 创建立方体
		osg::Geode* CreateCube(const osg::BoundingBox& bbox,
							   const osg::Vec4& clrFill,
							   const osg::Vec4& clrBorder,
							   const osg::Vec3d& offset)
		{
			osg::ref_ptr<osg::Geode> geode = new osg::Geode();
			// 设置节点掩码，使其不被掩码为 0xfffffff0 的 NodeVisitor捕获
			geode->setNodeMask(0x00000003);

			const auto& min = bbox._min;
			const auto& max = bbox._max;

			auto c = (min + max) / 2.0;
			auto x_size = max.x() - min.x();
			auto y_size = max.y() - min.y();
			auto z_size = max.z() - min.z();

			osg::Vec3 center(c.x(), c.y(), c.z());
			center -= offset;

			{
				// 添加包围框填充
				osg::ref_ptr<osg::ShapeDrawable> boxGeom =
					new osg::ShapeDrawable(new osg::Box(center, x_size, y_size, z_size));

				boxGeom->setUseVertexBufferObjects(true);
				boxGeom->setColor(clrFill);

				geode->addDrawable(boxGeom);
			}

			{
				unsigned int idx = 0;
				std::map<osg::Vec3, unsigned int> indices;
				std::vector<unsigned int> lines;

				// 添加包围框边缘
				osg::ref_ptr<osg::Geometry> borderGeom = new osg::Geometry();
				osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();

				osg::Vec3 p1(center.x() - x_size * 0.5f,
							 center.y() - y_size * 0.5f,
							 center.z() - z_size * 0.5f);
				osg::Vec3 p2(center.x() + x_size * 0.5f,
							 center.y() - y_size * 0.5f,
							 center.z() - z_size * 0.5f);
				osg::Vec3 p3(center.x() + x_size * 0.5f,
							 center.y() + y_size * 0.5f,
							 center.z() - z_size * 0.5f);
				osg::Vec3 p4(center.x() - x_size * 0.5f,
							 center.y() + y_size * 0.5f,
							 center.z() - z_size * 0.5f);
				osg::Vec3 p5(center.x() - x_size * 0.5f,
							 center.y() + y_size * 0.5f,
							 center.z() + z_size * 0.5f);
				osg::Vec3 p6(center.x() + x_size * 0.5f,
							 center.y() + y_size * 0.5f,
							 center.z() + z_size * 0.5f);
				osg::Vec3 p7(center.x() + x_size * 0.5f,
							 center.y() - y_size * 0.5f,
							 center.z() + z_size * 0.5f);
				osg::Vec3 p8(center.x() - x_size * 0.5f,
							 center.y() - y_size * 0.5f,
							 center.z() + z_size * 0.5f);

				vertices->push_back(p1);
				vertices->push_back(p2);
				vertices->push_back(p3);
				vertices->push_back(p4);
				vertices->push_back(p5);
				vertices->push_back(p6);
				vertices->push_back(p7);
				vertices->push_back(p8);

				indices[p1] = idx++;
				indices[p2] = idx++;
				indices[p3] = idx++;
				indices[p4] = idx++;
				indices[p5] = idx++;
				indices[p6] = idx++;
				indices[p7] = idx++;
				indices[p8] = idx++;
				borderGeom->setVertexArray(vertices);

				// 设置颜色
				osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
				colors->push_back(clrBorder);
				borderGeom->setColorArray(colors, osg::Array::BIND_OVERALL);

				// Frame由线段组成
				lines.push_back(indices[p1]);
				lines.push_back(indices[p2]);

				lines.push_back(indices[p2]);
				lines.push_back(indices[p3]);

				lines.push_back(indices[p3]);
				lines.push_back(indices[p4]);

				lines.push_back(indices[p4]);
				lines.push_back(indices[p1]);

				lines.push_back(indices[p5]);
				lines.push_back(indices[p6]);

				lines.push_back(indices[p6]);
				lines.push_back(indices[p7]);

				lines.push_back(indices[p7]);
				lines.push_back(indices[p8]);

				lines.push_back(indices[p8]);
				lines.push_back(indices[p5]);

				lines.push_back(indices[p1]);
				lines.push_back(indices[p8]);

				lines.push_back(indices[p2]);
				lines.push_back(indices[p7]);

				lines.push_back(indices[p3]);
				lines.push_back(indices[p6]);

				lines.push_back(indices[p4]);
				lines.push_back(indices[p5]);

				borderGeom->setUseVertexBufferObjects(true);
				borderGeom->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::LINES,
																	  lines.begin(),
																	  lines.end()));

				geode->addDrawable(borderGeom);
			}

			osg::ref_ptr<osg::StateSet> stateSet = geode->getOrCreateStateSet();
			
			// 禁用光照
			stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
			// 启用混合模式
			stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
			stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

			/*
			// 禁止写入深度
			osg::ref_ptr<osg::Depth> depth = new osg::Depth;
			depth->setWriteMask(false);
			stateSet->setAttributeAndModes(depth, osg::StateAttribute::ON);

			// 启用背面剔除，避免半透明面的重复叠加
			osg::ref_ptr<osg::CullFace> cullFace = new osg::CullFace;
			cullFace->setMode(osg::CullFace::BACK);
			stateSet->setAttributeAndModes(cullFace, osg::StateAttribute::ON);
			*/

			return geode.release();
		}

		osg::Geode* CreateSphere(const osg::Vec3d& position,
								 double radius,
								 const osg::Vec4& color,
								 const osg::Vec3d& offset)
		{

			osg::ref_ptr<osg::Geode> geode = new osg::Geode();
			// 设置节点掩码，使其不被掩码为 0xfffffff0 的 NodeVisitor捕获
			geode->setNodeMask(0x00000003);

			osg::Vec3d center = position - offset;

			osg::ref_ptr<osg::ShapeDrawable> sphere =
				new osg::ShapeDrawable(new osg::Sphere(center, radius));

			sphere->setColor(color);

			geode->addChild(sphere);

			return geode.release();
		}

		osg::Geode* CreatePolyline(const std::vector<osg::Vec3d>& path,
								   const osg::Vec4& color,
								   const osg::Vec3d& offset /*= osg::Vec3d()*/,
								   int linewidth /*= 3*/)
		{
			osg::ref_ptr<osg::Geode> geode = new osg::Geode();
			// 设置节点掩码，使其不被掩码为 0xfffffff0 的 NodeVisitor捕获
			geode->setNodeMask(0x00000003);

			osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
			osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();

			for (size_t i = 0; i < path.size(); ++i)
			{ 
				vertices->push_back(path[i] - offset);
			}
			
			// 设置顶点
			geom->setVertexArray(vertices);

			// 设置颜色
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
			colors->push_back(color);
			geom->setColorArray(colors, osg::Array::BIND_OVERALL);

			geom->addPrimitiveSet(
				new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));
			geode->addDrawable(geom);

			osg::ref_ptr<osg::StateSet> stateSet = geode->getOrCreateStateSet();

			// 禁用光照
			stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
			// 启用混合模式
			stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);

			// 线宽
			osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth(linewidth);
			stateSet->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);

			return geode.release();
		}

		osg::Geode* CreatePointCloud(const pcl::PointCloud<PointPCLH>& points,
									 const std::vector<int>& indices)
		{
			if (indices.size() <= 0)
				return nullptr;

			osg::ref_ptr<osg::Geode> geode = new osg::Geode;
			osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
			osg::ref_ptr<osg::Vec3Array> pointArray = new osg::Vec3Array;
			osg::ref_ptr<osg::Vec4Array> colorArray = new osg::Vec4Array;
			osg::ref_ptr<osg::UIntArray> attrIndexAttry = new osg::UIntArray;


			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& point = points[indices[i]];
				osg::Vec3d pvec = osg::Vec3d(point.x, point.y, point.z);

				pointArray->push_back(pvec);
				colorArray->push_back(osg::Vec4((float)point.r / 255.f,
												(float)point.g / 255.f,
												(float)point.b / 255.f,
												1.f));

				// 顶点属性，记录在原始点云数据上的索引
				attrIndexAttry->push_back(indices[i]);
			}

			geometry->setUseDisplayList(false);
			geometry->setUseVertexBufferObjects(true);
			geometry->setStateSet(getOrCreatePointStateSet());
			geometry->setVertexArray(pointArray);

			// 设置并绑定顶点属性
			geometry->setVertexAttribArray(1, attrIndexAttry);
			geometry->setVertexAttribBinding(1, osg::Geometry::BIND_PER_VERTEX);

			geometry->setColorArray(colorArray);
			geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

			geometry->addPrimitiveSet(
				new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, pointArray->size()));

			geode->addDrawable(geometry.get());

			return geode.release();
		}

		osg::Geode* CreatePointCloud(PointCloudViewPtr pointCloudPtr,
									 const std::vector<int>& indices)
		{
			if (indices.size() <= 0)
				return nullptr;

			osg::ref_ptr<osg::Geode> geode = new osg::Geode;
			osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
			osg::ref_ptr<osg::Vec3Array> pointArray = new osg::Vec3Array;
			osg::ref_ptr<osg::Vec4Array> colorArray = new osg::Vec4Array;
			osg::ref_ptr<osg::UIntArray> attrIndexAttry = new osg::UIntArray;


			for (size_t i = 0; i < indices.size(); ++i)
			{
				const auto& point = pointCloudPtr->points[indices[i]];
				osg::Vec3d pvec = osg::Vec3d(point.x, point.y, point.z);

				pointArray->push_back(pvec);
				colorArray->push_back(osg::Vec4((float)point.r / 255.f,
												(float)point.g / 255.f,
												(float)point.b / 255.f,
												1.f));

				// 顶点属性，记录在原始点云数据上的索引
				attrIndexAttry->push_back(indices[i]);
			}

			geometry->setUseDisplayList(false);
			geometry->setUseVertexBufferObjects(true);
			geometry->setStateSet(getOrCreatePointStateSet());
			geometry->setVertexArray(pointArray);

			// 设置并绑定顶点属性
			geometry->setVertexAttribArray(1, attrIndexAttry);
			geometry->setVertexAttribBinding(1, osg::Geometry::BIND_PER_VERTEX);

			geometry->setColorArray(colorArray);
			geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

			geometry->addPrimitiveSet(
				new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, pointArray->size()));

			geode->addDrawable(geometry.get());

			return geode.release();
		}
	}
}