//stdafx.h
#include "plotHandle.h"

#include <numeric>

#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/ComputeBoundsVisitor>
#include <osgViewer/Viewer>

#include "geomCreator.h"
#include "../segmentation/gridCell.h"
#include "../segmentation/powerCorridorsClassify.h"
#include "../segmentation/pylonLocation.h"
#include "../segmentation/improve/roadClassifier.h"
#include "../segmentation/improve/attachmentClassifier.h"
#include "../utils/logging.h"

extern osg::ref_ptr<osgViewer::Viewer> activeViewer;
extern osg::ref_ptr<osg::Group> dynamicModel;
extern osg::ref_ptr<osg::Group> staticModel;

static std::mutex render_mutex;

namespace d3s {
	namespace pcs {

		osg::Vec4ub ColorManager::colorTable[eNumOfClassification] = {
			osg::Vec4ub(255, 255, 255, 255), // eUnclassified = 0,
			osg::Vec4ub(250, 164, 120, 255), // eGround,
			osg::Vec4ub(51, 51, 255, 255),	 // ePylon,
			osg::Vec4ub(255, 0, 255, 255),	 // ePowerline,
			osg::Vec4ub(252, 70, 0, 255),	 // eLeadwire,
			osg::Vec4ub(255, 200, 50, 255),	 // eIsulator,
			osg::Vec4ub(121, 243, 0, 255),	 // eLowVegetation,
			osg::Vec4ub(40, 120, 58, 255),	 // eHighVegetation,
			osg::Vec4ub(19, 106, 213, 255),	 // eBuilding,
			osg::Vec4ub(61, 64, 77, 255),	 // eRoad,
			osg::Vec4ub(151, 219, 242, 255), // eWater,
			osg::Vec4ub(29, 40, 68, 255),	 // eOtherline,
			osg::Vec4ub(255, 215, 0, 255),	 // eCrop_Corn
			osg::Vec4ub(255, 0, 215, 255),	 // eCrop_Vegetable
			osg::Vec4ub(128, 55, 55, 255),	 // eCrop_GreenHouse
			osg::Vec4ub(30, 215, 0, 255),	 // eCrop_Paddy

		};

		osg::Vec4 ColorManager::colorMap[256] = { osg::Vec4() };

		std::unordered_map<int, int> ColorManager::powerlineClusterLabel;
		std::vector<int> ColorManager::treeSegments;
		std::vector<int> ColorManager::treeSpecies;
		std::vector<int> ColorManager::debugTags;
		std::unordered_map<int, osg::Vec4> ColorManager::treeColorMap;
		std::unordered_map<int, osg::Vec4> ColorManager::treeSpeciesColorMap;
		std::unordered_map<int, osg::Vec4> ColorManager::debugColorMap;

		// ColorizeVisitor
		//////////////////////////////////////////////////////////////////////////
		class ColorizeVisitor : public osg::NodeVisitor
		{
		public:
			enum RenderMode
			{
				Classification,
				RGB,
				Elevation,
				NormalizeElevation,
				Cluster,
				TreeSegmentation,
				TreeSpeciesClassification,
				DebugTag,
				ShowHide,
			};

			ColorizeVisitor(PointCloudViewConstPtr pointClourdPtr, RenderMode mode);

			virtual ~ColorizeVisitor();

			void SetShowClassification(int classification);

			virtual void apply(osg::Geode& geode);
			virtual void apply(osg::Geometry& geometry);
			virtual void apply(osg::Drawable& drawable);

		protected:
			PointCloudViewConstPtr _pointCloudPtr;
			RenderMode _renderMode;
			int _classification;
		};


		// PlotHandler
		//////////////////////////////////////////////////////////////////////////
		PlotHandler::PlotHandler() { ColorManager::CreateColorMap(); }

		PlotHandler* PlotHandler::inst = nullptr;

		PlotHandler* PlotHandler::GetInst()
		{
			if (!inst)
				inst = new PlotHandler();

			return inst;
		}


		bool PlotHandler::AddNode(const std::string& name,
								  osg::ref_ptr<osg::Node> node,
								  bool dynamic /*= true*/)
		{

			auto it = _nodeNamed.find(name);

			if (it != _nodeNamed.end())
				return false;

			_nodeNamed[name] = node;

			if (dynamic)
			{
				dynamicModel->addChild(node);
			}
			else
			{
				staticModel->addChild(node);
			}

			return true;
		}

		bool PlotHandler::RemoveNode(const std::string& name, bool dynamic /*= true*/)
		{
			auto it = _nodeNamed.find(name);

			if (it == _nodeNamed.end())
				return false;

			auto& node = _nodeNamed[name];

			if (dynamic)
			{
				dynamicModel->removeChild(node);
			}
			else
			{
				staticModel->removeChild(node);
			}

			_nodeNamed.erase(name);

			return true;
		}

		osg::ref_ptr<osg::Node> PlotHandler::GetNode(const std::string& name,
													 bool dynamic /*= true*/)
		{
			auto it = _nodeNamed.find(name);

			if (it == _nodeNamed.end())
				return nullptr;

			return _nodeNamed[name];
		}

		// ColorManager
		//////////////////////////////////////////////////////////////////////////
		void ColorManager::CreateColorMap()
		{
			std::vector<osg::Vec4> steps;

			osg::Vec4 blue(0., 0., 1., 1.);
			osg::Vec4 green(0., 1., 0., 1.);
			osg::Vec4 yellow(1., 1., 0., 1.);
			osg::Vec4 red(1., 0., 0., 1.);
			steps.push_back(blue);
			steps.push_back(green);
			steps.push_back(yellow);
			steps.push_back(red);

			assert(steps.size() >= 2);

			for (int i = 0; i < 256; ++i)
			{
				float intensity = (float)i / 255.f * (float)(steps.size() - 1);
				int floor = intensity;
				floor = std::max(0, floor);

				if (floor >= steps.size() - 1)
				{
					colorMap[i] = steps.back();
				}
				else
				{
					float upper = intensity - floor;
					float lower = 1. - upper;
					colorMap[i] = steps[floor] * lower + steps[floor + 1] * upper;
				}
			}
		}


		// ClassifyColorizeVisitor
		//////////////////////////////////////////////////////////////////////////
		ColorizeVisitor::ColorizeVisitor(PointCloudViewConstPtr pointClourdPtr, RenderMode mode)
			: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
			  _pointCloudPtr(pointClourdPtr),
			  _renderMode(mode),
			  _classification(-1)
		{
		}

		ColorizeVisitor::~ColorizeVisitor() {}

		void ColorizeVisitor::SetShowClassification(int classification)
		{
			_classification = classification;
		}

		void ColorizeVisitor::apply(osg::Geode& geode)
		{
			for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
			{
				osg::Drawable* drawable = geode.getDrawable(i);

				if (drawable)
					apply(*drawable);
			}
		}

		void ColorizeVisitor::apply(osg::Geometry& geometry)
		{
			if (_pointCloudPtr)
			{
				osg::ref_ptr<osg::UIntArray> attrIndexArray =
					dynamic_cast<osg::UIntArray*>(geometry.getVertexAttribArray(1));

				osg::ref_ptr<osg::Vec4Array> colorArray =
					dynamic_cast<osg::Vec4Array*>(geometry.getColorArray());

				if (!attrIndexArray.valid() || !colorArray.valid())
					return;

				CHECK(attrIndexArray->size() == colorArray->size());


				for (size_t i = 0; i < attrIndexArray->size(); ++i)
				{
					const auto& pointIndex = attrIndexArray->at(i);

					switch (_renderMode)
					{
					case Classification:
					{
						const auto& label = _pointCloudPtr->points[pointIndex].label;

						// 设置对应类别颜色
						const auto& labelColor = ColorManager::colorTable[label];

						osg::Vec4& color = colorArray->at(i);
						color.r() = (float)labelColor.r() / 255.f;
						color.g() = (float)labelColor.g() / 255.f;
						color.b() = (float)labelColor.b() / 255.f;
						color.a() = 1.0f;
					}
					break;
					case RGB:
					{
						const auto& point = _pointCloudPtr->points[pointIndex];

						osg::Vec4& color = colorArray->at(i);
						color.r() = (float)point.r / 255.f;
						color.g() = (float)point.g / 255.f;
						color.b() = (float)point.b / 255.f;
						color.a() = 1.0f;
					}
					break;
					case Elevation:
					{
						const auto& point = _pointCloudPtr->points[pointIndex];
						// 高程比例
						float x = (point.z - _pointCloudPtr->bbox.zMin()) /
								  (_pointCloudPtr->bbox.zMax() - _pointCloudPtr->bbox.zMin());

						// 在高度颜色映射中找到对应颜色
						int index = x * 255;
						index = std::max(0, std::min(255, index));
						osg::Vec4& color = colorArray->at(i);
						color = ColorManager::colorMap[index];
					}
					break;
					case NormalizeElevation:
					{
						CHECK(_pointCloudPtr->normalized);

						double heightAboveGround = _pointCloudPtr->points[pointIndex].hag;

						// 高程比例
						float x = (heightAboveGround) /
								  (_pointCloudPtr->bbox.zMax() - _pointCloudPtr->bbox.zMin());

						// 在高度颜色映射中找到对应颜色
						int index = x * 255;
						index = std::max(0, std::min(255, index));
						osg::Vec4& color = colorArray->at(i);
						color = ColorManager::colorMap[index];
					}
					break;
					case Cluster:
					{
						const auto& point = _pointCloudPtr->points[pointIndex];
						const auto iter = ColorManager::powerlineClusterLabel.find((int)pointIndex);
						osg::Vec4& color = colorArray->at(i);

						if (iter == ColorManager::powerlineClusterLabel.end())
						{
							color.r() = (float)point.r / 255.f;
							color.g() = (float)point.g / 255.f;
							color.b() = (float)point.b / 255.f;
							color.a() = 1.0f;
						}
						else
						{
							osg::Vec4ub colorub =
								ColorManager::colorTable[iter->second % eNumOfClassification];
							color = osg::Vec4((float)colorub.r() / 255.f,
											  (float)colorub.g() / 255.f,
											  (float)colorub.b() / 255.f,
											  (float)colorub.a() / 255.f);
						}
					}
					break;
					case TreeSegmentation:
					{
						auto treeId = ColorManager::treeSegments[pointIndex];
						osg::Vec4& color = colorArray->at(i);

						if (treeId < 0)
						{
							// 不是树木，按照类别着色
							const auto& label = _pointCloudPtr->points[pointIndex].label;
							// 设置对应类别颜色
							const auto& labelColor = ColorManager::colorTable[label];

							color.r() = (float)labelColor.r() / 255.f;
							color.g() = (float)labelColor.g() / 255.f;
							color.b() = (float)labelColor.b() / 255.f;
							color.a() = 1.0f;
						}
						else
						{
							color = ColorManager::treeColorMap[treeId];
						}
					}
					break;
					case TreeSpeciesClassification:
					{
						auto treeClassId = ColorManager::treeSpecies[pointIndex];
						osg::Vec4& color = colorArray->at(i);

						if (treeClassId < 0)
						{
							// 不是树种，按照类别着色
							const auto& label = _pointCloudPtr->points[pointIndex].label;
							// 设置对应类别颜色
							const auto& labelColor = ColorManager::colorTable[label];

							color.r() = (float)labelColor.r() / 255.f;
							color.g() = (float)labelColor.g() / 255.f;
							color.b() = (float)labelColor.b() / 255.f;
							color.a() = 1.0f;
						}
						else
						{
							color = ColorManager::treeSpeciesColorMap[treeClassId];
						}
					}
					break;
					case DebugTag:
					{
						auto tagId = ColorManager::debugTags[pointIndex];
						osg::Vec4& color = colorArray->at(i);

						if (tagId < 0)
						{
							// 不是树木，按照类别着色
							const auto& label = _pointCloudPtr->points[pointIndex].label;
							// 设置对应类别颜色
							const auto& labelColor = ColorManager::colorTable[label];

							color.r() = (float)labelColor.r() / 255.f;
							color.g() = (float)labelColor.g() / 255.f;
							color.b() = (float)labelColor.b() / 255.f;
							color.a() = 1.0f;
						}
						else
						{
							color = ColorManager::debugColorMap[tagId];
						}
					}
					break;
					case ShowHide:
					{
						if (_classification >= 0)
						{
							const auto& label = _pointCloudPtr->points[pointIndex].label;

							osg::Vec4& color = colorArray->at(i);

							if (label == _classification)
							{
								const auto& labelColor = ColorManager::colorTable[label];

								color.r() = (float)labelColor.r() / 255.f;
								color.g() = (float)labelColor.g() / 255.f;
								color.b() = (float)labelColor.b() / 255.f;
								color.a() = 1.0f;
							}
							else
							{
								const auto& label = _pointCloudPtr->points[pointIndex].label;
								// 设置对应类别颜色
								const auto& labelColor = ColorManager::colorTable[label];

								color.r() = (float)labelColor.r() / 255.f;
								color.g() = (float)labelColor.g() / 255.f;
								color.b() = (float)labelColor.b() / 255.f;
								color.a() = 0.1f;
							}
						}
					}
					break;
					default:
						break;
					}
				}

				colorArray->dirty();
				geometry.dirtyDisplayList();
			}
		}

		void ColorizeVisitor::apply(osg::Drawable& drawable)
		{
			if (drawable.asGeometry())
				apply(*drawable.asGeometry());
		}

		// Plot API
		//////////////////////////////////////////////////////////////////////////
		void UpdateAxisPlot()
		{
			// 删除坐标轴
			PlotHandler::GetInst()->RemoveNode("AXIS_0", false);

			osg::ComputeBoundsVisitor bv;
			dynamicModel->accept(bv);

			PlotHandler::GetInst()->AddNode("AXIS_0", CreateAxis(bv.getBoundingBox()), false);


			if (activeViewer.valid())
				activeViewer->home();
		}

		void RenderColorFromClassifcation(const std::string& name,
										  PointCloudViewConstPtr pointCloudPtr)
		{
			std::unique_lock<std::mutex> lock(render_mutex);

			if (!pointCloudPtr)
				return;

			osg::ref_ptr<osg::Node> pontCloudNode = PlotHandler::GetInst()->GetNode(name);

			if (pontCloudNode.valid())
			{
				ColorizeVisitor cv(pointCloudPtr, ColorizeVisitor::Classification);
				pontCloudNode->accept(cv);
			}
		}

		void RenderColorFromRGB(const std::string& name, PointCloudViewConstPtr pointCloudPtr)
		{
			if (!pointCloudPtr)
				return;

			osg::ref_ptr<osg::Node> pontCloudNode = PlotHandler::GetInst()->GetNode(name);

			if (pontCloudNode.valid())
			{
				ColorizeVisitor cv(pointCloudPtr, ColorizeVisitor::RGB);
				pontCloudNode->accept(cv);
			}
		}

		void RenderColorFromElevation(const std::string& name, PointCloudViewConstPtr pointCloudPtr)
		{
			if (!pointCloudPtr)
				return;

			osg::ref_ptr<osg::Node> pontCloudNode = PlotHandler::GetInst()->GetNode(name);

			if (pontCloudNode.valid())
			{
				ColorizeVisitor cv(pointCloudPtr, ColorizeVisitor::Elevation);
				pontCloudNode->accept(cv);
			}
		}

		void RenderColorFromNormalizeElevation(const std::string& name,
											   PointCloudViewConstPtr pointCloudPtr)
		{
			if (!pointCloudPtr)
				return;

			osg::ref_ptr<osg::Node> pontCloudNode = PlotHandler::GetInst()->GetNode(name);

			if (pontCloudNode.valid())
			{
				ColorizeVisitor cv(pointCloudPtr, ColorizeVisitor::NormalizeElevation);
				pontCloudNode->accept(cv);
			}
		}

		void RenderColorFromCluster(const std::string& name, PointCloudViewConstPtr pointCloudPtr)
		{
			if (!pointCloudPtr)
				return;

			osg::ref_ptr<osg::Node> pontCloudNode = PlotHandler::GetInst()->GetNode(name);

			if (pontCloudNode.valid())
			{
				ColorizeVisitor cv(pointCloudPtr, ColorizeVisitor::Cluster);
				pontCloudNode->accept(cv);
			}
		}

		void RenderColorFromTreeSegmentation(const std::string& name,
											 PointCloudViewConstPtr pointCloudPtr)
		{
			if (!pointCloudPtr)
				return;

			osg::ref_ptr<osg::Node> pontCloudNode = PlotHandler::GetInst()->GetNode(name);

			if (pontCloudNode.valid())
			{
				ColorizeVisitor cv(pointCloudPtr, ColorizeVisitor::TreeSegmentation);
				pontCloudNode->accept(cv);
			}
		}

		void RenderColorFromTreeSpceiesClassification(const std::string& name,
													  PointCloudViewConstPtr pointCloudPtr)
		{
			if (!pointCloudPtr)
				return;

			osg::ref_ptr<osg::Node> pontCloudNode = PlotHandler::GetInst()->GetNode(name);

			if (pontCloudNode.valid())
			{
				ColorizeVisitor cv(pointCloudPtr, ColorizeVisitor::TreeSpeciesClassification);
				pontCloudNode->accept(cv);
			}
		}

		void RenderColorFromDebugTag(const std::string& name, PointCloudViewConstPtr pointCloudPtr)
		{
			if (!pointCloudPtr)
				return;

			osg::ref_ptr<osg::Node> pontCloudNode = PlotHandler::GetInst()->GetNode(name);

			if (pontCloudNode.valid())
			{
				ColorizeVisitor cv(pointCloudPtr, ColorizeVisitor::DebugTag);
				pontCloudNode->accept(cv);
			}
		}

		void ShowExceptClassfication(const std::string& name,
									 PointCloudViewConstPtr pointCloudPtr,
									 int classfication)
		{
			if (!pointCloudPtr)
				return;

			osg::ref_ptr<osg::Node> pontCloudNode = PlotHandler::GetInst()->GetNode(name);

			if (pontCloudNode.valid())
			{
				ColorizeVisitor cv(pointCloudPtr, ColorizeVisitor::ShowHide);
				cv.SetShowClassification(classfication);
				pontCloudNode->accept(cv);
			}
		}

		void RenderGridCells(double cellsize, double step, PointCloudViewPtr pointCloudPtr)
		{
			Grid2D gridCells;
			std::vector<int> all(pointCloudPtr->size());
			std::iota(all.begin(), all.end(), 0);

			ComputeGridCells(cellsize, pointCloudPtr, all, gridCells);

			osg::Vec4 colorBorder(1.f, 0.f, 0.f, 0.3f);
			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			transform->setMatrix(osg::Matrixd::translate(pointCloudPtr->offset_xyz));

			for (size_t i = 0; i < gridCells.size(); ++i)
			{
				const auto& cell = gridCells[i];
				std::string cellkey = StringPrintf("Cell_%d", i);

				if (cell.GetSize() > 0)
				{
					// 在高度颜色映射中找到对应颜色
					osg::Vec4 colorFill = ColorManager::colorMap[0];
					colorFill.a() = 0.0f;

					osg::ref_ptr<osg::Geode> cube = CreateCube(cell, colorFill, colorBorder);

					cube->setName(cellkey);
					transform->addChild(cube);
				}
			}

			PlotHandler::GetInst()->AddNode("gridcell0", transform);
		}

		static int grid_index = 1;

		void RenderGridCells(const Grid3D& grid, PointCloudViewPtr pointCloudPtr)
		{
			CHECK(!grid.empty());

			std::string name = StringPrintf("gridcell%d", grid_index++);

			int xSize = grid.getXSize();
			int ySize = grid.getYSize();
			int zSize = grid.getZSize();

			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			transform->setMatrix(osg::Matrixd::translate(pointCloudPtr->offset_xyz));

			for (int zi = 0; zi < zSize; ++zi)
			{
				for (int yi = 0; yi < ySize; ++yi)
				{
					for (int xi = 0; xi < xSize; ++xi)
					{
						auto cell = grid.get(xi, yi, zi);

						if (!cell)
							continue;

						std::string cellkey = StringPrintf("Cell_%d_%d_%d", xi, yi, zi);

						if (cell->GetSize() > 0)
						{
							osg::Vec4 colorFill;
							osg::Vec4 colorBorder;

							if (cell->label == ePowerline)
							{
								colorBorder = osg::Vec4(1.f, 0.f, 0.f, 0.3f);
								colorFill = osg::Vec4(1.f, 0.f, 0.f, 0.1f);
							}
							else
							{
								colorBorder = osg::Vec4(1.f, 1.f, 1.f, 0.3f);
								colorFill = osg::Vec4(1.f, 1.f, 1.f, 0.1f);
							}

							osg::ref_ptr<osg::Geode> cube =
								CreateCube(*cell, colorFill, colorBorder);

							cube->setName(cellkey);
							transform->addChild(cube);
						}
					}
				}
			}

			PlotHandler::GetInst()->AddNode(name, transform);
		}

		void RenderGridCells(const LocationGrid& grid,
							 const std::vector<osg::Vec4>& colorMap,
							 PointCloudViewPtr pointCloudPtr)
		{
			CHECK(!grid.empty());

			std::string name = StringPrintf("gridcell%d", grid_index++);

			int nRows = grid.getNumRows();
			int nCols = grid.getNumColumns();

			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			transform->setMatrix(osg::Matrixd::translate(pointCloudPtr->offset_xyz));

			for (int r = 0; r < nRows; ++r)
			{
				for (int c = 0; c < nCols; ++c)
				{
					const auto& cell = grid.at(r, c);
					std::string cellkey = StringPrintf("Cell_%d_%d", r, c);

					if (cell.GetSize() > 0)
					{
						osg::Vec4 colorBorder, colorFill;

						const auto& cellFeature = grid.GetFeatureAt(r, c);

						if (cellFeature->label >= 0)
						{
							osg::Vec4 color = colorMap[cellFeature->label];
							colorBorder = osg::Vec4(color.r(), color.g(), color.b(), 0.5f);
							colorFill = osg::Vec4(color.r(), color.g(), color.b(), 0.1f);
						}
						else
						{
							/*colorBorder = osg::Vec4(1.f, 1.f, 1.f, 0.5f);
							colorFill = osg::Vec4(1.f, 1.f, 1.f, 0.1f);*/
							continue;
						}


						osg::ref_ptr<osg::Geode> cube = CreateCube(cell, colorFill, colorBorder);

						cube->setName(cellkey);
						transform->addChild(cube);
					}
				}
			}

			PlotHandler::GetInst()->AddNode(name, transform);
		}

		void RenderGridCells(const VirtualGrid& grid,
							 const std::vector<std::vector<std::array<int, 2>>>& clusters,
							 const std::vector<osg::Vec4>& colorMap,
							 PointCloudViewPtr pointCloudPtr)
		{
			CHECK(!grid.empty());
			CHECK(clusters.size() == colorMap.size());

			std::string name = StringPrintf("gridcell%d", grid_index++);

			int nRows = grid.getNumRows();
			int nCols = grid.getNumColumns();

			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			transform->setMatrix(osg::Matrixd::translate(pointCloudPtr->offset_xyz));

			for (size_t i = 0; i < clusters.size(); ++i)
			{
				const auto& cluster = clusters.at(i);

				for (size_t j = 0; j < cluster.size(); ++j)
				{
					const auto& index = cluster.at(j);
					int ri = index[0];
					int ci = index[1];

					std::string cellkey = StringPrintf("Cell_%d_%d", ri, ci);
					const auto& cell = grid.at(ri, ci);

					if (!cell || cell->GetSize() <= 0)
						continue;

					osg::Vec4 colorBorder, colorFill;
					osg::Vec4 color = colorMap[i];
					colorBorder = osg::Vec4(color.r(), color.g(), color.b(), 0.5f);
					colorFill = osg::Vec4(color.r(), color.g(), color.b(), 0.1f);

					osg::ref_ptr<osg::Geode> cube = CreateCube(*cell, colorFill, colorBorder);

					cube->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
					cube->setName(cellkey);

					transform->addChild(cube);
				}
			}

			PlotHandler::GetInst()->AddNode(name, transform);
		}

		void RenderBlocks(const std::vector<double>& xlist,
						  const std::vector<double>& ylist,
						  double blockSize,
						  PointCloudViewPtr pointCloudPtr)
		{
			if (!pointCloudPtr)
				return;

			if (xlist.size() != ylist.size())
				return;

			const auto& bbox = pointCloudPtr->bbox;

			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			transform->setMatrix(osg::Matrixd::translate(pointCloudPtr->offset_xyz));

			for (int idx = 0; idx < xlist.size(); ++idx)
			{
				double xbegin = xlist[idx];
				double ybegin = ylist[idx];

				osg::BoundingBox blockBox(
					osg::Vec3(xbegin, ybegin, bbox.zMin()),
					osg::Vec3(xbegin + blockSize, ybegin + blockSize,bbox.zMax()));

				osg::Vec4 fillClr(1.f, 1.f, 1.f, 0.0f);
				osg::Vec4 borderClr(1.f, 1.f, 1.f, 0.5f);
				osg::ref_ptr<osg::Geode> cube = CreateCube(blockBox, fillClr, borderClr);
				transform->addChild(cube);
			}

			PlotHandler::GetInst()->AddNode("block_boxs", transform);
		}

		void RenderPylonGrid(const PylonGrid3D& grid, PointCloudViewPtr pointCloudPtr)
		{
			CHECK(!grid.empty());

			std::string name = StringPrintf("gridcell%d", grid_index++);

			int xSize = grid.getXSize();
			int ySize = grid.getYSize();
			int zSize = grid.getZSize();

			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			transform->setMatrix(osg::Matrixd::translate(pointCloudPtr->offset_xyz));

			osg::ref_ptr<osg::MatrixTransform> obbTransform = new osg::MatrixTransform;
			obbTransform->setMatrix(osg::Matrix::inverse(grid.rotation) *
									osg::Matrix::translate(grid.center));
			transform->addChild(obbTransform);

			for (int zi = 0; zi < zSize; ++zi)
			{
				for (int yi = 0; yi < ySize; ++yi)
				{
					for (int xi = 0; xi < xSize; ++xi)
					{
						auto cell = grid.get(xi, yi, zi);

						if (!cell || cell->GetSize() <= 0)
							continue;

						auto featureCell = grid.GetFeature(xi, yi, zi);

						if (!featureCell)
							continue;

						osg::Vec4 colorBorder, colorFill;

#if 0
						double x_thr = 0.35;
						double x_length_thr = 2.0;
						bool is_thin_X = featureCell->xz_fillrate < x_thr;

						int index = featureCell->xz_fillrate * 255;
						// int index = featureCell->nbr_fill_rate * 255;
						// int index = std::min(featureCell->thickness / 10.0, 1.0) * 255;
						
						index = std::max(0, std::min(255, index));
						osg::Vec4 color = ColorManager::colorMap[index];

						if (is_thin_X)
							color = osg::Vec4(1, 0, 0, 1);
						else
							color = osg::Vec4(1, 1, 1, 1);

						colorBorder = osg::Vec4(color.r(), color.g(), color.b(), 0.3f);
						colorFill = osg::Vec4(color.r(), color.g(), color.b(), 0.1f);
#else
						if (featureCell->cluster_id == 0) 
						{
							colorBorder = osg::Vec4(0, 1.0f, 0, 0.3f);
							colorFill = osg::Vec4(0, 1.0f, 0, 0.1f);
						}
						else if (featureCell->cluster_id > 0)
						{
							colorBorder = osg::Vec4(206.f / 255.f, 57.f / 255.f, 5.f / 255.f, 0.3f);
							colorFill = osg::Vec4(206.f / 255.f, 57.f / 255.f, 5.f / 255.f, 0.1f);
						}
						else
						{
							colorBorder = osg::Vec4(1, 1, 1, 0.3f);
							colorFill = osg::Vec4(1, 1, 1, 0.01f);
						}
#endif

						osg::ref_ptr<osg::Geode> cube = CreateCube(*cell, colorFill, colorBorder);
						cube->setName(StringPrintf("Cell_%d_%d_%d", xi, yi, zi));
						obbTransform->addChild(cube);
					}
				}
			}

			PlotHandler::GetInst()->AddNode(name, transform);
		}

		void RenderPylonPositions(const std::vector<osg::Vec3d>& positions,
								  double radius,
								  PointCloudViewPtr pointCloudPtr)
		{
			for (size_t i = 0; i < positions.size(); ++i)
			{
				std::string name = StringPrintf("pylon_pos_%d", i);

				RenderPosition(name, positions.at(i), radius, pointCloudPtr);
			}
		}

		void RenderPosition(const std::string& name,
							const osg::Vec3d& position,
							double radius,
							PointCloudViewPtr pointCloudPtr,
							osg::Vec4 color)
		{
			std::unique_lock<std::mutex> lock(render_mutex);

			PlotHandler::GetInst()->RemoveNode(name);

			// 创建铁塔链接路径并显示
			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			transform->setMatrix(osg::Matrixd::translate(pointCloudPtr->offset_xyz));
			transform->addChild(CreateSphere(position, radius, color));


			PlotHandler::GetInst()->AddNode(name, transform);
		}

		void RenderLinePath(const std::string& name,
							const std::vector<osg::Vec3d>& path,
							PointCloudViewPtr pointCloudPtr,
							osg::Vec4 color)
		{
			std::unique_lock<std::mutex> lock(render_mutex);

			PlotHandler::GetInst()->RemoveNode(name);

			// 创建铁塔链接路径并显示
			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			transform->setMatrix(osg::Matrixd::translate(pointCloudPtr->offset_xyz));
			transform->addChild(CreatePolyline(path, color));

			PlotHandler::GetInst()->AddNode(name, transform);
		}

		void RenderPowerlineCurve(const std::string& name,
								  const std::vector<osg::Vec3d>& verts,
								  PointCloudViewPtr pointCloudPtr,
								  osg::Vec4 color)
		{
			std::unique_lock<std::mutex> lock(render_mutex);

			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			transform->setMatrix(osg::Matrixd::translate(pointCloudPtr->offset_xyz));
			transform->addChild(CreatePolyline(verts, color));

			PlotHandler::GetInst()->AddNode(name, transform);
		}

		void RenderPylonMask(const std::vector<osg::Vec3d>& path,
							 PointCloudViewPtr pointCloudPtr,
							 std::string name)
		{
			std::unique_lock<std::mutex> lock(render_mutex);

			CHECK(!path.empty());

			PlotHandler::GetInst()->RemoveNode(name);

			std::vector<osg::Vec3d> polygonVerts = path;
			polygonVerts.push_back(*path.begin());

			// 创建铁塔链接路径并显示
			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			transform->setMatrix(osg::Matrixd::translate(pointCloudPtr->offset_xyz));
			transform->addChild(CreatePolyline(polygonVerts, osg::Vec4(0.f, 0.f, 1.f, 1.f)));

			PlotHandler::GetInst()->AddNode(name, transform);
		}

		void RenderPowerlineMask(const std::vector<osg::Vec3d>& path,
								 PointCloudViewPtr pointCloudPtr,
								 std::string name,
								 osg::Vec4 color)
		{
			std::unique_lock<std::mutex> lock(render_mutex);

			CHECK(!path.empty());

			PlotHandler::GetInst()->RemoveNode(name);

			std::vector<osg::Vec3d> polygonVerts = path;
			polygonVerts.push_back(*path.begin());

			// 创建铁塔链接路径并显示
			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			transform->setMatrix(osg::Matrixd::translate(pointCloudPtr->offset_xyz));
			transform->addChild(CreatePolyline(polygonVerts, color));

			PlotHandler::GetInst()->AddNode(name, transform);
		}

		void RenderAABB(const osg::BoundingBox& aabb,
						PointCloudViewPtr pointCloudPtr,
						std::string name,
						osg::Vec4 fill,
						osg::Vec4 border) 
		{
			std::unique_lock<std::mutex> lock(render_mutex);
			PlotHandler::GetInst()->RemoveNode(name);

			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			transform->setMatrix(osg::Matrixd::translate(pointCloudPtr->offset_xyz));
			transform->addChild(CreateCube(aabb, fill, border));
			PlotHandler::GetInst()->AddNode(name, transform);
		}

		void RenderOBB(const osg::BoundingBox& aabb,
					   const osg::Vec3d& position,
					   const osg::Matrix& rotation,
					   PointCloudViewPtr pointCloudPtr,
					   std::string name)
		{
			std::unique_lock<std::mutex> lock(render_mutex);

			PlotHandler::GetInst()->RemoveNode(name);

			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			transform->setMatrix(osg::Matrixd::translate(pointCloudPtr->offset_xyz));

			osg::ref_ptr<osg::MatrixTransform> obbTransform = new osg::MatrixTransform;
			obbTransform->setMatrix(rotation * osg::Matrix::translate(position));
			obbTransform->addChild(CreateCube(aabb, osg::Vec4(1, 1, 1, 0), osg::Vec4(1, 1, 1, 1)));
			transform->addChild(obbTransform);

			PlotHandler::GetInst()->AddNode(name, transform);
		}

		void RenderAddMatrix(const std::string& name, const osg::Matrix& matrix) 
		{
			std::unique_lock<std::mutex> lock(render_mutex);

			osg::ref_ptr<osg::Node> node = PlotHandler::GetInst()->GetNode(name);
			PlotHandler::GetInst()->RemoveNode(name);

			osg::ref_ptr<osg::MatrixTransform> transformNode = new osg::MatrixTransform;
			transformNode->setMatrix(matrix);
			transformNode->addChild(node);

			PlotHandler::GetInst()->AddNode(name, transformNode);
		}

		void RenderKeyPositions(const std::vector<int>& kp,
								const osg::Vec3d& minPt,
								const osg::Vec3d& maxPt,
								const osg::Matrix& inverseTransform,
								double step,
								PointCloudViewPtr pointCloudPtr,
								std::string name)
		{
			osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
			transform->setMatrix(osg::Matrixd::translate(pointCloudPtr->offset_xyz));
			
			for (size_t i = 0; i < kp.size(); ++i)
			{
				int index = kp.at(i);

				osg::Vec3d min(minPt.x(), minPt.y(), minPt.z() + index * step);
				osg::Vec3d max(maxPt.x(), maxPt.y(), minPt.z() + (index + 1) * step);

				min = inverseTransform.preMult(min);
				max = inverseTransform.preMult(max);

				osg::ref_ptr<osg::Node> cube = CreateCube(osg::BoundingBox(min, max),
														  osg::Vec4(1, 1, 1, 0.2),
														  osg::Vec4(1, 1, 1, 1));

				transform->addChild(cube);
			}

			PlotHandler::GetInst()->AddNode(name, transform);
		}

	}
}
