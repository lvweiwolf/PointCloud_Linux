#include <src/io/pointCloudToLod.h>
#include <src/io/tileToLod.h>
#include <src/utils/logging.h>

#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <memory>
#include <vector>

namespace d3s {
	namespace pcs {
		namespace io {

			PointCloudToLOD::PointCloudToLOD() {}

			PointCloudToLOD::~PointCloudToLOD() {}

			bool PointCloudToLOD::Export(const std::string& input,
										 const std::string& output,
										 const std::string& exportMode,
										 int tileSize,
										 int maxPointNumPerOneNode,
										 int maxTreeDepth,
										 float lodRatio,
										 float pointSize,
										 std::string colorMode)
			{
				// check export mode
				ExportMode eExportMode;

				if (exportMode == "osgb")
				{
					eExportMode = ExportMode::OSGB;
				}
				else if (exportMode == "3mx")
				{
					eExportMode = ExportMode::_3MX;
				}
				else
				{
					PCS_ERROR("[pcl::io::PointCloudToLOD::Export] Mode %s is NOT support!.",
							  exportMode.c_str());

					return false;
				}

				PCS_INFO("[pcl::io::PointCloudToLOD::Export] Export mode: %s.", exportMode.c_str());

				// check color mode
				ColorMode eColorMode;

				if (colorMode == "debug")
				{
					eColorMode = ColorMode::Debug;
				}
				else if (colorMode == "rgb")
				{
					eColorMode = ColorMode::RGB;
				}
				else if (colorMode == "iGrey")
				{
					eColorMode = ColorMode::IntensityGrey;
				}
				else if (colorMode == "iBlueWhiteRed")
				{
					eColorMode = ColorMode::IntensityBlueWhiteRed;
				}
				else if (colorMode == "iHeightBlend")
				{
					eColorMode = ColorMode::IntensityHeightBlend;
				}
				else
				{

					PCS_ERROR(
						"[pcl::io::PointCloudToLOD::Export] ColorMode %s is NOT supported now.",
						colorMode.c_str());

					return false;
				}

				PCS_INFO("[pcl::io::PointCloudToLOD::Export] Color mode: %s.", colorMode.c_str());

				// check input
				std::shared_ptr<PointCloudReader> pointsReader =
					std::shared_ptr<PointCloudReader>(new PointCloudReader(input));

				if (!pointsReader->IsValid())
					return false;


				std::string ext = osgDB::getFileExtensionIncludingDot(input);
				PCS_INFO("[pcl::io::PointCloudToLOD::Export] Input file format.", ext.c_str());

				// check output
				if (osgDB::makeDirectory(output) == false)
				{
					PCS_ERROR("[pcl::io::PointCloudToLOD::Export] Make directory %s failed!",
							  output.c_str());

					return false;
				}
				std::string filePathData = output + "/Data";
				if (osgDB::makeDirectory(filePathData) == false)
				{
					PCS_ERROR("[pcl::io::PointCloudToLOD::Export] Make directory %s failed!",
							  filePathData.c_str());
					return false;
				}

				// convert
				std::vector<PointRGBI> lstPoints;
				lstPoints.reserve(tileSize);

				int processedPoints = 0;
				size_t tileID = 0;
				std::vector<std::string> tileIds;
				std::vector<std::string> tileRelativePaths;
				std::vector<osg::BoundingBox> tileBBoxes;

				while (
					this->LoadPointsForOneTile(pointsReader, lstPoints, tileSize, processedPoints))
				{
					TileToLOD lodGenerator(maxTreeDepth,
										   maxPointNumPerOneNode,
										   lodRatio,
										   pointSize,
										   pointsReader->GetBoundingBox(),
										   eColorMode);

					std::string tileName = "Tile_" + std::to_string(tileID++);
					std::string tilePath = filePathData + "/" + tileName;

					if (osgDB::makeDirectory(tilePath) == false)
					{
						PCS_ERROR("[pcl::io::PointCloudToLOD::Export] Make directory %s failed!",
								  tilePath.c_str());

						return false;
					}

					osg::BoundingBox box;
					if (!lodGenerator.Generate(&lstPoints, tilePath, tileName, eExportMode, box))
					{
						PCS_ERROR(
							"[pcl::io::PointCloudToLOD::Export] Generate point tiles %s failed!",
							tilePath.c_str());

						return false;
					}

					std::string topLevelNodeRelativePath;
					if (eExportMode == ExportMode::OSGB)
					{
						topLevelNodeRelativePath = tileName + "/" + tileName + ".osgb";
					}
					else if (eExportMode == ExportMode::_3MX)
					{
						topLevelNodeRelativePath = tileName + "/" + tileName + ".3mxb";
					}
					else
					{
						PCS_ERROR("[pcl::io::PointCloudToLOD::Export] Mode %d is NOT support!",
								  eExportMode);

						return false;
					}

					if (osgDB::fileExists(filePathData + "/" + topLevelNodeRelativePath))
					{
						tileIds.push_back(tileName);
						tileRelativePaths.push_back(topLevelNodeRelativePath);
						tileBBoxes.push_back(box);
					}

					processedPoints += lstPoints.size();
				}

				// export root and metadata
				if (eExportMode == ExportMode::OSGB)
				{
					std::string outputRoot = output + "/Root.osgb";
					std::string outputMetadata = output + "/metadata.xml";
					osg::ref_ptr<osg::MatrixTransform> pRoot = new osg::MatrixTransform();
					auto l_oOffset = pointsReader->GetOffset();
					pRoot->setMatrix(
						osg::Matrix::translate(l_oOffset.x(), l_oOffset.y(), l_oOffset.z()));

					for (int i = 0; i < (int)tileRelativePaths.size(); i++)
					{
						std::string tilepath = output + "/Data/" + tileRelativePaths[i];
						osg::ref_ptr<osg::Node> tileNode = osgDB::readNodeFile(tilepath);

						pRoot->addChild(tileNode);
					}


					// osg::ref_ptr<osgDB::Options> pOpt = new osgDB::Options("precision=15");

					if (osgDB::writeNodeFile(*pRoot, outputRoot) == false)
					{
						PCS_ERROR("[pcl::io::PointCloudToLOD::Export] Write node file %s failed!",
								  outputRoot.c_str());
					}

					std::string wkt = "";

					if (!ExportSRS(wkt, outputMetadata))
					{
						PCS_ERROR("[pcl::io::PointCloudToLOD::Export] Generate %s failed!",
								  outputMetadata.c_str());

						return false;
					}
				}
				else
				{
					PCS_ERROR("[pcl::io::PointCloudToLOD::Export] Mode %d is NOT support!",
							  eExportMode);

					return false;
				}

				return true;
			}

			bool PointCloudToLOD::LoadPointsForOneTile(
				std::shared_ptr<PointCloudReader> pointsReader,
				std::vector<PointRGBI>& lstPoints,
				size_t tileSize,
				size_t processedPoints)
			{
				lstPoints.clear();
				size_t count = 0;

				if (pointsReader->GetNumOfPoints() - processedPoints <= 1.5 * tileSize)
				{
					tileSize = pointsReader->GetNumOfPoints() - processedPoints;
				}

				PointRGBI point;
				while (count < tileSize)
				{
					int l_nFlag = pointsReader->ReadNextPoint(point);
					if (l_nFlag > 0)
					{
						point.P -= pointsReader->GetOffset();

						lstPoints.push_back(point);
						++count;
						if (l_nFlag == 2)
						{
							break;
						}
					}
					else
					{
						break;
					}
				}
				if (count > 0)
				{
					return true;
				}
				else
				{
					return false;
				}
			}

			bool PointCloudToLOD::ExportSRS(const std::string& srs, const std::string& filePath)
			{
				std::ofstream outfile(filePath);
				if (outfile.bad())
				{
					PCS_ERROR("[pcl::io::PointCloudToLOD::Export] Can NOT open file %s!",
							  filePath.c_str());
					return false;
				}

				outfile << "<?xml version=\"1.0\" encoding=\"utf - 8\"?>\n";
				outfile << "<ModelMetadata version=\"1\">\n";
				outfile << "	<SRS>\n";
				outfile << "		<WKT>" << srs << "</WKT>\n";
				outfile << "	</SRS>\n";
				outfile << "	<SRSOrigin>0, 0, 0</SRSOrigin>\n";
				outfile << "	<Texture>\n";
				outfile << "		<ColorSource>Visible</ColorSource>\n";
				outfile << "	</Texture>\n";
				outfile << "</ModelMetadata>\n";

				if (outfile.bad())
				{
					PCS_ERROR("[pcl::io::PointCloudToLOD::Export] An error has occurred while "
							  "writing file %s!",
							  filePath.c_str());

					return false;
				}

				return true;
			}
		}
	}
}