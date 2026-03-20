#include <include/PointCloudSegAPI.h>

// 分类器
#include <src/service/Segmentation.h>

// 分类参数配置
#include <src/service/Options.h>

// log信息
#include <src/utils/logging.h>

// 文件存在工具
#include <src/utils/misc.h>

#include <src/service/ValueBuffer.h>
#include <src/service/GeoProjectionConvertor.h>
#include <src/service/PointCloud.h>
#include <src/service/Recognition.h>

#include <osg/ref_ptr>
#include <osgViewer/Viewer>
#include <osgGA/CameraManipulator>
#include <osg/Group>

#include <boost/property_tree/ptree_fwd.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <string>
#include <exception>

// 当前激活的视图
osg::ref_ptr<osgViewer::Viewer> activeViewer;
osg::ref_ptr<osgGA::CameraManipulator> manipulator;
std::string DebugDirectory = R"(/usr/local/temp/PointCloudDATA/pylon_mask/)";

// 激活的模型
osg::ref_ptr<osg::Group> dynamicModel = new osg::Group;
osg::ref_ptr<osg::Group> staticModel = new osg::Group;

namespace d3s {
	namespace pcs {

		ICloudSegmentation* CreateSegmentation(d3s::pcs::SegmentationType eSegType)
		{
			ICloudSegmentation* segmentation = nullptr;

			switch (eSegType)
			{
			case SegmentationType::GROUND:
				segmentation = new CGroundSegmentation();
				break;
			case SegmentationType::POWERCORRIDORS:
				segmentation = new CPowerCorridorSegmentaion();
				break;
			case SegmentationType::ENVIROMENTS:
				segmentation = new CEnviromentSegmentation();
				break;
			case SegmentationType::TREEINDIVIDUAL:
				segmentation = new CTreeIndividual();
				break;
			case SegmentationType::TREESPECIESIDENTIFY:
				segmentation = new CTreeSpeciesIdentify();
				break;
			case SegmentationType::CROPS:
				segmentation = new CCropSegmentation();
				break;
			// case SegClassification::DANGER:
			//	segmentation = new d3s::pcs::CDangerSegmentation();
			//	break;
			default:
				break;
			}
			return segmentation;
		}

		IOptions* CreateSegOption(const char* iniFile)
		{
			IOptions* options = new d3s::pcs::COptions();

			if (iniFile)
			{
				std::string path = iniFile;

				boost::property_tree::ptree ini;
				// 这个位置如果没有读取到文件会崩溃,需要加异常处理
				boost::property_tree::ini_parser::read_ini(path, ini);
				auto optionItems = ini.get_child("Options");

				for (auto it = optionItems.begin(); it != optionItems.end(); ++it)
				{
					try
					{
						auto value = (*it).second.data();
						options->Set((*it).first.data(), value.c_str());
					}
					catch (std::exception& e)
					{
						PCS_ERROR("读取 INI 参数项错误: %s.", e.what());
					}
				}
			}
			else
			{
				PCS_INFO("未指定 INI 配置文件，创建空的 Options.");
			}
			return options;
		}

		IGeoProjectionConvertor* CreateGeoProjectionConvertor(int epsg)
		{
			return new CGeoProjectionConvertor(epsg);
		}

		ICloudDetection* CreateaPylonDetection() { return new CPylonDetection(); }

		ICloudReconstrct* CreatePylonReconstruct() { return new CPylonReconstruct(); }

		ICloudCurveFitting* CreatePowerlineFitting() { return new CPowerlineCurveFitting(); }

		ICloudClustering* CreateCloudClustering() { return new CCloudClusteringImpl(); }

		ICloudRasterize* CreateCloudRasterize() { return new CTiffRasterize(); }

		IPointCloud* CreatePointCloud() { return new CPointCloud(); }

		IValueBuffer* CreateRoadVectorize(const std::string& filename)
		{
			if (filename.empty())
				return nullptr;

			// 文件不存在
			if (!ExistsPath(filename))
			{
				PCS_ERROR("[ReadRoadVectorize] %s 文件不能存在!", filename.c_str());
				return nullptr;
			}

			d3s::pcs::CRoadVectorizeBuffer* roadBuffer = new d3s::pcs::CRoadVectorizeBuffer();
			// roadBuffer->ReadShapefile(filename);
			roadBuffer->ReadCompressFile(filename);

			return roadBuffer;
		}

		d3s::pcs::IOptions* CreateOptions(const char* iniFile)
		{
			IOptions* options = new COptions();

			if (iniFile)
			{
				std::string path = iniFile;

				boost::property_tree::ptree ini;
				try
				{
					boost::property_tree::ini_parser::read_ini(path, ini);
				}
				catch (std::exception& e)
				{
					PCS_INFO("读取INI 配置文件错误，创建空的 Options.");
					return options;
				}
				auto optionItems = ini.get_child("Options");
				for (auto it = optionItems.begin(); it != optionItems.end(); ++it)
				{
					try
					{
						auto value = (*it).second.data();
						options->Set((*it).first.data(), value.c_str());
					}
					catch (std::exception& e)
					{
						PCS_ERROR("读取 INI 参数项错误: %s.", e.what());
					}
				}
			}
			else
			{
				PCS_INFO("未指定 INI 配置文件，创建空的 Options.");
			}

			return options;
		}

		d3s::pcs::IImageRecognition* CreateRecogition(RecognitionType recognitionType)
		{
			IImageRecognition* pRecognition = NULL;
			switch (recognitionType)
			{
			case d3s::pcs::RecognitionType::eCrop:
			{
				pRecognition = new CCropRecognition();
			}
			break;
			default:
				break;
			}

			return pRecognition;
		}

	}
}
