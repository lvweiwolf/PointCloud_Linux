#include <include/PointCloudManagerRefs.h>
#include <Tool/FileToolkit.h>

#include <cstdio>
#include <vector>
#include <chrono>

std::map<char, bool> GetSegmentShowType()
{
	std::map<char, bool> showTypeMap;
	showTypeMap[pc::data::GROUND_CLASSIFY] = true;
	showTypeMap[pc::data::TOWER_CLASSIFY] = true;
	showTypeMap[pc::data::LOW_VEGETATION] = true;
	showTypeMap[pc::data::HIGH_VEGETATION] = true;
	showTypeMap[pc::data::WIRE_CLASSIFY] = true;
	showTypeMap[pc::data::CROSS_POWER_LINE_CLASSIFY] = true;
	showTypeMap[pc::data::JUMPER_WIRE_CLASSIFY] = true;
	showTypeMap[pc::data::POWER_LINE_CLASSIFY] = true;
	showTypeMap[pc::data::ROAD_CLASSIFY] = true;
	showTypeMap[pc::data::INSULATOR_CLASSIFY] = true;
	showTypeMap[pc::data::JK_GROUND_WIRE_CLASSIFY] = true;
	return std::move(showTypeMap);
}


int main()
{
	CString strLasFile = _T("/home/lvwei/cloud/temp/test2.las");
	CString strProjectPath = _T("/home/lvwei/cloud/temp/projects/");

	CString strProjectID = L"test.xmdx";
	CString strProjectFile = strProjectPath + strProjectID;

	/*1.加载或创建工程*/
	CBnsProjectNode bnsProject = CProjectManagerTool::LoadProject(strProjectFile);
	if (bnsProject.IsNull())
	{
		pc::data::CModelNodePtr pProject =
			new pc::data::CModelNode(EProjectNodeType::eBnsProjectRoot);
		bnsProject = pProject;
	}

	if (bnsProject.IsNull())
		return 0;


	/*2.导入las文件*/
	auto start = std::chrono::steady_clock::now();

	CLasFileToolkit::ImportLasFile({ strLasFile }, bnsProject, strProjectPath);

	auto end = std::chrono::steady_clock::now();
	auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "ImportLasFile() cost: " << elapsed_ms.count() << " ms" << std::endl;

#if 0
	/*3.保存工程文件*/
	CProjectManagerTool::SaveProject(strProjectFile, bnsProject);

	/*4.自动分类*/
	pc::data::SegmentParam segmentParam;

	segmentParam._strVolLevel = L"500kV";
	segmentParam._strTopographicFeatures = L"平坦";
	segmentParam._showTypeMap = GetSegmentShowType();
	std::map<unsigned, osg::BoundingBox> clusterBoxMap = {};
	const std::map<int, int> mapConvertType = CDataManager::GetTypeConvertInfo();
	segmentParam._nDenoiseLevel = 3;
	pc::data::CModelNodeVector nodes = bnsProject.GetPointCloudNodeList();
	CPCAutoSegmentToolkit::Segment(nodes, segmentParam, clusterBoxMap, mapConvertType);

	/*5.导出las*/
	CLasFileToolkit::ExportLasFile(bnsProject, CFileToolkit::GetFileDirectory(strLasFile));

	/* d3s::pcs::ICloudSegmentation* seg =
	 d3s::pcs::CreateSegmentation(SegClassification::eSegmentGround); std::string  strOptionPath=
	 "/home/whm/test.ini"; d3s::pcs::IOptions* opt = d3s::pcs::CreateOptions(strOptionPath.c_str());
	 std::vector<d3s::pcs::PointId> result;
	 seg->Segment(opt, result);*/

#endif

	printf("%s 向你问好!\n", "ConsoleApplicationTestSeg");
	return 0;
}
