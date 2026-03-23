#include <include/PointCloudManagerRefs.h>
#include <Tool/FileToolkit.h>
#include <include/Log.h>

#include <cstdio>
#include <vector>


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

	return showTypeMap;
}

int main(int argc, char* argv[])
{
	d3s::CLog::GetInst()->EnableRedirectTo(d3s::CLog::Sink::StdOutColor);

	if (argc != 3)
	{
		d3s::CLog::Warn("Usage: %s <input_las_file> <output_project_file>", argv[0]);
		return 1;
	}

	CString strLasFile = CStringToolkit::UTF8ToCString(argv[1]);
	CString strProjectPath = CStringToolkit::UTF8ToCString(argv[2]);

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

	CString prjID = bnsProject.GetID();

	/*2.导入las文件*/
	auto start = std::chrono::steady_clock::now();

	/*2.导入las文件*/
	CLasFileToolkit::ImportLasFile({ strLasFile }, bnsProject, strProjectPath);

	auto end = std::chrono::steady_clock::now();
	auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "ImportLasFile() cost: " << elapsed_ms.count() << " ms" << std::endl;


#if 1
	/*3.自动分类*/
	pc::data::SegmentParam segmentParam;

	segmentParam._strVolLevel = L"500kV";
	segmentParam._strTopographicFeatures = L"平坦";
	segmentParam._showTypeMap = GetSegmentShowType();
	std::map<unsigned, osg::BoundingBox> clusterBoxMap = {};
	const std::map<int, int> mapConvertType = CDataManager::GetTypeConvertInfo();
	segmentParam._nDenoiseLevel = 3;
	pc::data::CModelNodeVector nodes = bnsProject.GetPointCloudNodeList();
	CPCAutoSegmentToolkit::Segment(nodes, segmentParam, clusterBoxMap, mapConvertType);


	// 4. 单木分割(测试)
	osg::BoundingBox bbox(-78.9816284, -184.480499, 0.0, 129.116196, 127.675163, 0.0);
	std::set<unsigned> vegetationTypes = { 2 };
	std::map<unsigned, osg::BoundingBox> clusterBoxMap2;
	const unsigned groundType = 1;

	CPCAutoSegmentToolkit::TreeIndividual(nodes, bbox, vegetationTypes, groundType, clusterBoxMap2);


	/*5.导出las*/
	// CLasFileToolkit::ExportLasFile(bnsProject, CFileToolkit::GetFileDirectory(strLasFile));
#endif


	/*6.保存工程文件*/
	CProjectManagerTool::SaveProject(strProjectFile, bnsProject);

	d3s::CLog::Info("%s 向你问好!\n", "ConsoleApplicationTestSeg");
	return 0;
}
