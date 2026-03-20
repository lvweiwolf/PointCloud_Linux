//////////////////////////////////////////////////////////////////////
// 文件名称：ProjectManagerTool.h
// 功能描述：工程管理类
// 创建标识：吴建峰 2026/1/24
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////

#ifndef PROJECTMANAGERTOOL_H_
#define PROJECTMANAGERTOOL_H_

#include <include/PointCloudManagerExport.h>

#include <BusinessNode/BnsProjectNode.h>

class POINTCLOUDMANAGER_EXPORT CProjectManagerTool
{
public:
	// 加载或者创建工程
	static CBnsProjectNode LoadProject(const CString& strFile);

	// 保存工程
	static bool SaveProject(const CString& strFile, CBnsProjectNode bnsProject);
};

#endif // PROJECTMANAGERTOOL_H_
