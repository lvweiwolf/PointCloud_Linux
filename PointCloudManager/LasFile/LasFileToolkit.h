//////////////////////////////////////////////////////////////////////
// 文件名称：LasFileToolkit.h
// 功能描述：点云文件工具
// 创建标识：吴建峰 2026/3/01
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef LASFILETOOLKIT_H_
#define LASFILETOOLKIT_H_

#include <include/PointCloudManagerExport.h>

#include <include/ModelNode.h>
#include <include/StringToolkit.h>

class POINTCLOUDMANAGER_EXPORT CLasFileToolkit
{
public:
	/**
	 * Las文件解析
	 * @param [in] lasPathList		Las文件路径
	 * @param [in] nEpsg			坐标
	 * @param [in] pProjectNode		添加到指定的工程
	 * @param [in] strOutPath		解析到指定的目录
	 * @return
	 */
	static void ImportLasFile(const std::vector<CString>& lasPathList,
							  pc::data::CModelNodePtr pProjectNode,
							  const CString& strOutPath);

	/**
	 * 导出Las文件
	 * @param [in] pWrapperList
	 * @param [in] strFileName 导出文件
	 * @return
	 */
	static bool GetPointCloudEpsg(int& nOutEpsg, const CString& strLasFile);

	/**
	 * 导出Las文件
	 * @param [in] pWrapperList
	 * @param [in] strFileName 导出文件
	 * @return
	 */
	static void ExportLasFile(pc::data::CModelNodePtr pProjectNode, const CString& strOutPath);
};


#endif // LASFILETOOLKIT_H_