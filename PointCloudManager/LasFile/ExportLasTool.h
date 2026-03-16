//////////////////////////////////////////////////////////////////////
// 文件名称：ExportLasTool.h
// 功能描述：导出点云文件工具
// 创建标识：吴建峰 2026/3/01
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////

#ifndef EXPORT_LAS_TOOL_H_
#define EXPORT_LAS_TOOL_H_

#include <LASlib/laswriter.hpp>

#include <BusinessNode/PCNodeType.h>
#include <BusinessNode/BnsProjectNode.h>
#include <BusinessNode/BnsPointCloudNode.h>

#include <osg/Vec4ub>

// 写las工具信息
struct SLasWriterData
{
	SLasWriterData() : _pLaswriter(nullptr) {}

	LASwriteOpener _lasWriteOpener;
	LASheader _header;
	LASwriter* _pLaswriter;
	LASpoint _laspoint;
};

class CExportLasTool
{
public:
	CExportLasTool(pc::data::CModelNodePtr pProjectNode, const CString& strOutPath);
	~CExportLasTool();

public:
	/**
	 * 导出las文件
	 * @return
	 */
	void ExportLas();

protected:
	/**
	 * 执行导出 (带点云扫描时间)
	 * @param [in] pageLodList	pageLod
	 * @param [in] nEpsg			epsg
	 * @param [in] offsetXyz		偏移
	 * @param [in] strFileName	导出文件名称
	 * @param [in] bByType		是否按分类导出
	 * @param [in] typeFileMap	导出文件<分类类型，文件路径>
	 * @param [in] strScanTime	导出点云扫描时间
	 * @param [out] errorMap		错误信息<分类类型，是否成功>
	 * @return
	 */
	bool ExportLasFileWithTime(const std::vector<pc::data::CModelNodePtr>& pageLodList,
							   const int& nEpsg,
							   const osg::Vec3d& offsetXyz,
							   const CString& strFileName,
							   std::map<int, bool>& errorMap,
							   CString& strScanTime,
							   const bool& bByType = false,
							   const std::map<int, CString> typeFileMap = std::map<int, CString>(),
							   const std::vector<int>& vecType = std::vector<int>());

protected:
	/**
	 * 创建按分类导出的las文件写入器(带点云扫描时间)
	 * @param [in] typeLasWriterMap
	 * @param [in] strFileName	导出文件名称
	 * @param [in] bByType		是否按分类导出
	 * @param [in] typeFileMap	导出文件<分类类型，文件路径>
	 * @param [in] boundingBox	包围盒
	 * @param [in] nEpsg			epsg
	 * @param [in] offsetXyz		偏移
	 * @return
	 */
	bool CreateTypeLasWriterWithTime(std::map<int, SLasWriterData>& typeLasWriterMap,
									 const CString& strFileName,
									 const bool& bByType,
									 const std::map<int, CString> typeFileMap,
									 const osg::BoundingBox& boundingBox,
									 const int& nEpsg,
									 const osg::Vec3d& offsetXyz,
									 CString& strScanTime);

	/**
	 * 创建导出las文件头(带点云扫描时间)
	 * @param [out] lasWriteOpener	las写文件入口
	 * @param [out] header	文件头
	 * @param [in] boundingBox	点云包围盒
	 * @param [in] strFileName	文件名
	 * @param [in] nEpsg			epsg
	 * @param [in] offsetXyz		偏移
	 * @param [in] strScanTime	点云扫描时间
	 * @return
	 */
	void CreateFileHandleWithTime(LASwriteOpener& lasWriteOpener,
								  LASheader& header,
								  const osg::BoundingBox& boundingBox,
								  const CString& strFileName,
								  const int nEpsg,
								  const osg::Vec3d& offsetXyz,
								  CString& strScanTime);

	/**
	 * 创建写文件操作器
	 * @param [in] lasWriteOpener	las写文件入口
	 * @param [in] header	文件头
	 * @param [out] pLaswriter 写文件器
	 * @param [out] pLaspoint LAS点
	 * @return
	 */
	bool CreateWriteOperator(LASwriteOpener& lasWriteOpener,
							 LASheader& header,
							 LASwriter*& pLaswriter,
							 LASpoint& laspoint);

	/**
	 *  @brief    为LAS文件头写入地理信息
	 *  @param    LASheader * pHeader		LAS文件头信息
	 *  @param    const int nEpsg			地理坐标系EPSG编码
	 *  @return   void
	 */
	void SetGeotiff(LASheader* pHeader, const int nEpsg);

	/**
	 *  @brief    为LAS文件头写入点云扫描时间
	 *  @param    LASheader * pHeader		LAS文件头信息
	 *  @param    CString&  strScanTime		LAS点云扫描时间
	 *  @return   void
	 */
	void SetSacnTime(LASheader* pHeader, CString& strScanTime);

protected:
	/**
	 * 写las文件
	 * @param [in] pNode				osg节点
	 * @param [in] bByType			是否按分类导出
	 * @param [in] typeLasWriterMap	按分类写信息
	 * @param [in] offsetXyz			偏移
	 * @param [out] errorMap			错误信息<分类类型，是否成功>
	 * @return
	 */
	void WriteLas(osg::Node* pNode,
				  const bool& bByType,
				  std::map<int, SLasWriterData>& typeLasWriterMap,
				  const std::vector<int>& vecType,
				  std::map<int, bool>& errorMap,
				  const osg::Vec3d& offsetXyz);

	/**
	 * 写点信息
	 * @param [in] point	点
	 * @param [in] color 颜色
	 * @param [in] offsetXyz 偏移
	 * @param [in] nType 分类类型
	 * @param [in] pLaswriter las文件写入器
	 * @param [in] laspoint las点
	 * @return
	 */
	bool WritePointInfo(const osg::Vec3d& point,
						const osg::Vec4ub& color,
						const osg::Vec3d& offsetXyz,
						uint32_t nType,
						LASwriter* pLaswriter,
						LASpoint& laspoint);

protected:
	pc::data::CModelNodePtr _pProjectNode; // 工程节点
	CString _strOutPath;				   // 导出的目标目录
};

#endif // !EXPORT_LAS_TOOL_H_
