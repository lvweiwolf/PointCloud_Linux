//////////////////////////////////////////////////////////////////////
// 文件名称：BnsProjectNode.h
// 功能描述：工程节点包装类
// 创建标识：吴建峰 2026/1/24
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef BNS_PROJECT_NODE_H_
#define BNS_PROJECT_NODE_H_

#include <include/PointCloudManagerExport.h>

#include <BusinessNode/BnsPointCloudNode.h>

class POINTCLOUDMANAGER_EXPORT CBnsProjectNode : public pc::data::BnsWrapper
{
public:
	using pc::data::BnsWrapper::BnsWrapper;
	virtual ~CBnsProjectNode();

public:
	/**
	 * 设置工程名称
	 * @param [in] strName	绝缘子类别
	 * @return
	 */
	void SetProjectName(const CString& strName);

	/**
	 * 获取工程名称
	 * @return
	 */
	CString GetProjectName();

	/**
	 * 添加点云节点
	 * @param [in] nEpsg
	 * @return
	 */
	CBnsPointCloudNode AddPointCloudNode();

	/**
	 * 插入点云节点
	 * @param [in] nEpsg
	 * @return
	 */
	bool InsertPointCloudNode(pc::data::CModelNodePtr pPointNode);

	/**
	 * 获取点云节点数组
	 * @param [in] nEpsg
	 * @return
	 */
	pc::data::CModelNodeVector GetPointCloudNodeList();

	/**
	 * 设置基准坐标
	 * @param [in] nEpsg
	 * @return
	 */
	void setBasePos(const osg::Vec3d& pos);
	osg::Vec3d getBasePos();


	/**
	 * 设置获取坐标
	 * @param [in] nEpsg
	 * @return
	 */
	void SetEpsg(int nEpsg);
	int GetEpsg();

	/**
	 * 设置点云时间
	 * @param [in] CString
	 * @return
	 */
	void SetScanTime(const CString& strScanTime);
	CString GetScanTime();
};

#endif // BNS_PROJECT_NODE_H_
