//////////////////////////////////////////////////////////////////////
// 文件名称：BnsPointCloudNode.h
// 功能描述：工程节点包装类
// 创建标识：吴建峰 2026/1/24
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef BNS_POINT_CLOUD_NODE_H_
#define BNS_POINT_CLOUD_NODE_H_

#include <include/PointCloudManagerExport.h>
#include <include/BnsWrapper.h>

#include <osg/BoundingBox>

class POINTCLOUDMANAGER_EXPORT CBnsPointCloudNode : public pc::data::BnsWrapper
{
public:
	using pc::data::BnsWrapper::BnsWrapper;
	virtual ~CBnsPointCloudNode();

public:
	/**
	 * 设置对应的IVE文件名称
	 * @param [in] nEpsg
	 * @return
	 */
	void setFileName(const CString& strName);
	CString getFileName();

	/**
	 * 数据基准目录
	 * @param [in] nEpsg
	 * @return
	 */
	void setDatabasePath(const CString& strName);
	CString getDatabasePath();

	/**
	 * 偏移点
	 * @param [in] nEpsg
	 * @return
	 */
	void setOffset(const osg::Vec3d& pos);
	osg::Vec3d getOffset();

	/**
	 * 设置中心点
	 * @param [in] nEpsg
	 * @return
	 */
	void setCenter(const osg::Vec3d& pos);
	osg::Vec3d getCenter();

	/**
	 * 设置模型包围盒
	 * @param [in] nEpsg
	 * @return
	 */
	void SetModelBoundingBox(const osg::BoundingBox& boundingBox);
	osg::BoundingBox GetModelBoundingBox();

	void SetRealBoundingBox(const osg::BoundingBox& boundingBox);
	osg::BoundingBox GetRealBoundingBox();

	void setRadius(double Radius);
	double getRadius();


	/**
	 * 设置点数量
	 * @param [in] nEpsg
	 * @return
	 */
	void SetPointsNum(int nPointNum);
	int GetPointsNum(void);

	/**
	 * 设置层级
	 * @param [in] nEpsg
	 * @return
	 */
	void SetPcLevel(int nLevel);
	int GetPcLevel(void);
};

#endif // BNS_POINT_CLOUD_NODE_H_
