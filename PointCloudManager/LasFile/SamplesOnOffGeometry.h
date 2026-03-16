//////////////////////////////////////////////////////////////////////
// 文件名称：SamplesOnOffGeometry.h
// 功能描述：反走样开关图元
// 创建标识：李成 2022/10/18
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef SAMPLES_ON_OFF_GEOMETRY_H_
#define SAMPLES_ON_OFF_GEOMETRY_H_

#include <include/PointCloudManagerExport.h>
#include <LasFile/PointCloudPageLod.h>

#include <osg/Geometry>

class POINTCLOUDMANAGER_EXPORT CSamplesOnOffGeometry : public osg::Geometry
{
public:
	CSamplesOnOffGeometry(const CSamplesOnOffGeometry& conle,
						  const osg::CopyOp& copyop = osg::CopyOp::DEEP_COPY_ALL);
	CSamplesOnOffGeometry(const osg::Geometry& conle);
	CSamplesOnOffGeometry();
	virtual ~CSamplesOnOffGeometry();
	META_NodeEx(osg, CSamplesOnOffGeometry);

public:
	virtual void drawImplementation(osg::RenderInfo& renderInfo) const;

	/**
	 * 设置\获取反走样状态
	 * @param [in] bSamplesState
	 * @return
	 */
	void SetSamplesState(bool bSamplesState);
	bool GetSamplesState();

private:
	static bool _bSamplesState; // 反走样状态
};

#endif // SAMPLES_ON_OFF_GEOMETRY_H_