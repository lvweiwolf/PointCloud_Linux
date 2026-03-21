#ifndef PROPERTIESVISITORCOMMAND_H_
#define PROPERTIESVISITORCOMMAND_H_

#include <include/cstring.h>
#include <include/PointCloudManagerExport.h>

#include <LasFile/PointCloudToolDefine.h>

#include <osg/Node>

class POINTCLOUDMANAGER_EXPORT CPcPropVisitorCommand
{
public:
	/**
	 * 执行
	 * @param [in] pNode				节点
	 * @param [in] visitorInfos		遍历信息
	 * @param [in] strPageNodeId		PageNode id（扩展信息，用于提供给自定义回调内使用）
	 * @return 是否有效执行
	 */
	static bool Excute(osg::ref_ptr<osg::Node> pNode,
					   const pc::data::SVisitorInfos& visitorInfos,
					   const CString& strPageNodeId = L"");

};




#endif // PROPERTIESVISITORCOMMAND_H_