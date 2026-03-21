
#include <Segment/PropertyVisitorCommand.h>
#include <Segment/PointCloudPropertyVisitor.h>

bool CPcPropVisitorCommand::Excute(osg::ref_ptr<osg::Node> pNode,
								   const pc::data::SVisitorInfos& visitorInfos,
								   const CString& strPageNodeId)
{
	if (nullptr == pNode || nullptr == visitorInfos._pPropVisitorCallback)
		return false;

	CPointCloudSetPropVisitor visitor(visitorInfos, strPageNodeId);

	pNode->accept(visitor);
	return visitor.EffectiveTraversal();
}