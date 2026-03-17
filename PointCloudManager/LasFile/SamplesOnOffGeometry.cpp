#include <LasFile/SamplesOnOffGeometry.h>

#include <GL/gl.h>

bool CSamplesOnOffGeometry::_bSamplesState = true;
CSamplesOnOffGeometry::CSamplesOnOffGeometry(
	const CSamplesOnOffGeometry& conle,
	const osg::CopyOp& copyop /*= osg::CopyOp::DEEP_COPY_ALL*/)
	: osg::Geometry(conle, copyop)
{
	_bSamplesState = conle._bSamplesState;
}

CSamplesOnOffGeometry::CSamplesOnOffGeometry() {}

CSamplesOnOffGeometry::CSamplesOnOffGeometry(const osg::Geometry& conle)
	: osg::Geometry(conle, osg::CopyOp::SHALLOW_COPY)
{
}

CSamplesOnOffGeometry::~CSamplesOnOffGeometry() {}

void CSamplesOnOffGeometry::drawImplementation(osg::RenderInfo& renderInfo) const
{
	if (_bSamplesState)
	{
		osg::Geometry::drawImplementation(renderInfo);
		return;
	}

	// glDisable(GL_MULTISAMPLE_ARB);
	osg::Geometry::drawImplementation(renderInfo);
	// glEnable(GL_MULTISAMPLE_ARB);
}

void CSamplesOnOffGeometry::SetSamplesState(bool bSamplesState) { _bSamplesState = bSamplesState; }

bool CSamplesOnOffGeometry::GetSamplesState() { return _bSamplesState; }