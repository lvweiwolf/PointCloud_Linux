#include <LasFile/PointCloudPropertyTool.h>

#include <algorithm>

// 分类属性最大值
#define MAX_SEGMENT 0xFF
// 高亮属性字节位
#define HIGHLIGHT_BIT 9
// 障碍属性字节位
#define OBSTACLE_BIT 10
// 显隐属性字节位
#define DISPLAY_ENABLE_BIT 11
// 属性默认值
#define DEFAULT_SEGMENT 0
#define DEFAULT_HIGHLIGHT 0
#define DEFAULT_OBSTACLE 0
#define DEFAULT_DISPLAY_ENABLE 1
#define DEFAULT_TREE_INDIVIDUAL 2000


osg::Vec2 CPointCloudPropertyTool::GetDefaultProperty()
{
	unsigned int nDefaultSegment = DEFAULT_SEGMENT;
	unsigned int nDefaultHighlight = DEFAULT_HIGHLIGHT << (HIGHLIGHT_BIT - 1);
	unsigned int nDefaultObstacle = DEFAULT_OBSTACLE << (OBSTACLE_BIT - 1);
	unsigned int nDefaultDisplayEnable = DEFAULT_DISPLAY_ENABLE << (DISPLAY_ENABLE_BIT - 1);
	return osg::Vec2(nDefaultSegment | nDefaultHighlight | nDefaultObstacle | nDefaultDisplayEnable,
					 0);
}

bool CPointCloudPropertyTool::SetSegmentProperty(
	osg::Geometry& geometry,
	const size_t nPointIndex,
	const uint32_t nType,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /* = {}*/)
{
	osg::ref_ptr<osg::Vec2Array> pTexCoordArray =
		dynamic_cast<osg::Vec2Array*>(geometry.getTexCoordArray(0));
	geometry.dirtyDisplayList();
	return SetSegmentProperty(pTexCoordArray, nPointIndex, nType, bHideLimit, hideSegmentList);
}

bool CPointCloudPropertyTool::SetSegmentProperty(
	osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
	const size_t nPointIndex,
	const uint32_t nType,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /* = {}*/)
{
	if (bHideLimit && IsHide(pTexCoordArray, nPointIndex, hideSegmentList))
		return false;
	if (nullptr == pTexCoordArray || nPointIndex >= pTexCoordArray->size() || nType > MAX_SEGMENT)
		return false;

	int nA = pTexCoordArray->at(nPointIndex).x();
	nA = nA - (nA & MAX_SEGMENT);
	nA = nA | nType;
	pTexCoordArray->at(nPointIndex).x() = nA;

	pTexCoordArray->dirty();
	return true;
}

bool CPointCloudPropertyTool::GetSegmentProperty(
	const osg::Geometry& geometry,
	const size_t nPointIndex,
	uint32_t& nType,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /* = {}*/)
{
	const osg::Vec2Array* pTexCoordArray =
		dynamic_cast<const osg::Vec2Array*>(geometry.getTexCoordArray(0));
	return GetSegmentProperty(pTexCoordArray, nPointIndex, nType, bHideLimit, hideSegmentList);
}

bool CPointCloudPropertyTool::GetSegmentProperty(
	const osg::Vec2Array* pTexCoordArray,
	const size_t& nPointIndex,
	uint32_t& nType,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /* = {}*/)
{
	if (bHideLimit && IsHide(pTexCoordArray, nPointIndex, hideSegmentList))
		return false;
	if (nullptr == pTexCoordArray || nPointIndex >= pTexCoordArray->size())
		return false;

	unsigned int nA = pTexCoordArray->at(nPointIndex).x();
	nType = nA & MAX_SEGMENT;

	return true;
}

bool CPointCloudPropertyTool::SetSegmentProperty(osg::Vec2& property, const uint32_t nType)
{
	if (nType > MAX_SEGMENT)
		return false;

	int nA = property.x();
	nA = nA - (nA & MAX_SEGMENT);
	nA = nA | nType;
	property.x() = nA;
	return true;
}

bool CPointCloudPropertyTool::SetHighlightProperty(
	osg::Geometry& geometry,
	const size_t nPointIndex,
	const bool bHighlight,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	osg::ref_ptr<osg::Vec2Array> pTexCoordArray =
		dynamic_cast<osg::Vec2Array*>(geometry.getTexCoordArray(0));
	geometry.dirtyDisplayList();
	return SetHighlightProperty(pTexCoordArray,
								nPointIndex,
								bHighlight,
								bHideLimit,
								hideSegmentList);
}

bool CPointCloudPropertyTool::SetHighlightProperty(
	osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
	const size_t nPointIndex,
	const bool bHighlight,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	return SetBoolProperty(pTexCoordArray,
						   nPointIndex,
						   bHideLimit,
						   hideSegmentList,
						   bHighlight,
						   HIGHLIGHT_BIT);
}

bool CPointCloudPropertyTool::GetHighlightProperty(
	const osg::Geometry& geometry,
	const size_t nPointIndex,
	bool& bHighlight,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	const osg::Vec2Array* pTexCoordArray =
		dynamic_cast<const osg::Vec2Array*>(geometry.getTexCoordArray(0));
	return GetHighlightProperty(pTexCoordArray,
								nPointIndex,
								bHighlight,
								bHideLimit,
								hideSegmentList);
}

bool CPointCloudPropertyTool::GetHighlightProperty(
	const osg::Vec2Array* pTexCoordArray,
	const size_t nPointIndex,
	bool& bHighlight,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	return GetBoolProperty(pTexCoordArray,
						   nPointIndex,
						   bHideLimit,
						   hideSegmentList,
						   bHighlight,
						   HIGHLIGHT_BIT);
}

bool CPointCloudPropertyTool::SetObstacleProperty(
	osg::Geometry& geometry,
	const size_t nPointIndex,
	const bool bObstacle,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	osg::ref_ptr<osg::Vec2Array> pTexCoordArray =
		dynamic_cast<osg::Vec2Array*>(geometry.getTexCoordArray(0));
	geometry.dirtyDisplayList();
	return SetObstacleProperty(pTexCoordArray, nPointIndex, bObstacle, bHideLimit, hideSegmentList);
}

bool CPointCloudPropertyTool::SetObstacleProperty(
	osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
	const size_t nPointIndex,
	const bool bObstacle,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	return SetBoolProperty(pTexCoordArray,
						   nPointIndex,
						   bHideLimit,
						   hideSegmentList,
						   bObstacle,
						   OBSTACLE_BIT);
}

bool CPointCloudPropertyTool::GetObstacleProperty(
	const osg::Geometry& geometry,
	const size_t nPointIndex,
	bool& bObstacle,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	const osg::Vec2Array* pTexCoordArray =
		dynamic_cast<const osg::Vec2Array*>(geometry.getTexCoordArray(0));
	return GetObstacleProperty(pTexCoordArray, nPointIndex, bObstacle, bHideLimit, hideSegmentList);
}

bool CPointCloudPropertyTool::GetObstacleProperty(
	const osg::Vec2Array* pTexCoordArray,
	const size_t nPointIndex,
	bool& bObstacle,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	return GetBoolProperty(pTexCoordArray,
						   nPointIndex,
						   bHideLimit,
						   hideSegmentList,
						   bObstacle,
						   OBSTACLE_BIT);
}

bool CPointCloudPropertyTool::SetDisplayEnableProperty(
	osg::Geometry& geometry,
	const size_t nPointIndex,
	const bool bDisplayEnable,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	osg::ref_ptr<osg::Vec2Array> pTexCoordArray =
		dynamic_cast<osg::Vec2Array*>(geometry.getTexCoordArray(0));
	geometry.dirtyDisplayList();
	return SetDisplayEnableProperty(pTexCoordArray,
									nPointIndex,
									bDisplayEnable,
									bHideLimit,
									hideSegmentList);
}

bool CPointCloudPropertyTool::SetDisplayEnableProperty(
	osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
	const size_t nPointIndex,
	const bool bDisplayEnable,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	return SetBoolProperty(pTexCoordArray,
						   nPointIndex,
						   bHideLimit,
						   hideSegmentList,
						   bDisplayEnable,
						   DISPLAY_ENABLE_BIT);
}

bool CPointCloudPropertyTool::GetDisplayEnableProperty(
	const osg::Geometry& geometry,
	const size_t nPointIndex,
	bool& bDisplayEnable,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	const osg::Vec2Array* pTexCoordArray =
		dynamic_cast<const osg::Vec2Array*>(geometry.getTexCoordArray(0));
	return GetDisplayEnableProperty(pTexCoordArray,
									nPointIndex,
									bDisplayEnable,
									bHideLimit,
									hideSegmentList);
}

bool CPointCloudPropertyTool::GetDisplayEnableProperty(
	const osg::Vec2Array* pTexCoordArray,
	const size_t nPointIndex,
	bool& bDisplayEnable,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	return GetBoolProperty(pTexCoordArray,
						   nPointIndex,
						   bHideLimit,
						   hideSegmentList,
						   bDisplayEnable,
						   DISPLAY_ENABLE_BIT);
}

bool CPointCloudPropertyTool::SetClusterProperty(
	osg::Geometry& geometry,
	const size_t nPointIndex,
	const int nClusterID,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	osg::ref_ptr<osg::Vec2Array> pTexCoordArray =
		dynamic_cast<osg::Vec2Array*>(geometry.getTexCoordArray(0));
	geometry.dirtyDisplayList();
	return SetClusterProperty(pTexCoordArray, nPointIndex, nClusterID, bHideLimit, hideSegmentList);
}

bool CPointCloudPropertyTool::SetClusterProperty(
	osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
	const size_t nPointIndex,
	const int nClusterID,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	if (bHideLimit && IsHide(pTexCoordArray, nPointIndex, hideSegmentList))
		return false;
	if (nullptr == pTexCoordArray || nPointIndex >= pTexCoordArray->size())
		return false;

	pTexCoordArray->at(nPointIndex).y() = nClusterID;
	pTexCoordArray->dirty();
	return true;
}

bool CPointCloudPropertyTool::GetClusterProperty(
	const osg::Geometry& geometry,
	const size_t nPointIndex,
	int& nClusterID,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	const osg::Vec2Array* pTexCoordArray =
		dynamic_cast<const osg::Vec2Array*>(geometry.getTexCoordArray(0));
	return GetClusterProperty(pTexCoordArray, nPointIndex, nClusterID, bHideLimit, hideSegmentList);
}

bool CPointCloudPropertyTool::GetClusterProperty(
	const osg::Vec2Array* pTexCoordArray,
	const size_t nPointIndex,
	int& nClusterID,
	bool bHideLimit /* = true*/,
	const std::vector<unsigned>& hideSegmentList /*= std::vector<unsigned>{}*/)
{
	if (bHideLimit && IsHide(pTexCoordArray, nPointIndex, hideSegmentList))
		return false;
	if (nullptr == pTexCoordArray || nPointIndex >= pTexCoordArray->size())
		return false;

	nClusterID = pTexCoordArray->at(nPointIndex).y();
	return true;
}

bool CPointCloudPropertyTool::IsHide(const osg::Vec2Array* pTexCoordArray,
									 const size_t nPointIndex,
									 const std::vector<unsigned>& hideSegmentList)
{
	bool bDisplayEnable = true;
	if (nullptr == pTexCoordArray)
		return bDisplayEnable;

	GetDisplayEnableProperty(pTexCoordArray, nPointIndex, bDisplayEnable, false);
	if (!bDisplayEnable || hideSegmentList.empty())
		return !bDisplayEnable;
	return IsHideSegment(pTexCoordArray, nPointIndex, hideSegmentList);
}

bool CPointCloudPropertyTool::GetBoolProperty(const osg::Vec2Array* pTexCoordArray,
											  const size_t nPointIndex,
											  bool bHideLimit,
											  const std::vector<unsigned>& hideSegmentList,
											  bool& bValue,
											  const unsigned int nBit)
{
	if (bHideLimit && IsHide(pTexCoordArray, nPointIndex, hideSegmentList))
		return false;
	if (nullptr == pTexCoordArray || nPointIndex >= pTexCoordArray->size())
		return false;

	GetBit(pTexCoordArray->at(nPointIndex).x(), nBit, bValue);
	return true;
}


bool CPointCloudPropertyTool::SetBoolProperty(osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
											  const size_t nPointIndex,
											  bool bHideLimit,
											  const std::vector<unsigned>& hideSegmentList,
											  bool bValue,
											  const unsigned int nBit)
{
	if (bHideLimit && IsHide(pTexCoordArray, nPointIndex, hideSegmentList))
		return false;
	if (nullptr == pTexCoordArray || nPointIndex >= pTexCoordArray->size())
		return false;

	int nNum = pTexCoordArray->at(nPointIndex).x();
	SetBit(nNum, nBit, bValue);
	pTexCoordArray->at(nPointIndex).x() = nNum;
	pTexCoordArray->dirty();

	return true;
}

void CPointCloudPropertyTool::SetBit(int& nOperatorNum,
									 const unsigned int nBit,
									 const bool bSetValue)
{
	unsigned int nNum = nOperatorNum;
	unsigned int nBitNum = 1 << (nBit - 1);
	nNum = (~nBitNum) & nNum;

	if (bSetValue)
	{
		nNum |= nBitNum;
	}
	nOperatorNum = nNum;
}

void CPointCloudPropertyTool::GetBit(const int nOperatorNum,
									 const unsigned int nBit,
									 bool& bGetValue)
{
	unsigned int nNum = nOperatorNum;
	unsigned int nBitNum = nNum >> (nBit - 1);
	bGetValue = (nBitNum & 1);
}

bool CPointCloudPropertyTool::IsHideSegment(const osg::Vec2Array* pTexCoordArray,
											const size_t nPointIndex,
											const std::vector<unsigned>& hideSegmentList)
{
	uint32_t nType;
	GetSegmentProperty(pTexCoordArray, nPointIndex, nType, false);
	return hideSegmentList.end() !=
		   std::find(hideSegmentList.begin(), hideSegmentList.end(), nType);
}