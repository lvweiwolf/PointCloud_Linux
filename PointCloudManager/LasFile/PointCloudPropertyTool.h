//////////////////////////////////////////////////////////////////////
// 文件名称：PointCloudPropertyTool.h
// 功能描述：点云属性工具
// 创建标识：李成 2022/10/14
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef POINTCLOUDPROPERTYTOOL_H_
#define POINTCLOUDPROPERTYTOOL_H_
#include <include/PointCloudManagerExport.h>

#include <osg/Array>
#include <osg/Geometry>
#include <osg/Vec2>

#include <vector>
#include <cstdint>


//											             |显隐|  |障碍|  |高亮|      |类型|
// 属性存储位说明：TexCoord.x = 0000|0000|0000|0000|00000    0       0       0      0000|0000
//											| 簇ID |
// 属性存储位说明：TexCoord.y = 0000|0000|0000|0000|000000000000|0000
// texCoord整数范围均为23位，GLSL内使用float类型获取纹理值，高9位为符号、指数位


class POINTCLOUDMANAGER_EXPORT CPointCloudPropertyTool
{
public:
	/**
	 * 获取点云属性默认值（分类0、高亮false、显隐true、障碍false）
	 * @return
	 */
	static osg::Vec2 GetDefaultProperty();

	/**
	 * 设置、获取分类属性
	 * @param [in] geometry			图元
	 * @param [in] nPointIndex		点索引
	 * @param [in] nType				分类类型
	 * @param [in] pTexCoordArray	纹理数组
	 * @param [in] bHideLimit		是否受隐藏限制
	 * @param [in] hideSegmentList	分类隐藏列表
	 * @return
	 */
	static bool SetSegmentProperty(
		osg::Geometry& geometry,
		const size_t nPointIndex,
		const uint32_t nType,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	static bool SetSegmentProperty(
		osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
		const size_t nPointIndex,
		const uint32_t nType,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	static bool GetSegmentProperty(
		const osg::Geometry& geometry,
		const size_t nPointIndex,
		uint32_t& nType,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	static bool GetSegmentProperty(
		const osg::Vec2Array* pTexCoordArray,
		const size_t& nPointIndex,
		uint32_t& nType,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	/**
	 * 设置分类值
	 * @param [in] property
	 * @param [in] nType 分类值
	 * @return
	 */
	static bool SetSegmentProperty(osg::Vec2& property, const uint32_t nType);

	/**
	 * 设置、获取高亮属性
	 * @param [in] geometry			图元
	 * @param [in] nPointIndex		点索引
	 * @param [in] bool				是否高亮
	 * @param [in] pTexCoordArray	纹理数组
	 * @param [in] bHideLimit		是否受隐藏限制
	 * @param [in] hideSegmentList	分类隐藏列表
	 * @return
	 */
	static bool SetHighlightProperty(
		osg::Geometry& geometry,
		const size_t nPointIndex,
		const bool bHighlight,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	static bool SetHighlightProperty(
		osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
		const size_t nPointIndex,
		const bool bHighlight,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	static bool GetHighlightProperty(
		const osg::Geometry& geometry,
		const size_t nPointIndex,
		bool& bHighlight,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	static bool GetHighlightProperty(
		const osg::Vec2Array* pTexCoordArray,
		const size_t nPointIndex,
		bool& bHighlight,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});

	/**
	 * 设置、获取障碍属性
	 * @param [in] geometry			图元
	 * @param [in] nPointIndex		点索引
	 * @param [in] bool				是否表示障碍
	 * @param [in] pTexCoordArray	纹理数组
	 * @param [in] bHideLimit		是否受隐藏限制
	 * @param [in] hideSegmentList	分类隐藏列表
	 * @return
	 */
	static bool SetObstacleProperty(
		osg::Geometry& geometry,
		const size_t nPointIndex,
		const bool bObstacle,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	static bool SetObstacleProperty(
		osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
		const size_t nPointIndex,
		const bool bObstacle,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	static bool GetObstacleProperty(
		const osg::Geometry& geometry,
		const size_t nPointIndex,
		bool& bObstacle,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	static bool GetObstacleProperty(
		const osg::Vec2Array* pTexCoordArray,
		const size_t nPointIndex,
		bool& bObstacle,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});

	/**
	 * 设置、获取显隐
	 * @param [in] geometry			图元
	 * @param [in] nPointIndex		点索引
	 * @param [in] bool				是否显示
	 * @param [in] pTexCoordArray	纹理数组
	 * @param [in] bHideLimit		是否受隐藏限制
	 * @param [in] hideSegmentList	分类隐藏列表
	 * @return
	 */
	static bool SetDisplayEnableProperty(
		osg::Geometry& geometry,
		const size_t nPointIndex,
		const bool bDisplayEnable,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	static bool SetDisplayEnableProperty(
		osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
		const size_t nPointIndex,
		const bool bDisplayEnable,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	static bool GetDisplayEnableProperty(
		const osg::Geometry& geometry,
		const size_t nPointIndex,
		bool& bDisplayEnable,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	static bool GetDisplayEnableProperty(
		const osg::Vec2Array* pTexCoordArray,
		const size_t nPointIndex,
		bool& bDisplayEnable,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});

	/**
	 * 设置簇ID
	 * @param [in] geometry			图元
	 * @param [in] nPointIndex		点索引
	 * @param [in] nClusterID		簇ID
	 * @param [in] pTexCoordArray	纹理数组
	 * @param [in] bHideLimit		是否受隐藏限制
	 * @param [in] hideSegmentList	分类隐藏列表
	 * @return
	 */
	static bool SetClusterProperty(
		osg::Geometry& geometry,
		const size_t nPointIndex,
		const int nClusterID,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	static bool SetClusterProperty(
		osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
		const size_t nPointIndex,
		const int nClusterID,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	static bool GetClusterProperty(
		const osg::Geometry& geometry,
		const size_t nPointIndex,
		int& nClusterID,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});
	static bool GetClusterProperty(
		const osg::Vec2Array* pTexCoordArray,
		const size_t nPointIndex,
		int& nClusterID,
		bool bHideLimit = true,
		const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{});

	/**
	 * 判断是否隐藏
	 * @param [in] nPointIndex		点索引
	 * @param [in] pTexCoordArray	纹理数组
	 * @param [in] hideSegmentList	隐藏分类列表
	 * @return
	 */
	static bool IsHide(const osg::Vec2Array* pTexCoordArray,
					   const size_t nPointIndex,
					   const std::vector<unsigned>& hideSegmentList);

protected:
	/**
	 * 获取bool属性
	 * @param [in] pTexCoordArray	纹理数组
	 * @param [in] nPointIndex		点索引
	 * @param [in] bHideLimit		是否受隐藏限制
	 * @param [in] hideSegmentList	分类隐藏列表
	 * @param [out] bValue			属性值
	 * @param [in] nBit				字节位
	 */
	static bool GetBoolProperty(const osg::Vec2Array* pTexCoordArray,
								const size_t nPointIndex,
								bool bHideLimit,
								const std::vector<unsigned>& hideSegmentList,
								bool& bValue,
								const unsigned int nBit);

	/**
	 * 设置bool属性
	 * @param [in] pTexCoordArray	纹理数组
	 * @param [in] nPointIndex		点索引
	 * @param [in] bHideLimit		是否受隐藏限制
	 * @param [in] hideSegmentList	分类隐藏列表
	 * @param [in] bValue			属性值
	 * @param [in] nBit				字节位
	 * @return
	 */
	static bool SetBoolProperty(osg::ref_ptr<osg::Vec2Array> pTexCoordArray,
								const size_t nPointIndex,
								bool bHideLimit,
								const std::vector<unsigned>& hideSegmentList,
								bool bValue,
								const unsigned int nBit);

	/**
	 * 设置获取字节位
	 * @param [in] nOperatorNum			操作值
	 * @param [in] nBit					字节位
	 * @param [in] bSetValue、bGetValue	设置获取的值
	 * @return 设置后的
	 */
	static void SetBit(int& nOperatorNum, const unsigned int nBit, const bool bSetValue);
	static void GetBit(const int nOperatorNum, const unsigned int nBit, bool& bGetValue);

	/**
	 * 判断分类是否隐藏
	 * @param [in] nPointIndex		点索引
	 * @param [in] pTexCoordArray	纹理数组
	 * @param [in] hideSegmentList	隐藏分类列表
	 * @return
	 */
	static bool IsHideSegment(const osg::Vec2Array* pTexCoordArray,
							  const size_t nPointIndex,
							  const std::vector<unsigned>& hideSegmentList);
};

#endif // POINTCLOUDPROPERTYTOOL_H_
