//*****************************************************
//
//    @copyright      激光点云组
//    @version        v1.0
//    @file         PointCloudCommomTool.h
//    @author         AM
//    @data          2022/4.15
//    @brief		 点云管理公共工具
//*****************************************************
#ifndef POINTCLOUDTOOLKIT_H_
#define POINTCLOUDTOOLKIT_H_

#include <include/PointCloudManagerExport.h>
#include <include/cstring.h>
#include <include/Log.h>

#include <osg/BoundingBox>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Matrix>
#include <osg/Vec3d>
#include <osg/Vec4>

#include <vector>

class POINTCLOUDMANAGER_EXPORT CPointCloudToolkit
{
public:
	CPointCloudToolkit(void);
	~CPointCloudToolkit(void);

public:
	/* 函数介绍:  点在多边形内
	 *  输入参数:	std::vector<osg::Vec3d >& screenPointVec选择的屏幕点
	 *  输出参数:
	 *  返回值:     void
	 */
	static bool PtInPolygon(const osg::Vec3d& tagPt, const std::vector<osg::Vec3d>& screenPointVec);

	/**
	 *  @brief    判断平面多边形是否与模型的交集关系(是否在模型内)
	 *
	 *  @param    std::vector<osg::Vec3d> pointVec
	 *  @param    osg::Matrix matrixVPW
	 *  @param    std::vector<osg::Vec3d> selectScreenPointVec
	 *  @return   bool
	 */
	static bool JudgePolygonInModel(const osg::BoundingBox& boudingBox,
									const osg::Matrix& vpwMatrix,
									const std::vector<osg::Vec3d>& selectScreenPointVec);

	/**
	 *  @brief    判断平面多边形与多边形的交集关系(是否在模型内)
	 *
	 *  @param    std::vector<osg::Vec3d> pointVec
	 *  @param    osg::Matrix matrixVPW
	 *  @param    std::vector<osg::Vec3d> selectScreenPointVec
	 *  @return   bool
	 */
	static bool JudgePolygonInPolygon(const std::vector<osg::Vec3d>& pointVec,
									  const osg::Matrix& vpwMatrix,
									  const std::vector<osg::Vec3d>& selectScreenPointVec);
	/**
	 * 读取节点
	 * @param [in] fileName 文件名称
	 * @return
	 */
	static osg::ref_ptr<osg::Node> ReadNode(const std::string& pointFilePath,
											const std::string& texFileName);

	// 节点写入类型
	enum EWriteType
	{
		ePointData = 1 << 0,  // 写入点及颜色信息
		eTexCoord = 1 << 1,	  // 写入纹理信息
		eAllData = 0x7FFFFFFF // 写入所有信息
	};
	/**
	 * 写入节点（节点被拆分成 点信息+纹理数组 的方式写入）
	 * @param [in] node	需要写入文件的节点
	 * @param [in] fileName	写入的文件名称
	 * @param [in] eType 写入的类型
	 * @return
	 */
	static bool WriteNode(osg::ref_ptr<osg::Node> pNode,
						  const std::string& fileName,
						  EWriteType eType);

	/**
	 * 创建包围盒对应图元
	 * @param [in] box
	 * @param [in] color
	 * @return
	 **/
	static osg::ref_ptr<osg::Geode> BuildBoundingBoxGeode(osg::BoundingBox& box, osg::Vec4& color);

	/**
	 * 获取新的page文件名称
	 * @param [in] strDatabasePath	database路径
	 * @param [in] strFileName		文件名称
	 * @return
	 */
	static CString GetNewPageFileName(CString strDatabasePath, const CString& strFileName);
};

// 遍历所有Geometry
class CGeometryNodeVisitor : public osg::NodeVisitor
{
public:
	CGeometryNodeVisitor() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}
	~CGeometryNodeVisitor() {}

	virtual void apply(osg::Geometry& geometry) override
	{
		_vecGeo.emplace_back(&geometry);
		osg::NodeVisitor::apply(geometry);
	}

	std::vector<osg::ref_ptr<osg::Geometry>> _vecGeo;
};

#endif // POINTCLOUDTOOLKIT_H_
