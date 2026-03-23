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

#include <LasFile/PointCloudToolDefine.h>

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
	/**
	 * 判断点是否在线附近
	 * @param [in] tagPt		点
	 * @param [in] lineSt	线起点
	 * @param [in] lineEd	线终点
	 * @param [in] dTol		容差
	 * @return
	 */
	static bool NearLineSegment(const osg::Vec3d& tagPt,
								const osg::Vec3d& lineSt,
								const osg::Vec3d& lineEd,
								double dTol = 1);
	
	/**
	 * 计算点在多边形的相对位置：0(多边形外)，1(多边形内)，2(边)
	 * @param [in] tagPt			点
	 * @param [in] polygonPoint	多边形
	 * @return
	 */
	static char PtRelativePolygonPosition(const osg::Vec3d& tagPt,
										  const std::vector<osg::Vec3d>& polygonPoint);

	
	/*
	 * 函数介绍：获取包围盒范围（X,Y的最小值和最大值）
	 * 输入参数：const std::vector<osg::Vec3d> &vBoundingBoxs		包围盒
	 * 输出参数：double &dMinX				X最小值
	 *			double &dMaxX				X最大值
	 *			double &dMinY				Y最小值
	 *			double &dMaxY				Y最大值
	 * 返回值  ：void
	 */
	static void GetBoundingBoxRange(const std::vector<osg::Vec3d>& vBoundingBoxs,
									double& dMinX,
									double& dMaxX,
									double& dMinY,
									double& dMaxY);

	/*
	 * 函数介绍：获取包围盒范围（X,Y的最小值和最大值）
	 * 输入参数：const std::vector<osg::Vec3d> &vBoundingBoxs		包围盒
	 * 输出参数：int &nMinX				X最小值
	 *			int &nMaxX				X最大值
	 *			int &nMinY				Y最小值
	 *			int &nMaxY				Y最大值
	 * 返回值  ：void
	 */
	static void GetBoundingBoxRange(const std::vector<osg::Vec3d>& vBoundingBoxs,
									int& nMinX,
									int& nMaxX,
									int& nMinY,
									int& nMaxY);
	
	/**
	 * 计算多边形拟合区域
	 * @param [in] polygonPt 多边形点
	 * @param [in] nMaxX		拟合区域最大x值
	 * @param [in] nMinX		拟合区域最小x值
	 * @param [in] nMaxY		拟合区域最大y值
	 * @param [in] nMinY		拟合区域最小y值
	 * @return
	 */
	static std::vector<bool> CountPolygonMathValue(const std::vector<osg::Vec3d>& polygonPt,
												   unsigned nMaxX,
												   unsigned nMinX,
												   unsigned nMaxY,
												   unsigned nMinY);

	/**
	 * 计算拟合多边形
	 * @param [in] coarsePolygonPt	粗糙多边形点
	 * @param [in] coarseVpwMat		粗糙屏幕vpw矩阵
	 * @param [in] fineVpwMat		精细屏幕矩阵
	 * @param [out] coarsePolygon	粗糙多边形拟合值：0(多边形外)，1(多边形内)，2(边)
	 * @param [out] finePolygonMap	精细多边形拟合值：<粗糙多边形索引， 拟合多边形>
	 * @return
	 */
	static void CountPolygonMathValue(const std::vector<osg::Vec3d>& coarsePolygonPt,
									  const osg::Matrix& coarseVpwMat,
									  const osg::Matrix& fineVpwMat,
									  std::vector<char>& coarsePolygon,
									  pc::data::FinePolygonMap& finePolygonMap);

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

private:
	/**
	 * 点向右的射线是否与线段相交
	 * @param [in] tagPt
	 * @param [in] lineStPt
	 * @param [in] lineEdPt
	 * @return
	 */
	static bool PtRightRadialIntersectLine(const osg::Vec3d& tagPt,
										   const osg::Vec3d& lineStPt,
										   const osg::Vec3d& lineEdPt);
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
