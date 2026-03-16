/*
 * @Author: wujianfeng
 * @Date: 2026-02-25 10:14:22
 * @LastEditors: wujianfeng
 * @LastEditTime: 2026-03-06 11:49:00
 * @FilePath: \undefinedg:\PointCloud_Linux\PointCloudManager\Segment\PointCloudManagerDefine.h
 * @Description: 
 */
//*****************************************************
//    
//    @copyright      	三维技术部
//    @version        	v1.0
//    @file           	PointCloudManagerDefine.H
//    @author         	LC
//    @data           	2022/5/11 13:38
//    @brief          	点云管理模块定义文件（以下数据结构仅供插件内使用）
//*****************************************************
#pragma once
#include "PointCloudManagerDef.h"

namespace pc
{
	namespace data
	{
		struct PointInfo
		{
			PointInfo(void)
				: dX(0.0), dY(0.0), dZ(0.0), nTreeId(0), pointClassify(OTHER_CLASSIFY)
			{}
			PointInfo(double dX, double dY, double dZ, unsigned int nTreeId, char pointClassify)
				:dX(dX), dY(dY), dZ(dZ), nTreeId(nTreeId), pointClassify(pointClassify)
			{

			}

			double			dX;				// x坐标
			double			dY;				// y坐标
			double			dZ;				// z坐标
			unsigned int	nTreeId;		// 所属树 id
			char			pointClassify;	// 点类型
		};

		static const int QUERY_POINTS_LIMIT_MAX_NUM = 40000000;
		// 两位小数比较
		static	const	double		INT_COMPARSION_OF_TWO_DECIMAL_PLACES = 0.01;

		static const int JINGXI_LEVEL = 3;	// 精细层级从0开始，第3级

		// 点云查询异常信息枚举
		enum EPointCloudQueryErrorType
		{
			eNoError = 0,
			eOverMaxLimit
		};

		BEGIN_ENUM_CONVERSION(EPointCloudQueryErrorType)
		{
			{ eNoError, NULL },
			{ eOverMaxLimit, _T("查询的点云数量超过最大限制") }
		}
		END_ENUM_CONVERSION(EPointCloudQueryErrorType);

	}
}

enum ESegmentType
{
	eAutoSegment,
	eTreeIndividual
};

//namespace d3s {
//	namespace pcs {
//		enum class SegmentationType
//		{
//			GROUND,						// 地面提取
//			POWERCORRIDORS,				// 电力线元素(铁塔、电力线)分类
//			ENVIROMENTS,				// 环境元素(植被、建筑)分类
//			TREEINDIVIDUAL,				// 单木分割
//			TREESPECIESIDENTIFY,		// 树种识别
//			CROPS,						// 农作物分类
//			DANGER						// 危险区域分割
//		};
//	}
//}