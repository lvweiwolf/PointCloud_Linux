/////////////////////////////////////////////////////////////////////
// 文件名称：pointReg.h
// 功能描述：自定义点类型注册到PCL
// 创建标识：吕伟	2022/4/10
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef POINT_TYPES_INL_
#define POINT_TYPES_INL_

#include <src/core/pointTypes.h>

#include <pcl/point_traits.h>
#include <pcl/register_point_struct.h>

#include <boost/mpl/identity.hpp>
#include <boost/mpl/vector.hpp>

POINT_CLOUD_REGISTER_POINT_STRUCT (d3s::pcs::_PointXYZ_RGB_L_HAG,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, w, w)
	(uint32_t, rgba, rgba)
	(uint32_t, label, label)
	(float, hag, hag)
)

//POINT_CLOUD_REGISTER_POINT_STRUCT (d3s::pcs::PointPCLH,
//	(float, x, x)
//	(float, y, y)
//	(float, z, z)
//	(float, w, w)
//	(uint32_t, rgba, rgba)
//	(uint32_t, label, label)
//	(float, hag, hag)
//)

POINT_CLOUD_REGISTER_POINT_WRAPPER(d3s::pcs::PointPCLH, d3s::pcs::_PointXYZ_RGB_L_HAG)

#endif // POINT_TYPES_INL_