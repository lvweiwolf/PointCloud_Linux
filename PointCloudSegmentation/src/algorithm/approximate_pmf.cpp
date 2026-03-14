#include <src/algorithm/approximate_pmf.hpp>
#include <src/core/pointTypes.h>

#include <pcl/impl/instantiate.hpp>
// #include <pcl/impl/pcl_base.hpp>
// #include <pcl/point_types.h>

#define PCL_ONLY_CORE_POINT_TYPES

#ifdef PCL_ONLY_CORE_POINT_TYPES
PCL_INSTANTIATE(
	ApproximatePMF,
	(pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB)(d3s::pcs::PointPCLH))
#else
PCL_INSTANTIATE(ApproximatePMF, PCL_XYZ_POINT_TYPES)
#endif