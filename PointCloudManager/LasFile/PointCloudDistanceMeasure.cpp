#include <include/PointCloudDistanceMeasure.h>
#include <include/Log.h>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>
// #include <pcl/segmentation/sac_segmentation.h>

/////////////////分析类型距离//////////////////////
// 计算最小距离的容器大小
const int INT_CALCULATE_LIST_SIZE = 1;
// 两位小数比较
const double DOUBLE_COMPARSION_TWO_DECIMAL_PLACES = 0.01;


pcl::PointCloud<pcl::PointXYZ>::Ptr samplingPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud,
													   double dSampleDis)
{
	pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
	voxelGrid.setLeafSize(dSampleDis, dSampleDis, dSampleDis);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pReferenceCloud(new pcl::PointCloud<pcl::PointXYZ>());
	voxelGrid.setInputCloud(Cloud);

	voxelGrid.filter(*pReferenceCloud);

	return pReferenceCloud;
}

CPointCloudDistanceMeasure::CPointCloudDistanceMeasure(void) {}

CPointCloudDistanceMeasure::~CPointCloudDistanceMeasure(void) {}

///////////////////////距离////////////////////////
double CPointCloudDistanceMeasure::GetMinDistance(osg::Vec3d& minOutContrastPoint,
												  osg::Vec3d& minOutReferencePoint,
												  EMeasureType eMeasureType,
												  const std::vector<osg::Vec3d>& vContrastPoints,
												  const std::vector<osg::Vec3d>& vReferencePoints)
{
	double dMinDistance = std::numeric_limits<double>::max();
	if (vContrastPoints.empty() || vReferencePoints.empty())
	{
		d3s::CLog::Warn(L"计算两份点云距离时有空的点云!");
		return dMinDistance;
	}
	switch (eMeasureType)
	{
	case CPointCloudDistanceMeasure::eHorizontalMeasure:
	{
		dMinDistance = GetMinHorizontalDistance(minOutContrastPoint,
												minOutReferencePoint,
												vContrastPoints,
												vReferencePoints);
		break;
	}
	case CPointCloudDistanceMeasure::eVerticalMeasure:
	{
		dMinDistance = GetMinVerticalDistance(minOutContrastPoint,
											  minOutReferencePoint,
											  vContrastPoints,
											  vReferencePoints);
		break;
	}
	case CPointCloudDistanceMeasure::eClearanceMeasure:
	{
		dMinDistance = GetMinClearanceDistance(minOutContrastPoint,
											   minOutReferencePoint,
											   vContrastPoints,
											   vReferencePoints);
		break;
	}
	default:
	{
		break;
	}
	}
	return dMinDistance;
}

void CPointCloudDistanceMeasure::GetEigenVectors(osg::Vec3d& outMajorPoint,
												 osg::Vec3d& outMiddlePoint,
												 osg::Vec3d& outMinorPoint,
												 const std::vector<osg::Vec3d>& vPoints)
{
	if (vPoints.empty())
	{
		d3s::CLog::Warn(L"GetEigenVectors 点集为空!");
		return;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pCloud->points.reserve(vPoints.size());
	for (auto& point : vPoints)
	{
		pCloud->points.emplace_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
	}
	pCloud->width = pCloud->points.size();
	pCloud->height = 1;
	Eigen::Vector3f majorVector, middleVector, minorVector;
	{
		Eigen::Vector4f centroid;
		Eigen::Matrix3f covariance;
		pcl::compute3DCentroid(*pCloud, centroid);
		pcl::computeCovarianceMatrixNormalized(*pCloud, centroid, covariance);
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance,
															  Eigen::ComputeEigenvectors);
		Eigen::Matrix3f eigenVectors = solver.eigenvectors();
		Eigen::Vector3f eigenValues = solver.eigenvalues();
		Eigen::Vector3f majorAxis;
		Eigen::Vector3f middleAxis;
		Eigen::Vector3f minorAxis;
		float fMajorValue = 0.0f;
		float fMiddleValue = 0.0f;
		float fMinorValue = 0.0f;
		{
			unsigned int nTemp = 0;
			unsigned int nMajorIndex = 0;
			unsigned int nMiddleIndex = 1;
			unsigned int nMinorIndex = 2;
			if (eigenValues.real()(nMajorIndex) < eigenValues.real()(nMiddleIndex))
			{
				nTemp = nMajorIndex;
				nMajorIndex = nMiddleIndex;
				nMiddleIndex = nTemp;
			}
			if (eigenValues.real()(nMajorIndex) < eigenValues.real()(nMinorIndex))
			{
				nTemp = nMajorIndex;
				nMajorIndex = nMinorIndex;
				nMinorIndex = nTemp;
			}
			if (eigenValues.real()(nMiddleIndex) < eigenValues.real()(nMinorIndex))
			{
				nTemp = nMinorIndex;
				nMinorIndex = nMiddleIndex;
				nMiddleIndex = nTemp;
			}
			fMajorValue = eigenValues.real()(nMajorIndex);
			fMiddleValue = eigenValues.real()(nMiddleIndex);
			fMinorValue = eigenValues.real()(nMinorIndex);
			majorAxis = eigenVectors.col(nMajorIndex).real();
			middleAxis = eigenVectors.col(nMiddleIndex).real();
			minorAxis = eigenVectors.col(nMinorIndex).real();
			majorAxis.normalize();
			middleAxis.normalize();
			minorAxis.normalize();
			float det = majorAxis.dot(middleAxis.cross(minorAxis));
			if (det <= 0.0f)
			{
				majorAxis(0) = -majorAxis(0);
				majorAxis(1) = -majorAxis(1);
				majorAxis(2) = -majorAxis(2);
			}
		}
		majorVector = majorAxis;
		middleVector = middleAxis;
		minorVector = minorAxis;
	}
	outMajorPoint = osg::Vec3d(majorVector[0], majorVector[1], majorVector[2]);
	outMiddlePoint = osg::Vec3d(middleVector[0], middleVector[1], middleVector[2]);
	outMinorPoint = osg::Vec3d(minorVector[0], minorVector[1], minorVector[2]);
}

double CPointCloudDistanceMeasure::QuickGetMinClearanceDistance(
	CPointCloudDistanceMeasure::MeasureResult& measureResult,
	const std::vector<osg::Vec3d>& vContrastPoints,
	const std::vector<osg::Vec3d>& vReferencePoints,
	double dReferenceThin,
	double dContrastThin)
{
	double dMinDistance = std::numeric_limits<double>::max();
	if (vContrastPoints.empty() || vReferencePoints.empty())
	{
		d3s::CLog::Warn(L"计算两份点云净空距离时有空的点云!");
		return dMinDistance;
	}
	// 将参考点集转换为pcl点进行抽稀
	pcl::PointCloud<pcl::PointXYZ>::Ptr pReferenceCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pReferenceCloud->points.reserve(vReferencePoints.size());
	for (auto& point : vReferencePoints)
	{
		pReferenceCloud->points.emplace_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr pThinReferenceCloud =
		samplingPointCloud(pReferenceCloud, dReferenceThin);

	// 将比较点集转换为pcl点进行抽稀
	pcl::PointCloud<pcl::PointXYZ>::Ptr pContrastPoint(new pcl::PointCloud<pcl::PointXYZ>());
	pContrastPoint->points.reserve(vContrastPoints.size());
	for (auto& point : vContrastPoints)
	{
		pContrastPoint->points.emplace_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr pThinContrastPoint =
		samplingPointCloud(pContrastPoint, dContrastThin);

	// 计算最近距离点
	pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
	kdTree.setInputCloud(pThinReferenceCloud);
	for (auto& searchPoint : pThinContrastPoint->points)
	{
		std::vector<int> vIndexs(INT_CALCULATE_LIST_SIZE);
		std::vector<float> vDistances(INT_CALCULATE_LIST_SIZE);
		if (kdTree.nearestKSearch(searchPoint, INT_CALCULATE_LIST_SIZE, vIndexs, vDistances) > 0)
		{
			double dDistance = sqrt(vDistances[0]);
			if (dMinDistance > dDistance)
			{
				dMinDistance = dDistance;
				pcl::PointXYZ contrastPoint = pThinReferenceCloud->at(vIndexs[0]);
				measureResult.minContrastPt =
					osg::Vec3d(searchPoint.x, searchPoint.y, searchPoint.z);
				measureResult.minReferencePt =
					osg::Vec3d(contrastPoint.x, contrastPoint.y, contrastPoint.z);
			}
		}
	}
	return dMinDistance;
}

/*------------------------------------------------------------------------距离------------------------------------------------------------------------*/
double CPointCloudDistanceMeasure::GetMinHorizontalDistance(
	osg::Vec3d& minOutContrastPoint,
	osg::Vec3d& minOutReferencePoint,
	const std::vector<osg::Vec3d>& vContrastPoints,
	const std::vector<osg::Vec3d>& vReferencePoints)
{
	double dMinHorizontalDistance = std::numeric_limits<double>::max();
	if (vContrastPoints.empty() || vReferencePoints.empty())
	{
		d3s::CLog::Warn(L"GetMinHorizontalDistance 点集含有空!");
		return dMinHorizontalDistance;
	}
	// 点云Z值相等时的净空距离就是水平距离
	std::vector<osg::Vec3d> vZeroZContrastPoints;
	GetZeroZPoints(vZeroZContrastPoints, vContrastPoints);
	std::vector<osg::Vec3d> vZeroZReferencePoints;
	GetZeroZPoints(vZeroZReferencePoints, vReferencePoints);
	osg::Vec3d minContrastZeroZPoint;
	osg::Vec3d minReferenceZeroZPoint;
	dMinHorizontalDistance = GetMinClearanceDistance(minContrastZeroZPoint,
													 minReferenceZeroZPoint,
													 vZeroZContrastPoints,
													 vZeroZReferencePoints);
	// 查找对应的最短连线对应节点
	for (auto& point : vContrastPoints)
	{
		if (osg::equivalent(minContrastZeroZPoint.x(),
							point.x(),
							DOUBLE_COMPARSION_TWO_DECIMAL_PLACES) &&
			osg::equivalent(minContrastZeroZPoint.y(),
							point.y(),
							DOUBLE_COMPARSION_TWO_DECIMAL_PLACES))
		{
			minOutContrastPoint = point;
			break;
		}
	}
	for (auto& point : vReferencePoints)
	{
		if (osg::equivalent(minReferenceZeroZPoint.x(),
							point.x(),
							DOUBLE_COMPARSION_TWO_DECIMAL_PLACES) &&
			osg::equivalent(minReferenceZeroZPoint.y(),
							point.y(),
							DOUBLE_COMPARSION_TWO_DECIMAL_PLACES))
		{
			minOutReferencePoint = point;
			break;
		}
	}
	return dMinHorizontalDistance;
}

double CPointCloudDistanceMeasure::GetMinVerticalDistance(
	osg::Vec3d& minOutContrastPoint,
	osg::Vec3d& minOutReferencePoint,
	const std::vector<osg::Vec3d>& vContrastPoints,
	const std::vector<osg::Vec3d>& vReferencePoints)
{
	double dMinVerticalDistance = std::numeric_limits<double>::max();
	if (vContrastPoints.empty() || vReferencePoints.empty())
	{
		d3s::CLog::Warn(L"计算两份点云垂直距离时有空的点云!");
		return dMinVerticalDistance;
	}
	// 求比较点云Z最小最大值对应的点
	osg::Vec3d minZContrastPoint;
	osg::Vec3d maxZContrastPoint;
	if (!GetMinMaxZPoint(minZContrastPoint, maxZContrastPoint, vContrastPoints))
	{
		d3s::CLog::Error(L"比较点云最小最大值对应的点失败!");
		return dMinVerticalDistance;
	}
	// 求参考点云Z最小最大值对应的点
	osg::Vec3d minZReferencePoint;
	osg::Vec3d maxZReferencePoint;
	if (!GetMinMaxZPoint(minZReferencePoint, maxZReferencePoint, vReferencePoints))
	{
		d3s::CLog::Error(L"参考点云最小最大值对应的点失败!");
		return dMinVerticalDistance;
	}
	// 计算最小距离和最近距离点
	if (minZContrastPoint.z() >= maxZReferencePoint.z())
	{
		dMinVerticalDistance = minZContrastPoint.z() - maxZReferencePoint.z();
		minOutContrastPoint = minZContrastPoint;
		minOutReferencePoint = maxZReferencePoint;
	}
	else if (minZContrastPoint.z() < maxZReferencePoint.z() &&
			 minZContrastPoint.z() >= minZReferencePoint.z())
	{
		minOutContrastPoint = minZContrastPoint;
		GetCloserZPoint(minOutReferencePoint, vReferencePoints, minZContrastPoint.z());
		dMinVerticalDistance = abs(minOutContrastPoint.z() - minOutReferencePoint.z());
	}
	else if (minZContrastPoint.z() < minZReferencePoint.z() &&
			 maxZContrastPoint.z() >= minZReferencePoint.z())
	{
		GetCloserZPoint(minOutContrastPoint, vContrastPoints, minZReferencePoint.z());
		minOutReferencePoint = minZReferencePoint;
		dMinVerticalDistance = abs(minOutContrastPoint.z() - minOutReferencePoint.z());
	}
	else if (maxZContrastPoint.z() < minZReferencePoint.z())
	{
		dMinVerticalDistance = minZReferencePoint.z() - maxZContrastPoint.z();
		minOutContrastPoint = maxZContrastPoint;
		minOutReferencePoint = minZReferencePoint;
	}
	return dMinVerticalDistance;
}

double CPointCloudDistanceMeasure::GetMinClearanceDistance(
	osg::Vec3d& minOutContrastPoint,
	osg::Vec3d& minOutReferencePoint,
	const std::vector<osg::Vec3d>& vContrastPoints,
	const std::vector<osg::Vec3d>& vReferencePoints)
{
	double dMinDistance = std::numeric_limits<double>::max();
	if (vContrastPoints.empty() || vReferencePoints.empty())
	{
		d3s::CLog::Warn(L"计算两份点云净空距离时有空的点云!");
		return dMinDistance;
	}
	// 将参考点集转换为pcl点
	pcl::PointCloud<pcl::PointXYZ>::Ptr pReferenceCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pReferenceCloud->points.reserve(vReferencePoints.size());
	for (auto& point : vReferencePoints)
	{
		pReferenceCloud->points.emplace_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
	}
	// 计算最近距离点
	pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
	kdTree.setInputCloud(pReferenceCloud);
	for (auto& point : vContrastPoints)
	{
		std::vector<int> vIndexs(INT_CALCULATE_LIST_SIZE);
		std::vector<float> vDistances(INT_CALCULATE_LIST_SIZE);
		pcl::PointXYZ searchPoint(point.x(), point.y(), point.z());
		if (kdTree.nearestKSearch(searchPoint, INT_CALCULATE_LIST_SIZE, vIndexs, vDistances) > 0)
		{
			double dDistance = sqrt(vDistances[0]);
			if (dMinDistance > dDistance)
			{
				dMinDistance = dDistance;
				pcl::PointXYZ contrastPoint = pReferenceCloud->at(vIndexs[0]);
				minOutContrastPoint = osg::Vec3d(searchPoint.x, searchPoint.y, searchPoint.z);
				minOutReferencePoint =
					osg::Vec3d(contrastPoint.x, contrastPoint.y, contrastPoint.z);
			}
		}
	}
	return dMinDistance;
}

bool CPointCloudDistanceMeasure::GetMinMaxZPoint(osg::Vec3d& outMinZPoint,
												 osg::Vec3d& outMaxZPoint,
												 const std::vector<osg::Vec3d>& vPoints)
{
	if (vPoints.empty())
	{
		d3s::CLog::Warn(L"GetMinMaxZPoint 点云为空!");
		return false;
	}
	osg::Vec3d minPoint(0.0, 0.0, std::numeric_limits<double>::max());
	osg::Vec3d maxPoint(0.0, 0.0, -std::numeric_limits<double>::max());
	for (auto& point : vPoints)
	{
		double dZ = point.z();
		if (minPoint.z() > dZ)
		{
			minPoint = point;
		}
		if (maxPoint.z() < dZ)
		{
			maxPoint = point;
		}
	}
	outMinZPoint = minPoint;
	outMaxZPoint = maxPoint;
	return true;
}

bool CPointCloudDistanceMeasure::GetCloserZPoint(osg::Vec3d& outCloserZPoint,
												 const std::vector<osg::Vec3d>& vPoints,
												 double dZ)
{
	if (vPoints.empty())
	{
		d3s::CLog::Warn(L"GetCloserZPoint 点云为空!");
		return false;
	}
	outCloserZPoint = vPoints[0];
	for (auto& point : vPoints)
	{
		if (std::abs(outCloserZPoint.z() - dZ) > std::abs(point.z() - dZ))
		{
			outCloserZPoint = point;
		}
	}
	return true;
}

void CPointCloudDistanceMeasure::GetZeroZPoints(std::vector<osg::Vec3d>& vOutZeroZPoints,
												const std::vector<osg::Vec3d>& vPoints)
{
	int nPointSize = vPoints.size();
	if (vOutZeroZPoints.capacity() < nPointSize)
	{
		vOutZeroZPoints.reserve(nPointSize);
	}
	for (auto& point : vPoints)
	{
		vOutZeroZPoints.emplace_back(point.x(), point.y(), 0.0);
	}
}