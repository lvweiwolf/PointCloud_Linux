//stdafx.h
#include "CloudSegmentation.h"

namespace d3s {
	namespace pcs {

		CCloudProcess::CCloudProcess() : _start(0.0), _end(100.0) {}

		CCloudProcess::~CCloudProcess() {}

		void CCloudProcess::SetCloudInput(IPointCloudPtr cloud)
		{
			CHECK(cloud.valid());
			_cloud = cloud;
		}

		void CCloudProcess::SetIndices(const std::vector<PointId>& indices) { _indices = indices; }

		//void CCloudProcess::SetProgressCallback(IProgressPtr progress)
		//{
		//	_progressCallback = progress;
		//}

		//void CCloudProcess::SetProgressRange(double start, double end)
		//{
		//	_start = start;
		//	_end = end;

		//	CHECK(_start <= _end);
		//}

		//void CCloudProcess::Progress(double precent, const char* pszMessage)
		//{
		//	CHECK(_start <= _end);

		//	double dist = _end - _start;
		//	double pos = _start + dist * (precent / 100.0);
		//	CHECK(pos <= _end);

		//	_progressCallback->Progress(pos, pszMessage, nullptr);
		//}

		
	}
}