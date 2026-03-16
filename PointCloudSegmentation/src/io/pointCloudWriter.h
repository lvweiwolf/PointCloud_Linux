/////////////////////////////////////////////////////////////////////
// 文件名称：pointCloudWriter.h
// 功能描述：点云写入类
// 创建标识：吕伟	2022/6/13
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef POINT_CLOUD_WRITER_H_
#define POINT_CLOUD_WRITER_H_

#include <src/core/pointTypes.hpp>

namespace d3s {
	namespace pcs {
		namespace io {

			class PointCloudWriter
			{
			public:
				PointCloudWriter(const std::string& filename);

				~PointCloudWriter();

				bool Write(const PointCloudView<PointPCLH>& pcv);

				bool Write(const PointCloudView<PointPCLH>& pcv, const std::vector<int>& indices);

			private:

				bool WriteLAS(const PointCloudView<PointPCLH>& pcv,
							  const std::vector<int>& indices);

				
				bool WritePCD(const PointCloudView<PointPCLH>& pcv,
							  const std::vector<int>& indices);

				std::string _filename;
			};


		}
	}
}

#endif // GEO_PROJECTION_CONVERTER_H_
