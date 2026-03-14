/////////////////////////////////////////////////////////////////////
// 文件名称：pylonReconstruct.h
// 功能描述：杆塔重建
// 创建标识：吕伟	2022/12/27
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once
#include "../core/pointTypes.h"
#include "../utils/logging.h"
#include <string>
namespace d3s {
	namespace pcs {

		class GenericMesh;

		struct PylonReconstructOptions
		{
			double decompose_head_len;
			double decompose_z_step;
			double decompose_x_step;
			double decompose_windows_size;
			double decompose_fillrate;
			double decompose_s1_ratio;
			double decompose_aspect_error;

			double matching_rms_threshold;
			double matching_bbox_threshold;
			int matching_num_registration;
			std::string matching_library_folder;

			void Print() const;
		};

		class PylonReconstruct
		{
		public:
			PylonReconstruct(const PylonReconstructOptions& options,
							 PointCloudViewPtr input,
							 GenericMesh* mesh);

			~PylonReconstruct();

			void Reconstruct();

		private:
			PylonReconstructOptions _options;
			PointCloudViewPtr _input;
			GenericMesh* _mesh;
		};
	}
}
