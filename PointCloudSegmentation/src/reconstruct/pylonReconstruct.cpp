#include <src/reconstruct/pylonReconstruct.h>
#include <src/reconstruct/pylonDecompose.h>
#include <src/reconstruct/pylonMatching.h>
#include <src/core/pointTypes.h>
#include <src/utils/logging.h>

#include <osg/Matrix>

namespace d3s {
	namespace pcs {

		// PylonReconstructOptions
		//////////////////////////////////////////////////////////////////////////
		void PylonReconstructOptions::Print() const
		{
			PCS_INFO("decompose_head_len: %lf", decompose_head_len);
			PCS_INFO("decompose_z_step: %lf", decompose_z_step);
			PCS_INFO("decompose_x_step: %lf", decompose_x_step);
			PCS_INFO("decompose_windows_size: %lf", decompose_windows_size);
			PCS_INFO("decompose_fillrate: %lf", decompose_fillrate);
			PCS_INFO("decompose_s1_ratio: %lf", decompose_s1_ratio);
			PCS_INFO("decompose_aspect_error: %lf", decompose_aspect_error);
			PCS_INFO("matching_rms_threshold: %lf", matching_rms_threshold);
			PCS_INFO("matching_bbox_threshold: %lf", matching_bbox_threshold);
			PCS_INFO("matching_num_registration: %d", matching_num_registration);
			PCS_INFO("matching_library_folder: %s", matching_library_folder.c_str());
		}

		// PylonReconstruct
		//////////////////////////////////////////////////////////////////////////
		PylonReconstruct::PylonReconstruct(const PylonReconstructOptions& options,
										   PointCloudViewPtr input,
										   GenericMesh* mesh)
			: _options(options), _input(input), _mesh(mesh)
		{
		}

		PylonReconstruct::~PylonReconstruct() {}

		void PylonReconstruct::Reconstruct()
		{
			PylonDecomposeOptions options;
			options.head_length = _options.decompose_head_len;
			options.z_step = _options.decompose_z_step;
			options.x_step = _options.decompose_x_step;
			options.window_size = _options.decompose_windows_size;
			options.fillrate = _options.decompose_fillrate;
			options.s1_ratio = _options.decompose_s1_ratio;
			options.aspect_error = _options.decompose_aspect_error;

			PylonDecomposer decomposer(options, _mesh, _input, std::vector<int>());
			decomposer.Decompose();
			/*RenderAddMatrix("pointcloud0", decomposer.GetTransform());
			UpdateAxisPlot();*/

			osg::Matrix transform = decomposer.GetTransform();
			std::vector<int> headIndices = decomposer.GetPylonHeadIndices();
			std::vector<int> bodyIndices = decomposer.GetPylonBodyIndices();
			std::vector<int> footIndices = decomposer.GetPylonFootIndices();

			PointCloudView<PointPCLH> pylonHead, pylonBody, pylonFoot;
			pylonHead.resize(headIndices.size());
			pylonBody.resize(bodyIndices.size());
			pylonFoot.resize(footIndices.size());
			auto offset = _input->offset_xyz;

			transform = osg::Matrix::translate(-offset) * transform;

			{
				// 初始化杆塔点云
				for (size_t i = 0; i < headIndices.size(); ++i)
				{
					const auto& p = _input->points[headIndices[i]];
					pylonHead.points[i].x = p.x + offset.x();
					pylonHead.points[i].y = p.y + offset.y();
					pylonHead.points[i].z = p.z + offset.z();

					pylonHead.bbox.expandBy(pylonHead.points[i].x,
											pylonHead.points[i].y,
											pylonHead.points[i].z);
				}

				for (size_t i = 0; i < bodyIndices.size(); ++i)
				{
					const auto& p = _input->points[bodyIndices[i]];
					pylonBody.points[i].x = p.x + offset.x();
					pylonBody.points[i].y = p.y + offset.y();
					pylonBody.points[i].z = p.z + offset.z();

					pylonBody.bbox.expandBy(pylonBody.points[i].x,
											pylonBody.points[i].y,
											pylonBody.points[i].z);
				}

				for (size_t i = 0; i < footIndices.size(); ++i)
				{
					const auto& p = _input->points[footIndices[i]];
					pylonFoot.points[i].x = p.x + offset.x();
					pylonFoot.points[i].y = p.y + offset.y();
					pylonFoot.points[i].z = p.z + offset.z();

					pylonFoot.bbox.expandBy(pylonFoot.points[i].x,
											pylonFoot.points[i].y,
											pylonFoot.points[i].z);
				}
			}

			PylonMatchingOptions recon_options;
			recon_options.rms_threshold = _options.matching_rms_threshold;
			recon_options.bbox_threshold = _options.matching_bbox_threshold;
			recon_options.num_registration = _options.matching_num_registration;
			std::string libraryFolder = _options.matching_library_folder;

			{
				PylonMatching match(recon_options, &pylonHead, transform);
				match.SetMatchingLibrary(libraryFolder + R"(杆塔组件库\塔头\)");
				match.Matching();
			}
			{
				PylonMatching match(recon_options, &pylonBody, transform);
				match.SetMatchingLibrary(libraryFolder + R"(杆塔组件库\塔体\)");
				match.Matching();
			}
			{
				PylonMatching match(recon_options, &pylonFoot, transform);
				match.SetMatchingLibrary(libraryFolder + R"(杆塔组件库\塔脚\)");
				match.Matching();
			}
		}

	}
}