#include <src/reconstruct/pylonMatching.h>
#include <src/core/api.h>
#include <src/core/private/filters.h>
#include <src/core/private/statistics.h>
#include <src/core/private/cloudProcess.h>
#include <src/utils/misc.h>
#include <src/utils/logging.h>
#include <src/utils/stringutil.h>
#include <src/plot/plotHandle.h>

#include <osg/MatrixTransform>
#include <osgDB/ReadFile>

// #define RENDER_PYLON_MODEL

namespace d3s {
	namespace pcs {

		PylonMatching::PylonMatching(const PylonMatchingOptions& options,
									 PointCloudViewPtr input,
									 osg::Matrix transform)
			: _options(options), _input(input), _transform(transform)
		{
		}

		PylonMatching::~PylonMatching() {}

		void PylonMatching::SetMatchingLibrary(const std::string& libpath) { _library = libpath; }

		std::vector<std::string> PylonMatching::GetMatchingResult() const { return _result; }

		void PylonMatching::Matching()
		{
			int numberOfPoints = std::max(500, _options.num_registration);
			double rmsThreshold = _options.rms_threshold;

			// 计算对齐到X轴后的包围框大小
			auto alignedBBox = ComputeAlignedBBox();

			PointCloudView<PointPCLH> inputSampled;
			cloudRandomSampling(*_input, numberOfPoints, inputSampled);

			double best_rms = DBL_MAX;
			std::string best_filename;
			osg::Matrix transform;

			std::vector<std::pair<std::string, double>> rank;
			std::vector<std::string> libFiles = GetFileList(_library);

			for (size_t i = 0; i < libFiles.size(); ++i)
			{
				const auto& filepath = libFiles.at(i);
				std::string filename, ext;
				SplitFileExtension(filepath, &filename, &ext);
				StringToLower(&ext);

				if (ext != ".las")
					continue;

				PointCloudView<PointPCLH> cloud;
				if (!readPointCloud(filepath, cloud))
					continue;

				// 还原点云原始坐标
				{
					auto offset = cloud.offset_xyz;

					for (size_t i = 0; i < cloud.points.size(); ++i)
					{
						auto& p = cloud.points[i];
						p.x += offset.x();
						p.y += offset.y();
						p.z += offset.z();
					}

					cloud.offset_xyz = osg::Vec3d();
				}

				// 计算包围盒
				computeMinMax3D(cloud, cloud.bbox);

				if (!Similarity(cloud.bbox, alignedBBox))
					continue;

				PointCloudView<PointPCLH> cloudSampled;
				cloudRandomSampling(cloud, numberOfPoints, cloudSampled);

				osg::Matrix matrix;
				double rms = RegistrationRMS(cloudSampled, inputSampled, matrix);

				if (rms < rmsThreshold)
				{
					rank.push_back(std::make_pair(filename, rms));
				}

				if (rms < best_rms)
				{
					best_rms = rms;
					best_filename = filename;
					transform = matrix;
				}

				PCS_INFO("[PylonReconstruction] %s RMS: %lf",
						 GetPathBaseName(filename).c_str(),
						 rms);
			}

			std::sort(rank.begin(), rank.end(), [](const auto& lhs, const auto& rhs) {
				return lhs.second < rhs.second;
			});

			_result.clear();

			for (const auto& r : rank)
				_result.push_back(r.first);

			std::string best_filepath = best_filename + ".stl";
			osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(best_filepath);
			osg::ref_ptr<osg::MatrixTransform> transformNode = new osg::MatrixTransform();
			transformNode->setMatrix(transform);
			transformNode->addChild(model);

#ifdef RENDER_PYLON_MODEL
			PlotHandler::GetInst()->RemoveNode("pointcloud0");
			PlotHandler::GetInst()->RemoveNode(best_filepath);
			PlotHandler::GetInst()->AddNode(best_filepath, transformNode);
#endif
		}

		osg::BoundingBox PylonMatching::ComputeAlignedBBox()
		{
			osg::BoundingBox bbox;

			for (size_t i = 0; i < _input->points.size(); ++i)
			{
				const auto& p = _input->points[i];
				osg::Vec3d pt = osg::Vec3d(p.x, p.y, p.z) * _transform;
				bbox.expandBy(pt);
			}

			return bbox;
		}

		bool PylonMatching::Similarity(const osg::BoundingBox& lhs, const osg::BoundingBox& rhs)
		{
			double threshold = _options.bbox_threshold;

			double W0 = lhs.xMax() - lhs.xMin();
			double H0 = lhs.yMax() - lhs.yMin();
			double T0 = lhs.zMax() - lhs.zMin();

			double W1 = rhs.xMax() - rhs.xMin();
			double H1 = rhs.yMax() - rhs.yMin();
			double T1 = rhs.zMax() - rhs.zMin();


			return fabs(W1 - W0) < threshold && fabs(H1 - H0) < threshold &&
				   fabs(T1 - T0) < threshold;
		}

		double PylonMatching::RegistrationRMS(PointCloudView<PointPCLH>& source,
											  PointCloudView<PointPCLH>& target,
											  osg::Matrix& transform)
		{
			osg::BoundingBox bbox;
			computeMinMax3D(source, bbox);

			double rms = DBL_MAX;
			osg::Vec3d trans = target.bbox.center() - bbox.center();
			source.offset_xyz = trans;

			Registration(source, target, transform, rms);

			transform = osg::Matrix::translate(source.offset_xyz) * transform;

			return rms;
		}

	}
}