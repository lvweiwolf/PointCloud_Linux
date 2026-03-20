#include <src/reconstruct/pylonDecompose.h>
#include <src/core/private/statistics.h>
#include <src/core/private/meshProcess.h>
#include <src/segmentation/pylonCommon.h>
#include <src/algorithm/math.h>
#include <src/plot/plotHandle.h>
#include <src/utils/logging.h>

#include <numeric>
#include <set>

// #define RENDER_KEY_POSITIONS
// #define WRITE_LAS

extern std::string DebugDirectory;

namespace d3s {
	namespace pcs {

		// PylonDecomposer
		//////////////////////////////////////////////////////////////////////////
		PylonDecomposer::PylonDecomposer(const PylonDecomposeOptions& options,
										 GenericMesh* mesh,
										 PointCloudViewPtr input,
										 const std::vector<int>& indices)
			: _options(options), _mesh(mesh), _input(input), _indices(indices)
		{
			if (_indices.empty())
			{
				_indices.resize(_input->points.size());
				std::iota(_indices.begin(), _indices.end(), 0);
			}
		}

		PylonDecomposer::~PylonDecomposer() {}

		const osg::Matrix& PylonDecomposer::GetTransform() const { return _transform; }

		std::vector<int> PylonDecomposer::GetPylonHeadIndices() const { return _pylonHeadIndices; }

		std::vector<int> PylonDecomposer::GetPylonBodyIndices() const { return _pylonBodyIndices; }

		std::vector<int> PylonDecomposer::GetPylonFootIndices() const { return _pylonFootIndices; }

		void PylonDecomposer::Decompose()
		{
			// 获得杆塔塔头部分点云，以便更准确地重定向
			double towerHeadLength = _options.head_length;
			double zStep = _options.z_step;
			double windowSize = _options.window_size;
			double s1Ratio = _options.s1_ratio;

			// 边界范围
			osg::BoundingBox bbox;
			computeMinMax3D(*_input, _indices, bbox);

			// 获得杆塔塔头点云
			std::vector<int> headIndices = GetPylonHead(bbox, towerHeadLength);

			// 重定向杆塔点云的方向
			int unused = -1;
			osg::Vec3d dir;
			GetHeadDirection(_input, headIndices, 2.0, dir, unused);

			dir.normalize();
			osg::Vec3d trans(-bbox.center().x(), -bbox.center().y(), 0);
			_transform =
				osg::Matrix::translate(trans) * osg::Matrix::rotate(dir, osg::Vec3d(1, 0, 0));

			const double* data = _transform.ptr();

			PCS_INFO("[PylonDecomposer] Transform matrix:"
					 "\n			    %lf, %lf, %lf, %lf"
					 "\n			    %lf, %lf, %lf, %lf"
					 "\n			    %lf, %lf, %lf, %lf"
					 "\n			    %lf, %lf, %lf, %lf",
					 data[0],
					 data[1],
					 data[2],
					 data[3],
					 data[4],
					 data[5],
					 data[6],
					 data[7],
					 data[8],
					 data[9],
					 data[10],
					 data[11],
					 data[12],
					 data[13],
					 data[14],
					 data[15]);

			osg::Vec3d minPt = _transform.preMult(bbox._min);
			osg::Vec3d maxPt = _transform.preMult(bbox._max);
			osg::Matrix inverse = osg::Matrix::inverse(_transform);

			// 沿 Z 轴方向切片
			int slideSize = std::floor(windowSize / zStep);

			std::vector<std::vector<int>> slices;
			GetSlices(minPt.z(), maxPt.z(), zStep, slices);

			// 计算关键点位置
			std::vector<int> keyPositions;
			ComputeKeyPositions(slices, slideSize, keyPositions);

			// 计算填充率并过滤无效的关键位置
			KeyPositionsFiltering(slices, keyPositions);

			// 过于接近的 KP 聚类到一起作为一个 KP
			KeyPositionsClustering(keyPositions);

			// 处理塔脚
			if (keyPositions.size() > 2)
			{
				std::vector<double> values;
				for (int i = 0; i < keyPositions.size() - 1; ++i)
				{
					int m = keyPositions[i];
					int n = keyPositions[i + 1];
					double val = zStep * (n - m);

					values.push_back(val);
				}

				double valMean = Mean(values);
				double dist = (keyPositions[1] - keyPositions[0]) * zStep;

				if (dist < s1Ratio * valMean)
					keyPositions.erase(keyPositions.begin());
			}

			if (keyPositions.empty())
				return;

			std::vector<int> ksp;
			ComputeKeySegmentPositions(slices, keyPositions, ksp);

#ifdef RENDER_KEY_POSITIONS
			// 绘制关键位置边界框
			RenderKeyPositions(ksp, minPt, maxPt, inverse, zStep, _input);
#endif

			if (ksp.size() != 2)
			{
				PCS_WARN("[PylonDecomposer] 杆塔关键分割位置 KSP 提取失败.");
				return;
			}

			int S1 = ksp[0];
			int Sk = ksp[1];
			double Z1 = minPt.z() + zStep * S1;
			double Zk = minPt.z() + zStep * Sk;

			// 用 KSP 将杆塔分割成不同组件部位
			std::vector<int> pylonHead, pylonBody, pylonFoot;
			osg::BoundingBox pylonHeadBBox, pylonBodyBBox, pylonFootBBox;

			for (size_t i = 0; i < _indices.size(); ++i)
			{
				const auto& p = _input->points[_indices[i]];
				osg::Vec3d pt = _transform.preMult(osg::Vec3d(p.x, p.y, p.z));

				if (pt.z() <= Z1)
				{
					pylonFoot.push_back(_indices[i]);
					pylonFootBBox.expandBy(p.x, p.y, p.z);
				}
				else if (pt.z() > Z1 && pt.z() <= Zk)
				{
					pylonBody.push_back(_indices[i]);
					pylonBodyBBox.expandBy(p.x, p.y, p.z);
				}
				else
				{
					pylonHead.push_back(_indices[i]);
					pylonHeadBBox.expandBy(p.x, p.y, p.z);
				}
			}

			_pylonHeadIndices = pylonHead;
			_pylonBodyIndices = pylonBody;
			_pylonFootIndices = pylonFoot;

#ifdef WRITE_LAS
			std::string filename = DebugDirectory + "Head.las";
			PCS_DEBUG("[PylonDecomposer] 正在写入 %s.", filename.c_str());
			writePointCloud(filename, *_input, pylonHead);

			filename = DebugDirectory + "Body.las";
			PCS_DEBUG("[PylonDecomposer] 正在写入 %s.", filename.c_str());
			writePointCloud(filename, *_input, pylonBody);

			filename = DebugDirectory + "Foot.las";
			PCS_DEBUG("[PylonDecomposer] 正在写入 %s.", filename.c_str());
			writePointCloud(filename, *_input, pylonFoot);

			if (_mesh)
			{
				ClipMesh(_mesh, pylonHeadBBox, DebugDirectory + "Head.stl");
				ClipMesh(_mesh, pylonBodyBBox, DebugDirectory + "Body.stl");
				ClipMesh(_mesh, pylonFootBBox, DebugDirectory + "Foot.stl");
			}
#endif
		}

		std::vector<int> PylonDecomposer::GetPylonHead(const osg::BoundingBox& bbox, double length)
		{
			std::vector<int> headIndices;
			double zLower = std::max((double)bbox.zMin(), (double)bbox.zMax() - length);

			headIndices.clear();
			headIndices.reserve(_indices.size());

			for (size_t i = 0; i < _indices.size(); ++i)
			{
				const auto& p = _input->points[_indices[i]];

				if (p.z > zLower)
					headIndices.push_back(_indices[i]);
			}

			headIndices.shrink_to_fit();

			return headIndices;
		}

		void PylonDecomposer::GetSlices(double zmin,
										double zmax,
										double step,
										std::vector<std::vector<int>>& slices)
		{
			CHECK(zmax >= zmin);
			CHECK(step > 0.0);

			int num = std::floor((zmax - zmin) / step) + 1;

			if (num < 0)
			{
				PCS_ERROR("[PylonDecomposer] 切分单元失败.");
				return;
			}

			slices.resize(num);

			for (size_t i = 0; i < _indices.size(); ++i)
			{
				const auto& p = _input->points[_indices[i]];
				osg::Vec3d pt(p.x, p.y, p.z);
				pt = _transform.preMult(pt);

				int index = std::floor(((double)pt.z() - zmin) / step);
				index = clamp(index, 0, num - 1);

				slices[index].push_back(_indices[i]);
			}
		}

		void PylonDecomposer::ComputeKeyPositions(const std::vector<std::vector<int>>& slices,
												  int size,
												  std::vector<int>& kp)
		{
			CHECK(!slices.empty());
			kp.clear();

			std::set<int> localmax_set;

			int sliceSize = std::max((int)slices.size() - size, 0) + 1;

			for (int i = 0; i < sliceSize; ++i)
			{
				int local_max = i;
				int local_num = 0;

				for (int j = i; j < i + size; ++j)
				{
					const auto& slice = slices.at(j);
					int numPoints = (int)slice.size();

					if (numPoints > local_num)
					{
						local_num = numPoints;
						local_max = j;
					}
				}

				localmax_set.insert(local_max);
			}

			kp.insert(kp.end(), localmax_set.begin(), localmax_set.end());
		}

		void PylonDecomposer::KeyPositionsFiltering(const std::vector<std::vector<int>>& slices,
													std::vector<int>& kp)
		{
			double step = _options.x_step;
			double fillrateThr = _options.fillrate;
			osg::Matrix transform = _transform;

			// 按照点云密度过滤
			std::vector<int> sizes;

			for (size_t i = 0; i < kp.size(); ++i)
			{
				int numPoints = slices[kp[i]].size();
				sizes.push_back(numPoints);
			}

			double mul = 0.1;
			int mean = (int)Mean(sizes);
			int stddev = (int)Stddev(sizes);
			int threshold = mul * stddev;

			// 按X方向填充率过滤
			for (int i = (int)kp.size() - 1; i >= 0; --i)
			{
				const auto& slice = slices[kp[i]];

				// X 方向最大、最小值
				double xmin = DBL_MAX;
				double xmax = -DBL_MAX;

				for (size_t j = 0; j < slice.size(); ++j)
				{
					const auto& p = _input->points[slice[j]];
					osg::Vec3d pt = transform.preMult(osg::Vec3d(p.x, p.y, p.z));

					xmin = std::min(xmin, (double)pt.x());
					xmax = std::max(xmax, (double)pt.x());
				}

				int num = std::floor((xmax - xmin) / step) + 1;

				if (num <= 0)
				{
					PCS_ERROR("[PylonDecomposer] 切分单元失败.");
					return;
				}

				int hit = 0;
				std::vector<int> bins(num, 0);

				for (size_t j = 0; j < slice.size(); ++j)
				{
					const auto& p = _input->points[slice[j]];
					osg::Vec3d pt = transform.preMult(osg::Vec3d(p.x, p.y, p.z));

					int index = std::floor(((double)pt.x() - xmin) / step);
					index = clamp(index, 0, num - 1);

					if (bins[index] == 0)
						hit++;

					bins[index]++;
				}

				double fr = (double)hit / (double)num;
				int delta = (int)slice.size() - mean;

				if (delta < threshold || fr < fillrateThr)
					kp.erase(kp.begin() + i);
			}
		}

		void PylonDecomposer::KeyPositionsClustering(std::vector<int>& kp)
		{
			std::vector<std::vector<int>> kpClusters;
			std::set<int> processed;

			for (int i = 0; i < (int)kp.size(); ++i)
			{
				if (processed.find(i) != processed.end())
					continue;

				std::vector<int> seed_queue;

				int sq_idx = 0;
				seed_queue.push_back(i);
				processed.insert(i);

				while (sq_idx < (int)seed_queue.size())
				{
					int j = seed_queue[sq_idx];
					const auto& p = kp[j];

					// 遍历相邻接节点
					for (int k = (int)j + 1; k < (int)kp.size(); ++k)
					{
						if (processed.find(k) != processed.end())
							continue;

						const auto& q = kp[k];

						if (abs(q - p) < 3)
						{
							seed_queue.push_back(k);
							processed.insert(k);
						}
					}

					sq_idx++;

				} // end of while

				if (!seed_queue.empty())
					kpClusters.push_back(seed_queue);
			}


			std::vector<int> newKp;

			for (size_t i = 0; i < kpClusters.size(); ++i)
			{
				const auto cluster = kpClusters.at(i);

				/*if (cluster.size() > 1)
				{
					int mid = cluster.size() / 2;
					newKp.push_back(kp[cluster[mid]]);
				}
				else
				{
					newKp.push_back(kp[cluster[0]]);
				}*/

				newKp.push_back(kp[cluster[0]]);
			}

			kp = newKp;
		}

		void PylonDecomposer::ComputeKeySegmentPositions(
			const std::vector<std::vector<int>>& slices,
			const std::vector<int>& kp,
			std::vector<int>& ksp)
		{
			CHECK(!kp.empty());

			osg::Matrix transform = _transform;

			ksp.clear();

			// 计算XY平面投影区域的纵横比
			auto ComputeAspect = [&](const std::vector<int>& indices) {
				osg::BoundingBox bbox;

				for (size_t i = 0; i < indices.size(); ++i)
				{
					const auto& p = _input->points[indices[i]];
					osg::Vec3d pt = transform.preMult(osg::Vec3d(p.x, p.y, p.z));
					bbox.expandBy(pt);
				}

				double w = bbox.xMax() - bbox.xMin();
				double h = bbox.yMax() - bbox.yMin();

				// 保证 w > h
				if (w < h)
					std::swap(w, h);

				return (w / h);
			};


			// 第一个KeyPosition是塔腿分割位置，它是关键分割位置
			ksp.push_back(kp[0]);

			// 计算纵横比G1
			double G1 = ComputeAspect(slices[kp[0]]);
			double Gt = G1 + _options.aspect_error;

			for (size_t i = 1; i < kp.size(); ++i)
			{
				int index = kp[i];
				CHECK(index >= 0 && index < slices.size());
				const auto& slice = slices.at(index);

				// 计算关键位置层内点云在XY平面上投影区域的纵横比
				double G = ComputeAspect(slice);

				if (G > Gt)
				{
					ksp.push_back(i == 1 ? kp[i] : kp[i - 1]);
					break;
				}
			}
		}

	}
}