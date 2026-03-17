#ifndef APPROXIMATE_PMF_HPP_
#define APPROXIMATE_PMF_HPP_

#include <src/algorithm/approximate_pmf.h>
#include <src/utils/logging.h>

#include <pcl/common/common.h>

// #include <pcl/common/common.h>
// #include <pcl/common/io.h>
// #include <pcl/filters/morphological_filter.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/pcl_macros.h>


#ifndef pcl_isnan
#include <cmath>
#define pcl_isnan(x) std::isnan(x)
#endif
namespace d3s {
	namespace pcs {

		template <typename PointT>
		ApproximatePMF<PointT>::ApproximatePMF()
			: max_window_size_(33),
			  slope_(0.7f),
			  max_distance_(10.0f),
			  initial_distance_(0.15f),
			  cell_size_(1.0f),
			  base_(2.0f),
			  exponential_(true),
			  threads_(0)
		{
		}


		template <typename PointT>
		ApproximatePMF<PointT>::~ApproximatePMF()
		{
		}


		template <typename PointT>
		void ApproximatePMF<PointT>::extract(std::vector<int>& ground)
		{
			bool segmentation_is_possible = initCompute();
			if (!segmentation_is_possible)
			{
				deinitCompute();
				return;
			}

			// Compute the series of window sizes and height thresholds
			std::vector<float> height_thresholds;
			std::vector<float> window_sizes;
			std::vector<int> half_sizes;
			int iteration = 0;
			int half_size = 0.0f;
			float window_size = 0.0f;
			float height_threshold = 0.0f;

			while (window_size < max_window_size_)
			{
				// Determine the initial window size.
				if (exponential_)
					half_size = static_cast<int>(std::pow(static_cast<float>(base_), iteration));
				else
					half_size = (iteration + 1) * base_;

				window_size = 2 * half_size + 1;

				// Calculate the height threshold to be used in the next iteration.
				if (iteration == 0)
					height_threshold = initial_distance_;
				else
					height_threshold =
						slope_ * (window_size - window_sizes[iteration - 1]) * cell_size_ +
						initial_distance_;

				// Enforce max distance on height threshold
				if (height_threshold > max_distance_)
					height_threshold = max_distance_;

				half_sizes.push_back(half_size);
				window_sizes.push_back(window_size);
				height_thresholds.push_back(height_threshold);

				iteration++;
			}

			// setup grid based on scale and extents
			Eigen::Vector4f global_max, global_min;
			pcl::getMinMax3D<PointT>(*input_, *indices_, global_min, global_max);

			float xextent = global_max.x() - global_min.x();
			float yextent = global_max.y() - global_min.y();

			int rows = static_cast<int>(std::floor(yextent / cell_size_) + 1);
			int cols = static_cast<int>(std::floor(xextent / cell_size_) + 1);

			Eigen::MatrixXf A(rows, cols);
			A.setConstant(std::numeric_limits<float>::quiet_NaN());

			Eigen::MatrixXf Z(rows, cols);
			Z.setConstant(std::numeric_limits<float>::quiet_NaN());

			Eigen::MatrixXf Zf(rows, cols);
			Zf.setConstant(std::numeric_limits<float>::quiet_NaN());

#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
			for (int i = 0; i < (int)indices_->size(); ++i)
			{
				// ...then test for lower points within the cell
				PointT p = input_->points[(*indices_)[i]];
				int row = std::floor((p.y - global_min.y()) / cell_size_);
				int col = std::floor((p.x - global_min.x()) / cell_size_);

				if (p.z < A(row, col) || pcl_isnan(A(row, col)))
				{
					A(row, col) = p.z;
				}
			}

			// Ground indices are initially limited to those points in the input cloud we
			// wish to process
			ground = *indices_;

			// Progressively filter ground returns using morphological open
			for (size_t i = 0; i < window_sizes.size(); ++i)
			{
				PCS_DEBUG("      Iteration %d (height threshold = %f, window size = %f, half size "
						  "= %d)...",
						  i,
						  height_thresholds[i],
						  window_sizes[i],
						  half_sizes[i]);

				// Limit filtering to those points currently considered ground returns
				/*typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
				pcl::copyPointCloud<PointT>(*input_,  ground, *cloud);*/

				// Apply the morphological opening operation at the current window size.
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
				for (int row = 0; row < rows; ++row)
				{
					int rs, re;
					rs = ((row - half_sizes[i]) < 0) ? 0 : row - half_sizes[i];
					re = ((row + half_sizes[i]) > (rows - 1)) ? (rows - 1) : row + half_sizes[i];

					for (int col = 0; col < cols; ++col)
					{
						int cs, ce;
						cs = ((col - half_sizes[i]) < 0) ? 0 : col - half_sizes[i];
						ce =
							((col + half_sizes[i]) > (cols - 1)) ? (cols - 1) : col + half_sizes[i];

						float min_coeff = std::numeric_limits<float>::max();

						for (int j = rs; j < (re + 1); ++j)
						{
							for (int k = cs; k < (ce + 1); ++k)
							{
								if (A(j, k) != std::numeric_limits<float>::quiet_NaN())
								{
									if (A(j, k) < min_coeff)
										min_coeff = A(j, k);
								}
							}
						}

						if (min_coeff != std::numeric_limits<float>::max())
							Z(row, col) = min_coeff;
					}
				}

#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
				for (int row = 0; row < rows; ++row)
				{
					int rs, re;
					rs = ((row - half_sizes[i]) < 0) ? 0 : row - half_sizes[i];
					re = ((row + half_sizes[i]) > (rows - 1)) ? (rows - 1) : row + half_sizes[i];

					for (int col = 0; col < cols; ++col)
					{
						int cs, ce;
						cs = ((col - half_sizes[i]) < 0) ? 0 : col - half_sizes[i];
						ce =
							((col + half_sizes[i]) > (cols - 1)) ? (cols - 1) : col + half_sizes[i];

						float max_coeff = -std::numeric_limits<float>::max();

						for (int j = rs; j < (re + 1); ++j)
						{
							for (int k = cs; k < (ce + 1); ++k)
							{
								if (Z(j, k) != std::numeric_limits<float>::quiet_NaN())
								{
									if (Z(j, k) > max_coeff)
										max_coeff = Z(j, k);
								}
							}
						}

						if (max_coeff != -std::numeric_limits<float>::max())
							Zf(row, col) = max_coeff;
					}
				}

				// Find indices of the points whose difference between the source and
				// filtered point clouds is less than the current height threshold.
				std::vector<int> pt_indices;

				for (size_t p_idx = 0; p_idx < ground.size(); ++p_idx)
				{
					PointT p = input_->points[ground[p_idx]];
					int erow = static_cast<int>(std::floor((p.y - global_min.y()) / cell_size_));
					int ecol = static_cast<int>(std::floor((p.x - global_min.x()) / cell_size_));

					float diff = p.z - Zf(erow, ecol);
					if (diff < height_thresholds[i])
						pt_indices.push_back(ground[p_idx]);
				}

				A.swap(Zf);

				// Ground is now limited to pt_indices
				ground.swap(pt_indices);

				PCS_DEBUG("ground now has %d points.\n", ground.size());
			}

			deinitCompute();
		}
	}
}

#define PCL_INSTANTIATE_ApproximatePMF(T) template class d3s::pcs::ApproximatePMF<T>;


#endif // APPROXIMATE_PMF_HPP_