#ifndef PREPROCESS_OP_H_
#define PREPROCESS_OP_H_

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace d3s {
	namespace pcs {
		namespace paddlecls {

			class Normalize
			{
			public:
				virtual void Run(cv::Mat* im,
								 const std::vector<float>& mean,
								 const std::vector<float>& std,
								 float& scale);
			};

			// RGB -> CHW
			class Permute
			{
			public:
				virtual void Run(const cv::Mat* im, float* data);
			};

			class CenterCropImg
			{
			public:
				virtual void Run(cv::Mat& im, const int crop_size = 224);
			};

			class ResizeImg
			{
			public:
				virtual void Run(const cv::Mat& img, cv::Mat& resize_img, int max_size_len);
			};
		}
	}
}

#endif // PREPROCESS_OP_H_
