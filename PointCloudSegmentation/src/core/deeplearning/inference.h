//////////////////////////////////////////////////////////////////////
// 文件名称：inference.h
// 功能描述：深度学习推理算法
// 创建标识：吕伟	2023/1/19
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef INFERENCE_H_
#define INFERENCE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#ifdef INFERENCE_USE_NCNN
#include <ncnn/net.h>
#include <ncnn/mat.h>
#else
namespace ncnn {
	class Net;
}
#endif


namespace d3s {
	namespace pcs {

		/**
		 *  @brief    UNet 分割图像推理
		 *
		 *  @param    const std::string & onnx_model_path			UNet 模型路径
		 *  @param    unsigned int image_size						网络输入尺寸
		 *  @param    std::string input_file						输入RGB图像
		 *  @param    std::string output_file						输出掩码图像
		 *
		 *  @return   void
		 */
		void unet_inference_dnn(const std::string& onnx_model_path,
								unsigned int image_size,
								std::string input_file,
								std::string output_file);

		/**
		 *  @brief    UNet 分割图像推理(使用OpenCV DNN)
		 *
		 *  @param    cv::dnn::Net& model							UNet 模型路径
		 *  @param    unsigned int image_size						网络输入尺寸
		 *  @param    cv::Mat image									RGB 图像
		 *  @param    cv::Mat& score								Score
		 *
		 *  @return   void
		 */
		void unet_inference_dnn(cv::dnn::Net& model,
								unsigned int image_size,
								cv::Mat image,
								cv::Mat& score);


		/**
		 *  @brief    UNet 分割图像推理(使用腾讯NCNN)
		 *
		 *  @param    const std::string & param_path				模型参数文件
		 *  @param    const std::string & model_path				模型权重文件
		 *  @param    unsigned int image_size						网络输入尺寸
		 *  @param    std::string input_file						输入RGB图像
		 *  @param    std::string output_file						输出掩码图像
		 *
		 *  @return   void
		 */
		void unet_inference_ncnn(const std::string& param_path,
								 const std::string& model_path,
								 unsigned int image_size,
								 std::string input_file,
								 std::string output_file);

		/**
		 *  @brief    UNet 分割图像推理(使用腾讯NCNN)
		 *
		 *  @param    ncnn::Net & model								UNet 模型路径
		 *  @param    unsigned int image_size						网络输入尺寸
		 *  @param    cv::Mat image									RGB 图像
		 *  @param    cv::Mat & score								Score
		 *
		 *  @return   void
		 */
		void unet_inference_ncnn(ncnn::Net& model,
								 unsigned int image_size,
								 cv::Mat image,
								 cv::Mat& score);

		void pp_stdnet2_inference_ncnn(const std::string& param_path,
									   const std::string& model_path,
									   unsigned int image_size,
									   std::string input_file,
									   std::string output_file);

		void pp_stdnet2_inference_ncnn(ncnn::Net& model,
									   unsigned int image_size,
									   cv::Mat image,
									   cv::Mat& mask);

		void pp_stdnet2_inference_ncnn(ncnn::Net& model,
									   unsigned int image_size,
									   cv::Mat image,
									   cv::Mat& mask,
									   cv::Mat& prec);

		void pplcnet_x1_0_inference_ncnn(const std::string& param_path,
										 const std::string& bin_path,
										 unsigned int image_size,
										 std::string input_file,
										 std::vector<float>& scores);

		void pplcnet_x1_0_inference_ncnn(ncnn::Net& model,
										 unsigned int image_size,
										 cv::Mat image,
										 std::vector<float>& scores);


		bool select_gpu_device(int& device_index);
	}
}

#endif // INFERENCE_H_
