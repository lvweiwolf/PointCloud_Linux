#include <src/core/deeplearning/inference.h>
#include <src/core/deeplearning/preprocess_op.h>
#include <src/core/private/rasterProcess.h>
#include <src/utils/timer.h>

#include <ncnn/gpu.h>

#include <vulkan/vulkan.h>

namespace d3s {
	namespace pcs {

		
		template <class T>
		T get_value(const ncnn::Mat& _to_process_mat,
					const int row_index,
					const int col_index,
					const int channel_index)
		{
			return *((T*)_to_process_mat.data + (channel_index * _to_process_mat.cstep) +
					 row_index * _to_process_mat.w + col_index);
		}

		template <class T>
		void set_value(const ncnn::Mat& _to_process_mat,
					   const int row_index,
					   const int col_index,
					   const int channel_index,
					   T value)
		{
			*((T*)_to_process_mat.data + (channel_index * _to_process_mat.cstep) +
			  row_index * _to_process_mat.w + col_index) = value;
		}

		template <class T>
		void argmax(const ncnn::Mat& _to_process_mat, ncnn::Mat& _result_mat)
		{
			assert(sizeof(T) == _to_process_mat.elemsize);
			int classes = _to_process_mat.c;

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int h = 0; h < _to_process_mat.h; ++h)
			{
				for (int w = 0; w < _to_process_mat.w; ++w)
				{
					int max_index = 0;
					T max_value = get_value<T>(_to_process_mat, h, w, 0);
					for (int i = 1; i < classes; ++i)
					{
						T current_value = get_value<T>(_to_process_mat, h, w, i);
						if (current_value > max_value)
						{
							max_index = i;
							max_value = current_value;
						}
					}

					set_value<T>(_result_mat, h, w, 0, max_index);
				}
			}
		}

		template <class T>
		void argmax(const ncnn::Mat& _to_process_mat, ncnn::Mat& _index_mat, ncnn::Mat& _prec_mat)
		{
			assert(sizeof(T) == _to_process_mat.elemsize);
			int classes = _to_process_mat.c;

#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int h = 0; h < _to_process_mat.h; ++h)
			{
				for (int w = 0; w < _to_process_mat.w; ++w)
				{
					int max_index = 0;
					T max_value = get_value<T>(_to_process_mat, h, w, 0);
					for (int i = 1; i < classes; ++i)
					{
						T current_value = get_value<T>(_to_process_mat, h, w, i);
						if (current_value > max_value)
						{
							max_index = i;
							max_value = current_value;
						}
					}

					set_value<T>(_index_mat, h, w, 0, max_index);
					set_value<T>(_prec_mat, h, w, 0, max_value);
				}
			}
		}

		std::vector<float> softmax(const std::vector<float>& input)
		{
			std::vector<float> output(input.size());

			float sum = 0.0;
			for (int i = 0; i < (int)input.size(); i++)
			{
				output[i] = exp(input[i]);
				sum += output[i];
			}

			for (int i = 0; i < (int)output.size(); i++)
			{
				output[i] /= sum;
			}

			return output;
		}

		cv::Mat sigmoid(const cv::Mat& x)
		{
			cv::Mat x_exp;
			cv::exp(-x, x_exp);
			return 1.0f / (1.0f + x_exp);
		}

		void cv_normalize(const cv::Mat& src,
						  cv::Mat& dst,
						  cv::Vec3d mean_value = cv::Vec3d(0.485, 0.456, 0.406),
						  cv::Vec3d std_value = cv::Vec3d(0.229, 0.224, 0.225),
						  double max_pixel_value = 255.0)
		{
			cv::Mat mat = src;
			mat.convertTo(mat, CV_64FC3);

			cv::Mat mean = cv::Mat3d(mat.rows, mat.cols, mean_value);
			cv::Mat std = cv::Mat3d(mat.rows, mat.cols, std_value);
			mean *= max_pixel_value;
			std *= max_pixel_value;
			cv::Mat denominator = 1.0 / std;

			cv::subtract(mat, mean, mat);
			cv::multiply(mat, denominator, mat);

			mat.convertTo(mat, CV_32FC3);
			dst = mat;
		}

		void unet_inference_dnn(const std::string& onnx_model_path,
								unsigned int image_size,
								std::string input_file,
								std::string output_file)
		{
			cv::Mat image = cv::imread(input_file);
			cv::Size input_size(image.cols, image.rows);

			cv::dnn::Net model = cv::dnn::readNetFromONNX(onnx_model_path);

			cv::Mat score;
			unet_inference_dnn(model, image_size, image, score);

			// 预测数据后处理
			cv::Mat mask(score.rows, score.cols, CV_8U, cv::Scalar(0));
			cv::threshold(score, mask, 0.85, 255, cv::THRESH_BINARY);

			// 调整大小到原始尺寸
			cv::resize(mask, mask, input_size, cv::INTER_NEAREST);

			// 写入掩码图像
			cv::imwrite(output_file, mask);
		}

		void unet_inference_dnn(cv::dnn::Net& model,
								unsigned int image_size,
								cv::Mat image,
								cv::Mat& score)
		{
			// 推理前的数据预处理
			cv::Size img_size(image_size, image_size);
			cv::resize(image, image, img_size, cv::INTER_NEAREST);

			// 推理前的数据预处理
			cv_normalize(image, image);
			image = image / 255.0f;

			// 将图像数据送入UNet网络，推理结果
			cv::Mat input = cv::dnn::blobFromImage(image, 1.0f, img_size);
			model.setInput(input);

			cv::Mat output = model.forward();
			output = sigmoid(output); // 因为是二分类，此处采用sigmoid. 多分类则使用 argmax.

			PCS_INFO("output size: (%d, %d, %d, %d)",
					 output.size[0],
					 output.size[1],
					 output.size[2],
					 output.size[3]);

			// 拷贝内存
			// output: [1, 1, image_size, image_size]
			// unsigned int sizes[3] = { 1, image_size, image_size }; // C, H, W
			score = cv::Mat(img_size, CV_32F, output.ptr<float>(0, 0)).clone();
		}



		void unet_inference_ncnn(const std::string& param_path,
								 const std::string& model_path,
								 unsigned int image_size,
								 std::string input_file,
								 std::string output_file)
		{
#ifdef INFERENCE_USE_NCNN
			cv::Mat image = cv::imread(input_file);
			cv::Size input_size(image.cols, image.rows);

			ncnn::Net model;
#if NCNN_VULKAN
			int device_index = 0;
			if (select_gpu_device(device_index))
			{
				model.opt.use_vulkan_compute = true;
				model.set_vulkan_device(device_index);
			}
			
			if (device_index < 0)
			{
				PCS_ERROR("[unet_inference_ncnn] 推理失败，未能检测到设备.");
				return;
			}
#endif
			model.load_param(param_path.c_str());
			model.load_model(model_path.c_str());

			Timer timer;
			timer.Start();

			cv::Mat score;
			unet_inference_ncnn(model, image_size, image, score);

			PCS_INFO("UNet 推理时间 %lf s.", timer.ElapsedSeconds());

			// 预测数据后处理
			cv::Mat mask(score.rows, score.cols, CV_8U, cv::Scalar(0));
			cv::threshold(score, mask, 0.85, 255, cv::THRESH_BINARY);

			// 调整大小到原始尺寸
			cv::resize(mask, mask, input_size, cv::INTER_NEAREST);

			// 写入掩码图像
			cv::imwrite(output_file, mask);

#else
			PCS_WARN("NCNN 推理引擎未启用，开启 INFERENCE_USE_NCNN 可启用 NCNN.");
#endif
		}


		void unet_inference_ncnn(ncnn::Net& model,
								 unsigned int image_size,
								 cv::Mat image,
								 cv::Mat& score)
		{
#ifdef INFERENCE_USE_NCNN
			// 推理前的数据预处理
			cv::Size img_size(image_size, image_size);
			cv::resize(image, image, img_size, cv::INTER_NEAREST);

			// 推理前的数据预处理
			cv_normalize(image, image);
			image = image / 255.0f;

			// 转换数据，并调整尺寸
			ncnn::Mat inputpack3(image.cols,
								 image.rows,
								 1,
								 (void*)image.data,
								 sizeof(float) * 3,
								 3);
			ncnn::Mat input;
			ncnn::convert_packing(inputpack3, input, 1);

			ncnn::Extractor extractor = model.create_extractor();
			extractor.set_light_mode(true);
			extractor.set_num_threads(4);
#if NCNN_VULKAN
			if (model.opt.use_vulkan_compute)
				extractor.set_vulkan_compute(true);
#endif

			extractor.input("input.1", input);
			ncnn::Mat out;
			extractor.extract("211", out);

			cv::Mat output(out.h, out.w, CV_32FC1);
			memcpy((uchar*)output.data, out.data, out.w * out.h * sizeof(float));
			score = sigmoid(output);
#else
			PCS_WARN("NCNN 推理引擎未启用，开启 INFERENCE_USE_NCNN 可启用 NCNN.");
#endif
		}

		void pp_stdnet2_inference_ncnn(const std::string& param_path,
									   const std::string& model_path,
									   unsigned int image_size,
									   std::string input_file,
									   std::string output_file)
		{

#ifdef INFERENCE_USE_NCNN 
			cv::Mat image = cv::imread(input_file);
			cv::Size input_size(image.cols, image.rows);

			ncnn::Net model;
#if NCNN_VULKAN
			int device_index = 0;
			if (select_gpu_device(device_index))
			{
				model.opt.use_vulkan_compute = true;
				model.set_vulkan_device(device_index);
			}

			if (device_index < 0)
			{
				PCS_ERROR("[pp_stdnet2_inference_ncnn] 推理失败，未能检测到设备.");
				return;
			}
#endif
			model.load_param(param_path.c_str());
			model.load_model(model_path.c_str());

			Timer timer;
			timer.Start();

			cv::Mat mask;
			pp_stdnet2_inference_ncnn(model, image_size, image, mask);

			PCS_INFO("PP_STDNet2 推理时间 %lf s.", timer.ElapsedSeconds());

			// 预测数据后处理
			auto colorMap = createPseudoColors(256);

			// 灰度转伪彩色...
			cv::Mat dst(mask.rows, mask.cols, CV_8UC3, cv::Scalar(0));

			for (int r = 0; r < mask.rows; ++r)
			{
				for (int c = 0; c < mask.cols; ++c)
				{
					int index = (int)mask.at<uint8_t>(r, c);
					if ( index > 0)
					{
						dst.at<cv::Vec3b>(r, c) = cv::Vec3b(colorMap[index][0] * 255,
															colorMap[index][1] * 255,
															colorMap[index][2] * 255);
					}
				}
			}

			cv::cvtColor(dst, dst, cv::COLOR_RGB2BGR);
			mask = dst;

			// 调整大小到原始尺寸
			cv::resize(mask, mask, input_size, cv::INTER_NEAREST);

			// 写入掩码图像
			cv::imwrite(output_file, dst);

#else
			PCS_WARN("NCNN 推理引擎未启用，开启 INFERENCE_USE_NCNN 可启用 NCNN.");
#endif

		}

		void pp_stdnet2_inference_ncnn(ncnn::Net& model,
									   unsigned int image_size,
									   cv::Mat image,
									   cv::Mat& mask)
		{
			cv::Mat prec;
			pp_stdnet2_inference_ncnn(model, image_size, image, mask, prec);
		}

		

		void pp_stdnet2_inference_ncnn(ncnn::Net& model,
									   unsigned int image_size,
									   cv::Mat image,
									   cv::Mat& mask,
									   cv::Mat& prec)
		{
#ifdef INFERENCE_USE_NCNN
			// 转换到RGB
			cv::cvtColor(image, image, CV_BGR2RGB);

			// 推理前的数据预处理
			cv_normalize(image, image, cv::Vec3d(0.5, 0.5, 0.5), cv::Vec3d(0.5, 0.5, 0.5));

			// 推理前的数据预处理
			cv::Size img_size(image_size, image_size);
			cv::resize(image, image, img_size, cv::INTER_NEAREST);


			// 转换数据，并调整尺寸
			ncnn::Mat inputpack3(image.cols,
								 image.rows,
								 1,
								 (void*)image.data,
								 sizeof(float) * 3,
								 3);
			ncnn::Mat input;
			ncnn::convert_packing(inputpack3, input, 1);

			ncnn::Extractor extractor = model.create_extractor();
			extractor.set_light_mode(true);
			extractor.set_num_threads(30);
#if NCNN_VULKAN
			if (model.opt.use_vulkan_compute)
				extractor.set_vulkan_compute(true);
#endif
			extractor.input("x", input);
			ncnn::Mat logit;
			extractor.extract("bilinear_interp_v2_6.tmp_0", logit);

			ncnn::Mat pred(logit.w, logit.h, 1);
			ncnn::Mat score(logit.w, logit.h, 4);
			pred.fill(0);
			score.fill(0.0f);

			argmax<float>(logit, pred, score);

			mask = cv::Mat(pred.h, pred.w, CV_8UC1);
			prec = cv::Mat(score.h, score.w, CV_32FC1);
			pred.to_pixels(mask.data, ncnn::Mat::PIXEL_GRAY);
			memcpy((uchar*)prec.data, score.data, score.w * score.h * sizeof(float));
#else
			PCS_WARN("NCNN 推理引擎未启用，开启 INFERENCE_USE_NCNN 可启用 NCNN.");
#endif
		}

		void pplcnet_x1_0_inference_ncnn(const std::string& param_path,
										 const std::string& bin_path,
										 unsigned int image_size,
										 std::string input_file,
										 std::vector<float>& scores)
		{
#ifdef INFERENCE_USE_NCNN
			cv::Mat image = cv::imread(input_file);
			cv::Size input_size(image.cols, image.rows);

			ncnn::Net model;
#if NCNN_VULKAN
			int device_index = 0;
			if (select_gpu_device(device_index))
			{
				model.opt.use_vulkan_compute = true;
				model.set_vulkan_device(device_index);
			}
			
			if (device_index < 0)
			{
				PCS_ERROR("[pplcnet_x1_0_inference_ncnn] 推理失败，未能检测到设备.");
				return;
			}
#endif
			model.load_param(param_path.c_str());
			model.load_model(bin_path.c_str());

			Timer timer;
			timer.Start();

			pplcnet_x1_0_inference_ncnn(model, image_size, image, scores);
			PCS_INFO("PPLCNET_x1_0 推理时间 %lf s.", timer.ElapsedSeconds());
#else
			PCS_WARN("NCNN 推理引擎未启用，开启 INFERENCE_USE_NCNN 可启用 NCNN.");
#endif
		}

		void pplcnet_x1_0_inference_ncnn(ncnn::Net& model,
										 unsigned int image_size,
										 cv::Mat image,
										 std::vector<float>& scores)
		{
#ifdef INFERENCE_USE_NCNN
			// 转换到RGB
			cv::cvtColor(image, image, CV_BGR2RGB);
			
			int resize_short_size = 256;
			int crop_size = image_size;
			float scale = 1.0f / 255.0f;
			std::vector<float> mean = { 0.485f, 0.456f, 0.406f };
			std::vector<float> std = { 0.229f, 0.224f, 0.225f };

			paddlecls::ResizeImg resize_op;
			paddlecls::Normalize normalize_op;
			paddlecls::CenterCropImg crop_op;
			paddlecls::Permute premute_op;

			cv::Mat resize_img;
			// 短边缩放
			resize_op.Run(image, resize_img, resize_short_size);
			
			// 裁剪到指定尺寸
			crop_op.Run(resize_img, crop_size);

			// 标准化数据
			normalize_op.Run(&resize_img, mean, std, scale);

			// HWC => CHW
			std::vector<float> buffer(1 * 3 * resize_img.rows * resize_img.cols, 0.0f);
			premute_op.Run(&resize_img, buffer.data());

			ncnn::Mat input(resize_img.cols, resize_img.rows, 3, sizeof(float));
			memcpy(input.data, buffer.data(), buffer.size() * sizeof(float));
			
			/*ncnn::Mat inputpack3(resize_img.cols,
								 resize_img.rows,
								 1,
								 (void*)resize_img.data,
								 sizeof(float) * 3,
								 3);
			ncnn::Mat input;
			ncnn::convert_packing(inputpack3, input, 1);
			*/
			ncnn::Mat channel0 = input.channel(0);
			ncnn::Mat channel1 = input.channel(1);
			ncnn::Mat channel2 = input.channel(2);
			

			ncnn::Extractor extractor = model.create_extractor();
			extractor.set_light_mode(true);
			extractor.set_num_threads(4);
#if NCNN_VULKAN
			if (model.opt.use_vulkan_compute)
				extractor.set_vulkan_compute(true);
#endif

			extractor.input("x", input);
			ncnn::Mat pred;
			extractor.extract("linear_1.tmp_1", pred);

			std::vector<float> linear_feat;
			linear_feat.resize(pred.w);
			for (int i = 0; i < pred.w; i++)
				linear_feat[i] = pred[i];

			scores = softmax(linear_feat);

			/*for (int i = 0; i < scores.size(); i++)
				PCS_INFO("score %d : %.4f", i, scores[i]);*/
#else
			PCS_WARN("NCNN 推理引擎未启用，开启 INFERENCE_USE_NCNN 可启用 NCNN.");
#endif
		}

		bool select_gpu_device(int& device_index)
		{
			int gpu_count = ncnn::get_gpu_count();

			if (gpu_count <= 0)
				PCS_ERROR("[select_gpu_device] 推理失败，未能检测到GPU设备.");

			bool gpu_select = false;
			device_index = 0;

			const ncnn::GpuInfo* gpuInfo = nullptr;

			// 先搜索合适的独立显卡
			for (int i = 0; i < gpu_count; ++i)
			{
				gpuInfo = &ncnn::get_gpu_info(i);
				if (!gpuInfo)
					continue;

				if (gpuInfo->type() == 0) // 独立显卡
				{
					device_index = i;
					gpu_select = true;
					break;
				}
			}

			if (!gpu_select)
			{
				device_index = ncnn::get_default_gpu_index();
				gpuInfo = device_index >= 0 ? &ncnn::get_gpu_info(device_index) : nullptr;
				PCS_WARN("[select_cpu_device] WARNING: 未找到适合开启Vulkan的GPU设备，使用CPU进行推理.");
			}

			if (gpuInfo)
			{
				std::string gpu_type = "未知";
				int type = gpuInfo->type();

				switch (type)
				{
				case 0:
					gpu_type = "独立显卡";
					break;
				case 1:
					gpu_type = "集成显卡";
					break;
				case 2:
					gpu_type = "虚拟显卡";
					break;
				case 3:
					gpu_type = "CPU设备";
					break;
				default:
					break;
				}

				PCS_INFO("[select_gpu_device] GPU device name: %s[%s]",
						 gpuInfo->device_name(),
						 gpu_type.c_str());

				uint32_t major = 0;
				uint32_t minor = 0;
				uint32_t patch = 0;

				uint32_t api_version = gpuInfo->api_version();
				major = VK_VERSION_MAJOR(api_version);
				minor = VK_VERSION_MINOR(api_version);
				patch = VK_VERSION_PATCH(api_version);
				PCS_INFO("[select_gpu_device] Api version: %d.%d.%d", major, minor, patch);

				uint32_t driver_version = gpuInfo->driver_version();
				major = VK_VERSION_MAJOR(driver_version);
				minor = VK_VERSION_MINOR(driver_version);
				patch = VK_VERSION_PATCH(driver_version);
				PCS_INFO("[select_gpu_device] Driver version: %d.%d.%d", major, minor, patch);

				PCS_INFO("[select_gpu_device] Support fp16 packed: %s",
						 gpuInfo->support_fp16_packed() ? "Yes" : "No");
				PCS_INFO("[select_gpu_device] Support fp16 storage: %s",
						 gpuInfo->support_fp16_storage() ? "Yes" : "No");
				PCS_INFO("[select_gpu_device] Support fp16 arithmetic: %s",
						 gpuInfo->support_fp16_arithmetic() ? "Yes" : "No");

				PCS_INFO("[select_gpu_device] Support int8 packed: %s",
						 gpuInfo->support_int8_packed() ? "Yes" : "No");
				PCS_INFO("[select_gpu_device] Support int8 storage: %s",
						 gpuInfo->support_int8_storage() ? "Yes" : "No");
				PCS_INFO("[select_gpu_device] Support int8 arithmetic: %s",
						 gpuInfo->support_int8_arithmetic() ? "Yes" : "No");
			}

			return gpu_select;
		}
	}
}