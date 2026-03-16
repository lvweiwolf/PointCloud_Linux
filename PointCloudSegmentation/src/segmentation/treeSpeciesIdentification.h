//////////////////////////////////////////////////////////////////////
// 文件名称：treeSpeciesIdentification.h
// 功能描述：树种识别
// 创建标识：吕伟	2023/8/2
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef TREE_SPECIES_IDENTIFICATION_H_
#define TREE_SPECIES_IDENTIFICATION_H_

#include <src/utils/threading.h>
#include <src/core/pointTypes.h>

namespace d3s {
	namespace pcs {

		struct TreeSpeciesIndendtifyOptions
		{
			std::string cache_dir;	// 缓存目录
			std::string ncnn_param; // ncnn模型参数
			std::string ncnn_model; // ncnn模型权重
			unsigned int num_classes;

			std::vector<std::string> dom_files;

			// 打印参数
			void Print();
		};

		class TreeSpeciesIdentify : public Thread
		{
		public:
			TreeSpeciesIdentify(const TreeSpeciesIndendtifyOptions& options,
								PointCloudViewPtr input,
								const std::vector<int>& indices,
								std::function<void(double)> progress_func);

			virtual ~TreeSpeciesIdentify();


		private:
			virtual void Run() override;


			TreeSpeciesIndendtifyOptions _options; // 树种识别参数
			PointCloudViewPtr _input;			   // 输入点云数据
			std::vector<int> _indices;			   // 待处理点云索引

			std::function<void(double)> _progress_func;
		};
	}
}

#endif // TREE_SPECIES_IDENTIFICATION_H_
