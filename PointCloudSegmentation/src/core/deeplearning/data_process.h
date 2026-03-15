#ifndef _DATA_PROCESS_H
#define _DATA_PROCESS_H

#include "../pointTypes.h"
#include <vector>
#include <cstddef>
namespace d3s {
	namespace pcs {

		// PointNet/PointNet++ 点云分割数据处理
		class DataProcess
		{
		public:	
			DataProcess(PointCloudViewPtr cloud);

			DataProcess(PointCloudViewPtr cloud, const std::vector<int>& indices);

			~DataProcess() {}

			/**
			 *  @brief	  按照采样点数和相关参数生成点云分块
			 *
			 *  @param    size_t numPoint									采样点数
			 *  @param    double blockSize									分块大小，以米为单位
			 *  @param    double stride										原数据滑动窗口大小
			 *  @param    std::vector<std::vector<int>> & blockIndices		分块点云索引
			 *  @param    std::vector<PointCloudViewPtr> & blocks			分块点云
			 * 
			 *	注意: blocks 是点云数据指针数组，点云数据内存需要调用者自行销毁
			 *
			 *  @return   void
			 */
			void GenerateBlocks(size_t numPoint,
								double blockSize,
								double stride,
								std::vector<std::vector<int>>& blockIndices,
								std::vector<PointCloudViewPtr>& blocks);
			
			void Normalized(std::vector<PointCloudViewPtr>& blocks);

		private:
			PointCloudViewPtr _cloud;
			std::vector<int> _indices;
		};

	}
}

#endif // _DATA_PROCESS_H
