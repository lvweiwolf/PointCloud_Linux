/////////////////////////////////////////////////////////////////////
// 文件名称：hash.h
// 功能描述：哈希索引工具
// 创建标识：吕伟	2022/4/22
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef HASH_H_
#define HASH_H_

#include <functional>

namespace d3s {
	namespace pcs {

		template <typename T>
		struct hash_eigen
		{
			std::size_t operator()(T const& matrix) const
			{
				std::size_t seed = 0;
				for (int i = 0; i < (int)matrix.size(); i++)
				{
					auto elem = *(matrix.data() + i);
					seed ^= std::hash<int>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
				}
				return seed;
			}
		};
	}
}

#endif // HASH_H_