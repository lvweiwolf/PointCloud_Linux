//////////////////////////////////////////////////////////////////////
// 文件名称：Endian.h
// 功能描述：数据编码
// 创建标识：吕伟	2022/8/11
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef ENDIAN_H_
#define ENDIAN_H_

#include <vector>
#include <algorithm>
#include <iostream>

namespace d3s {
	namespace pcs {

		inline bool IsLittleEndian()
		{
			return true;
		}

		template <typename T>
		T ReverseBytes(const T& data)
		{
			T data_reversed = data;
			std::reverse(reinterpret_cast<char*>(&data_reversed),
						 reinterpret_cast<char*>(&data_reversed) + sizeof(T));
			return data_reversed;
		}

		template <typename T>
		T NativeToLittleEndian(const T x)
		{
			if (IsLittleEndian())
			{
				return x;
			}
			else
			{
				return ReverseBytes(x);
			}
		}

		template <typename T>
		T LittleEndianToNative(const T x)
		{
			if (IsLittleEndian())
			{
				return x;
			}
			else
			{
				return ReverseBytes(x);
			}
		}


		template <typename T>
		void WriteBinaryLittleEndian(std::ostream* stream, const T& data)
		{
			const T data_little_endian = NativeToLittleEndian(data);
			stream->write(reinterpret_cast<const char*>(&data_little_endian), sizeof(T));
		}

		template <typename T>
		void WriteBinaryLittleEndian(std::ostream* stream, const std::vector<T>& data)
		{
			for (const auto& elem : data)
			{
				WriteBinaryLittleEndian<T>(stream, elem);
			}
		}

		template <typename T>
		T ReadBinaryLittleEndian(std::istream* stream)
		{
			T data_little_endian;
			stream->read(reinterpret_cast<char*>(&data_little_endian), sizeof(T));
			return LittleEndianToNative(data_little_endian);
		}

		template <typename T>
		void ReadBinaryLittleEndian(std::istream* stream, std::vector<T>* data)
		{
			for (size_t i = 0; i < data->size(); ++i)
			{
				(*data)[i] = ReadBinaryLittleEndian<T>(stream);
			}
		}
	}
}

#endif // ENDIAN_H_