#include <src/algorithm/raster.h>

namespace d3s {
	namespace pcs {
		// This should probably call expand().
		template <typename T>
		void Raster<T>::setExtents(const RasterExtents& extents)
		{
			_extents = extents;
			DataVec dataVec(width() * height(), _initializer);
			_data.swap(dataVec);
		}


		template <typename T>
		void Raster<T>::expandToInclude(double x, double y)
		{
			int xi = xCell(x);
			int yi = yCell(y);

			if (xi >= 0 && yi >= 0 && xi < width() && yi < height())
				return;

			int w = (std::max)(width(), xi + 1);
			int h = (std::max)(height(), yi + 1);
			int xshift = (std::max)(-xi, 0);
			int yshift = (std::max)(-yi, 0);

			if (xshift)
				w += xshift;
			if (yshift)
				h += yshift;
			expand(w, h, xshift, yshift);
		}


		template <typename T>
		int Raster<T>::expand(int newWidth, int newHeight, int xshift, int yshift)
		{
			if (newWidth < width())
			{
				PCS_ERROR("扩展网格的宽度必须至少与现有网格的宽度相同.");
				return -1;
			}

			if (newHeight < height())
			{
				PCS_ERROR("扩展网格的高度必须至少与现有网格的高度相同.");
				return -1;
			}

			if (width() + xshift > newWidth || height() + yshift > newHeight)
			{
				PCS_ERROR("扩展网格不能超出现有网格的边界.");
				return -1;
			}

			if (newWidth == width() && newHeight == height())
				return true;

			_extents.x -= xshift * edgeLength();
			_extents.y -= yshift * edgeLength();

			// 光栅X/Y上下颠倒
			yshift = newHeight - (height() + yshift);

			auto dstIndex = [newWidth, xshift, yshift](size_t i, size_t j) {
				return ((yshift + j) * newWidth) + i + xshift;
			};

			// i，j位于光栅的内部，从左上角开始，并在光栅上下移动。
			DataVec& src = _data;
			DataVec dst(newWidth * newHeight, _initializer);
			for (int j = 0; j < height(); ++j)
			{
				size_t srcPos = index(0, j);
				size_t dstPos = dstIndex(0, j);
				std::copy(src.begin() + srcPos,
						  src.begin() + srcPos + width(),
						  dst.begin() + dstPos);
			}
			_data = std::move(dst);
			_extents.width = newWidth;
			_extents.height = newHeight;
			return true;
		}

		// 实例化光栅
		template class Raster<double>;

	}
}