/////////////////////////////////////////////////////////////////////
// 文件名称：raster.h
// 功能描述：栅格数据
// 创建标识：吕伟	2022/4/8
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef RASTER_H_
#define RASTER_H_

#include <src/utils/logging.h>

namespace d3s {
	namespace pcs {

		// 光栅数据范围
		//////////////////////////////////////////////////////////////////////////
		struct RasterExtents
		{
		public:
			RasterExtents(double x_, double y_, int width_, int height_, double resolution_)
				: x(x_), y(y_), width(width_), height(height_), resolution(resolution_)
			{
			}

			RasterExtents() : x(0), y(0), width(-1), height(-1), resolution(0) {}

			bool valid() const { return width > 0; }

		public:
			double x;
			double y;
			int width;
			int height;
			double resolution;
		};


		inline bool operator==(const RasterExtents& l, const RasterExtents& r)
		{
			return l.x == r.x && l.y == r.y && l.width == r.width && l.height == r.height &&
				   l.resolution == r.resolution;
		}

		inline bool operator!=(const RasterExtents& l, const RasterExtents& r) { return !(l == r); }



		// 这是一个适合粘贴到GDAL的原始光栅数据。X从右到左，Y从上到下
		//////////////////////////////////////////////////////////////////////////
		template <typename T>
		class Raster
		{
		public:
			using DataVec = std::vector<T>;
			using iterator = typename DataVec::iterator;
			using const_iterator = typename DataVec::const_iterator;

			Raster(const T& initializer = T()) : Raster("", initializer) {}

			Raster(const std::string& name, const T& initializer = T())
				: _name(name), _initializer(initializer)
			{
			}

			Raster(const RasterExtents& limits, const T& initializer = T())
				: Raster(limits, "", initializer)
			{
			}

			Raster(const RasterExtents& limits, const std::string& name, const T& initializer = T())
				: _name(name), _extents(limits), _initializer(initializer)
			{
				double memSize = ((double)width() * (double)height()) / (1024.0 * 1024.0);
				memSize *= sizeof(T);

				if (memSize > 2000.0)
					PCS_WARN("Raster分配内存过大！占用内存 %lf MB.", memSize);

				_data = DataVec(width() * height(), initializer);
			}

			std::string name() const { return _name; }

			int width() const { return _extents.width; }

			int height() const { return _extents.height; }

			double edgeLength() const { return _extents.resolution; }

			double xOrigin() const { return _extents.x; }

			double yOrigin() const { return _extents.y; }

			T initializer() const { return _initializer; }

			int xCell(double x) const { return std::floor((x - _extents.x) / _extents.resolution); }

			int yCell(double y) const { return std::floor((y - _extents.y) / _extents.resolution); }

			double xCellPos(size_t i) const { return _extents.x + (i + .5) * edgeLength(); }

			double yCellPos(size_t j) const { return _extents.y + (j + .5) * edgeLength(); }

			const T* data() const { return _data.data(); }

			T* data() { return _data.data(); }

			const T& at(int x, int y) const { return _data[index(x, height() - y - 1)]; }

			T& at(int x, int y) { return _data[index(x, height() - y - 1)]; }

			const T& at(size_t idx) const { return _data[idx]; }

			T& at(size_t idx) { return _data[idx]; }

			const T& operator[](size_t idx) const { return _data[idx]; }

			T& operator[](size_t idx) { return _data[idx]; }

			iterator begin() { return _data.begin(); }

			const_iterator begin() const { return _data.begin(); }

			iterator end() { return _data.end(); }

			const_iterator end() const { return _data.end(); }

			void expandToInclude(double x, double y);

			// ABELL - This should probably call expand().
			void setExtents(const RasterExtents& extents);

			const RasterExtents& extents() const { return _extents; }

			size_t size() const { return _data.size(); }

		private:
			std::string _name;
			RasterExtents _extents;
			DataVec _data;
			T _initializer;

			int expand(int w, int h, int xshift, int yshift);

			// You need to pass in internal (Y down) indices.
			size_t index(int i, int j) const { return (j * width()) + i; }
		};

		extern template class Raster<double>;
		typedef Raster<double> Rasterd;
	}
}

#endif // RASTER_H_