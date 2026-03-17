/////////////////////////////////////////////////////////////////////
// 文件名称：rasterWriter.h
// 功能描述：光栅数据写入
// 创建标识：吕伟	2022/4/9
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef RASTER_WRITER_H_
#define RASTER_WRITER_H_

#include <src/algorithm/raster.h>
#include <src/algorithm/gdal_types.h>

namespace d3s {
	namespace pcs {

		class RasterWriter
		{
		public:
			RasterWriter(const std::string& filename);

			virtual ~RasterWriter();

			// 文件名
			void setFileName(const std::string& filename) { _filename = filename; }
			const std::string& getFileName() const { return _filename; }

			// GDAL驱动
			void setDriver(const std::string& driver) { _drivername = driver; }
			const std::string& getDriver() const { return _drivername; }

			// GDAL 选项
			void setOptions(const std::vector<std::string>& options) { _options = options; }
			const std::vector<std::string>& getOptions() const { return _options; }

			// 数据类型
			void setDataType(gdal::Type dataType) { _dataType = dataType; }
			gdal::Type getDataType() const { return _dataType; }

			void write(const Rasterd* r, const std::string& srs);

		private:
			std::string _filename;
			std::string _drivername;
			std::vector<std::string> _options;
			std::vector<std::string> _rasterNames;
			gdal::Type _dataType;
			double _noData;

			std::vector<const Rasterd*> _rasters;
		};

	}
}

#endif // RASTER_WRITER_H_