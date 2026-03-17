#include <src/io/rasterWriter.h>

#include <src/algorithm/gdal_raster.h>

namespace d3s {
	namespace pcs {

		RasterWriter::RasterWriter(const std::string& filename) : _filename(filename)
		{
			_drivername = "GTiff";
			_dataType = gdal::Type::Double;
			_noData = std::numeric_limits<double>::quiet_NaN();
		}

		RasterWriter::~RasterWriter() {}

		void RasterWriter::write(const Rasterd* r, const std::string& srs)
		{
			_rasters.push_back(r);

			if (_rasters.empty())
				return;

			if (_rasterNames.size() == 1 && _rasterNames[0] == "")
				_rasterNames.clear();

			for (const std::string& name : _rasterNames)
			{
				std::string info = "Raster '" + name + "' not found.";
				CHECK_MSG(std::find_if(_rasters.begin(),
									   _rasters.end(),
									   [name](const Rasterd* r) { return r->name() == name; }) ==
							  _rasters.end(),
						  info.c_str());
			}


			// Stick rasters whose limits match the first raster in our final raster list.
			std::vector<const Rasterd*> rasters;

			for (const Rasterd* r : _rasters)
			{
				if (r->extents() != _rasters.front()->extents())
				{
					PCS_ERROR("[RasterWriter] Ignoring raster '%s'. Raster limits don't match that "
							  "of raster '%s'.",
							  r->name(),
							  _rasters.front()->name());

					continue;
				}

				rasters.push_back(r);
			}

			std::array<double, 6> pixelToPos;
			RasterExtents extents = rasters.front()->extents();
			pixelToPos[0] = extents.x;
			pixelToPos[1] = extents.resolution;
			pixelToPos[2] = 0;
			pixelToPos[3] = extents.y + (extents.resolution * extents.height);
			pixelToPos[4] = 0;
			pixelToPos[5] = -extents.resolution;

			gdal::Raster rasterFile(_filename, _drivername, srs, pixelToPos);

			gdal::GDALError err = rasterFile.open(extents.width,
												  extents.height,
												  rasters.size(),
												  _dataType,
												  _noData,
												  _options);

			CHECK_MSG(err == gdal::GDALError::None, rasterFile.errorMsg().c_str());

			int bandNum = 1;

			for (const Rasterd* r : rasters)
			{
				err = rasterFile.writeBand(r->begin(), r->initializer(), bandNum++, r->name());
				CHECK_MSG(err == gdal::GDALError::None, rasterFile.errorMsg().c_str());
			}

			// getMetadata().addList("filename", m_filename);
		}
	}
}