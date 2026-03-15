//stdafx.h
#include "pointCloudReader.h"

#include <lasreader.hpp>
#include <string>
#include "../core/private/gdalProcess.h"
#include "../utils/logging.h"

#define U8_CLAMP(n) (((n) <= U8_MIN) ? U8_MIN : (((n) >= U8_MAX) ? U8_MAX : ((U8)(n))))

namespace d3s {
	namespace pcs {
		namespace io {

			PointCloudReader::PointCloudReader(const std::string& filename)
				: _lasHeader(nullptr), _lasReader(nullptr), _epsg(0)
			{
				LASreadOpener lasFile;
				lasFile.set_file_name(filename.c_str());

				_lasReader = lasFile.open();

				if (!_lasReader)
				{
					PCS_ERROR("[PointCloudReader::PointCloudReader] 无法打开文件 '%s'.",
							  filename.c_str());

					return;
				}

				_lasHeader = &_lasReader->header;
				CHECK(_lasHeader);

				int epsg_code = -1;

				if (_lasHeader->vlr_geo_keys && _lasHeader->vlr_geo_key_entries)
				{
					// 打印坐标投影信息
					for (U16 i = 0; i < _lasHeader->vlr_geo_keys->number_of_keys; ++i)
					{
						if (_lasHeader->vlr_geo_key_entries[i].key_id == 3072)
						{
							epsg_code = _lasHeader->vlr_geo_key_entries[i].value_offset;
							break;
						}
					}
				}

				if (epsg_code > 0)
				{
					std::string strWKT = getWktFromEPSGCode(epsg_code);
					_epsg = epsg_code;

					PCS_INFO("[PointCloudReader::PointCloudReader] 坐标系 EPSG: %d", epsg_code);
					PCS_INFO("[PointCloudReader::PointCloudReader] 坐标系 WKT: %s", strWKT.c_str());
				}


				osg::Vec3d min, max, offset;
				max[0] = _lasHeader->max_x;
				max[1] = _lasHeader->max_y;
				max[2] = _lasHeader->max_z;

				min[0] = _lasHeader->min_x;
				min[1] = _lasHeader->min_y;
				min[2] = _lasHeader->min_z;

				offset[0] = _lasHeader->x_offset;
				offset[1] = _lasHeader->y_offset;
				offset[2] = _lasHeader->z_offset;

				_center = (max + min) / 2.0;
				_offset = osg::Vec3(min.x(), min.y(), min.z());
				//	_offset = offset;

				PCS_INFO("[PointCloudReader::PointCloudReader] Offset: (%lf, %lf, %lf)",
						 _offset.x(),
						 _offset.y(),
						 _offset.z());

				_bbox = osg::BoundingBox(min, max);
			}

			PointCloudReader::~PointCloudReader() {}

			bool PointCloudReader::IsValid() { return _lasReader != nullptr; }


			bool PointCloudReader::ReadNextPoint(PointRGBI& point)
			{
				if (!_lasReader)
					return false;

				if (_lasReader->read_point())
				{
					// 减去偏移量
					point.P[0] = _lasReader->point.get_x() - _offset.x();
					point.P[1] = _lasReader->point.get_y() - _offset.y();
					point.P[2] = _lasReader->point.get_z() - _offset.z();

					// 如果存在颜色
					if (_lasReader->point.have_rgb)
					{
						point.C[0] = _lasReader->point.rgb[0];
						point.C[1] = _lasReader->point.rgb[1];
						point.C[2] = _lasReader->point.rgb[2];
					}

					point.I = _lasReader->point.classification;

					return true;
				}

				else
				{
					return false;
				}
			}


			bool PointCloudReader::ReadNextPoint(PointPCLH& point)
			{
				if (!_lasReader)
					return false;

				if (_lasReader->read_point())
				{
					// 减去偏移量
					point.data[0] = _lasReader->point.get_x() - _offset.x();
					point.data[1] = _lasReader->point.get_y() - _offset.y();
					point.data[2] = _lasReader->point.get_z() - _offset.z();

					// 如果存在颜色
					if (_lasReader->point.have_rgb)
					{
						if (_lasReader->point.rgb[0] > 255)
							point.r = U8_CLAMP(_lasReader->point.rgb[0] / 256);
						else
							point.r = _lasReader->point.rgb[0];

						if (_lasReader->point.rgb[1] > 255)
							point.g = U8_CLAMP(_lasReader->point.rgb[1] / 256);
						else
							point.g = _lasReader->point.rgb[1];

						if (_lasReader->point.rgb[2] > 255)
							point.b = U8_CLAMP(_lasReader->point.rgb[2] / 256);
						else
							point.b = _lasReader->point.rgb[2];
					}

					point.label = _lasReader->point.classification;

					return true;
				}

				else
				{
					return false;
				}
			}


			bool PointCloudReader::ReadNextPoint2(PointPCLH& point)
			{
				if (!_lasReader)
					return false;

				if (_lasReader->read_point())
				{
					point.data[0] = _lasReader->point.get_x()- _offset.x();
					point.data[1] = _lasReader->point.get_y()- _offset.y();
					point.data[2] = _lasReader->point.get_z()- _offset.z();
					point.label = _lasReader->point.classification;

					return true;
				}
				else
				{
					return false;
				}
			}

			bool PointCloudReader::Seek(std::size_t pos)
			{
				if (!_lasReader)
					return false;

				_lasReader->seek(pos);

				return true;
			}

			size_t PointCloudReader::GetNumOfPoints()
			{
				unsigned int numPoints = _lasHeader ? _lasHeader->number_of_point_records : 0;
				return numPoints;
			}

		}
	}
}