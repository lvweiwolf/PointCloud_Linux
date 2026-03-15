//stdafx.h
#include "pointCloudWriter.h"

#include <numeric>
#include <laswriter.hpp>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/impl/pcd_io.hpp>
//#include <pcl/conversions.h>

#include "geoProjectionConverter.h"

#include "../core/private/statistics.h"
#include "../core/private/gdalProcess.h"
#include "../utils/misc.h"
#include "../utils/stringutil.h"
#include "../utils/logging.h"

namespace d3s {
	namespace pcs {
		namespace io {
			PointCloudWriter::PointCloudWriter(const std::string& filename) : _filename(filename) {}

			PointCloudWriter::~PointCloudWriter() {}

			bool PointCloudWriter::Write(const PointCloudView<PointPCLH>& pcv)
			{
				if (pcv.empty())
					return false;

				std::vector<int> indices(pcv.size());
				std::iota(indices.begin(), indices.end(), 0);

				return Write(pcv, indices);
			}
	
			bool PointCloudWriter::Write(const PointCloudView<PointPCLH>& pcv,
										 const std::vector<int>& indices)
			{
				std::string filename, ext;
				SplitFileExtension(_filename, &filename, &ext);
				StringToLower(&ext);

				if (ext == ".las")
				{
					return WriteLAS(pcv, indices);
				}
				else if (ext == ".pcd")
				{
					return WritePCD(pcv, indices);
				}
				else
				{
					PCS_WARN("[PointCloudWriter] ��֧�ֵ����ݸ�ʽ.");
					return false;
				}
			}

			bool PointCloudWriter::WriteLAS(const PointCloudView<PointPCLH>& pcv,
											const std::vector<int>& indices)
			{
				if (pcv.empty())
					return false;

				LASwriteOpener laswriteopener;
				laswriteopener.set_file_name(_filename.c_str());

				// ����ֲ���Χ��
				osg::BoundingBox bbox;
				computeMinMax3D(pcv, indices, bbox);

				auto offset = pcv.offset_xyz;

				LASheader header;
				header.set_bounding_box(bbox.xMin() + offset.x(),
										bbox.yMin() + offset.y(),
										bbox.zMin() + offset.z(),
										bbox.xMax() + offset.x(),
										bbox.yMax() + offset.y(),
										bbox.zMax() + offset.z());

				header.x_scale_factor = 0.0001;
				header.y_scale_factor = 0.0001;
				header.z_scale_factor = 0.0001;
				header.x_offset = offset.x();
				header.y_offset = offset.y();
				header.z_offset = offset.z();
				header.point_data_format = 2;
				header.point_data_record_length = 26;

				if (pcv.epsg > 0)
				{
					int len = 0;
					char* wkt = nullptr;

					GeoProjectionConverter geoprojectionconverter;
					geoprojectionconverter.set_epsg_code(pcv.epsg);

					int number_of_keys;
					GeoProjectionGeoKeys* geo_keys = 0;
					int num_geo_double_params;
					double* geo_double_params = 0;
					bool projection_was_set = false;

					if (geoprojectionconverter.has_projection())
					{
						projection_was_set = geoprojectionconverter.get_geo_keys_from_projection(
							number_of_keys,
							&geo_keys,
							num_geo_double_params,
							&geo_double_params);
					}

					if (projection_was_set)
					{
						header.set_geo_keys(number_of_keys, (LASvlr_key_entry*)geo_keys);
						free(geo_keys);

						if (geo_double_params)
						{
							header.set_geo_double_params(num_geo_double_params, geo_double_params);
							free(geo_double_params);
						}
						else
						{
							header.del_geo_double_params();
						}

						header.del_geo_ascii_params();
					}
				}
				// �������Ƽ�¼��˾����
				strcpy(header.generating_software, "BOOWAY");
				LASwriter* laswriter = laswriteopener.open(&header);

				LASpoint laspoint;
				laspoint.init(&header,
							  header.point_data_format,
							  header.point_data_record_length,
							  0);


				if (!laswriter)
				{
					PCS_ERROR("[PointCloudWriter::PointCloudWriter] �޷�д���ļ� '%s'.",
							  _filename.c_str());

					return false;
				}

				bool writeComplete = true;

				for (size_t i = 0; i < indices.size(); ++i)
				{
					const auto& p = pcv.points[indices[i]];

					laspoint.set_x(p.x + offset.x());
					laspoint.set_y(p.y + offset.y());
					laspoint.set_z(p.z + offset.z());

					laspoint.set_R(p.r);
					laspoint.set_G(p.g);
					laspoint.set_B(p.b);
					// 558136 ������ã���������𳬹�3�֣���3�ּ����ϵ�����޷�������ʾ����ͼ�� ��ͼV1.6.0.19
					laspoint.set_user_data(p.label);

					if (!laswriter->write_point(&laspoint))
					{
						PCS_ERROR(
							"[PointCloudWriter::PointCloudWriter] д��� %d ����ʱ����������.",
							i);

						writeComplete = false;

						break;
					}

					laswriter->update_inventory(&laspoint);
				}


				laswriter->update_header(&header, true);
				I64 total_bytes = laswriter->close();

				delete laswriter;

				return writeComplete;
			}

			bool PointCloudWriter::WritePCD(const PointCloudView<PointPCLH>& pcv,
											const std::vector<int>& indices)
			{
				if (pcv.empty())
					return false;

				typedef pcl::PointXYZ PointT;

				pcl::PCDWriter writer;
				pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
				cloud->width = pcv.points.size();
				cloud->height = 1;
				cloud->points.reserve(pcv.points.size());

				auto offset = pcv.offset_xyz;

				for (size_t i = 0; i < pcv.points.size(); ++i)
				{
					const auto& p = pcv.points[indices[i]];

					cloud->points.push_back(
						PointT(p.x + offset.x(), p.y + offset.y(), p.z + offset.z()));
				}

				return writer.write<PointT>(_filename, *cloud, true) == 0;
			}

		}
	}
}

// 显式实例化 PCL 转换函数
//#include <pcl/conversions.h>
///template void pcl::toPCLPointCloud2<d3s::pcs::PointPCLH>(const pcl::PointCloud<d3s::pcs::PointPCLH>&, pcl::PCLPointCloud2&);