#ifndef POINT_CLOUD_TO_LOD_H_
#define POINT_CLOUD_TO_LOD_H_

#include <src/io/pointCloudReader.h>

#include <string>
#include <memory>

namespace d3s {
	namespace pcs {
		namespace io {

			class PointCloudToLOD
			{
			public:
				///////////////////////////////////////
				// constructors and destructor

				PointCloudToLOD();

				~PointCloudToLOD();

				///////////////////////////////////////
				// public member functions

				bool Export(const std::string& input,
							const std::string& output,
							const std::string& exportMode,
							int tileSize,
							int maxPointNumPerOneNode,
							int maxTreeDepth,
							float lodRatio,
							float pointSize,
							std::string colorMode);

			private:
				///////////////////////////////////////
				// private member functions

				static bool LoadPointsForOneTile(std::shared_ptr<PointCloudReader> pointsReader,
												 std::vector<PointRGBI>& lstPoints,
												 size_t tileSize,
												 size_t processedPoints);

				static bool ExportSRS(const std::string& srs, const std::string& filePath);
			};

		}
	}
}

#endif // POINT_CLOUD_TO_LOD_H_