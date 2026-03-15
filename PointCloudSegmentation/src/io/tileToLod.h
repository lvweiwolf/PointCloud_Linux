#pragma once
#include <float.h>
#include <osg/BoundingBox>
#include <osg/Geode>
#include <osg/Vec4>
#include <vector>
#include "../core/pointTypes.h"
namespace osg {
	class Geode;
}

namespace d3s {
	namespace pcs {
		namespace io {
			enum AxisType { X = 0, Y, Z };

			struct AxisInfo
			{
				AxisInfo() : aixType(0), max(-DBL_MAX), min(DBL_MAX) {}
				int aixType;
				double max;
				double min;
			};


			enum ColorMode {
				Debug = 0,
				RGB = 1,
				IntensityGrey = 2,
				IntensityBlueWhiteRed = 3,
				IntensityHeightBlend = 4
			};

			enum ExportMode { OSGB = 0, _3MX = 1 };

			class TileToLOD
			{
			public:
				TileToLOD(unsigned int maxTreeLevel,
						  unsigned int maxPointNumPerOneNode,
						  double lodRatio,
						  float pointSize,
						  osg::BoundingBox boundingBoxGlobal,
						  ColorMode colorMode)
				{
					_maxTreeLevel = maxTreeLevel;
					_maxPointNumPerOneNode = maxPointNumPerOneNode;
					_lodRatio = lodRatio;
					_pointSize = pointSize;
					_boundingBoxGlobal = boundingBoxGlobal;
					_colorMode = colorMode;
					CreateColorBar();
				}

				void CreateColorBar();

				bool Generate(const std::vector<PointRGBI>* pointSet,
							  const std::string& saveFilePath,
							  const std::string& strBlock,
							  ExportMode exportMode,
							  osg::BoundingBox& boundingBoxLevel0);

			protected:
				AxisInfo FindMaxAxis(osg::BoundingBox boundingBox,
									 osg::BoundingBox& boundingBoxLeft,
									 osg::BoundingBox& boundingBoxRight);

				bool BuildNode(const std::vector<PointRGBI>* pointSet,
							   std::vector<unsigned int>& pointIndex,
							   osg::BoundingBox boundingBox,
							   osg::BoundingBox boundingBoxLevel0,
							   const std::string& saveFilePath,
							   const std::string& strBlock,
							   unsigned int level,
							   unsigned int childNo,
							   ExportMode exportMode);

				osg::Geode* MakeNodeGeode(const std::vector<PointRGBI>* pointSet,
										  std::vector<unsigned int>& pointIndex,
										  ExportMode exportMode);

			private:
				unsigned int _maxTreeLevel;
				unsigned int _maxPointNumPerOneNode;
				double _lodRatio;
				float _pointSize;
				osg::BoundingBox _boundingBoxGlobal;
				ColorMode _colorMode;
				osg::Vec4 _colorBar[256];
			};

		}
	}
}
