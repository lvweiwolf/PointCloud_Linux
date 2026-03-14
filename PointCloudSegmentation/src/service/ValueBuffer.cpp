//stdafx.h
#include "ValueBuffer.h"

#include "../core/private/gdalProcess.h"
#include "../utils/logging.h"
namespace d3s {
	namespace pcs {

		CRoadVectorizeBuffer::CRoadVectorizeBuffer() : _epsg(-1) {}

		CRoadVectorizeBuffer::~CRoadVectorizeBuffer() { Clear(); }

		size_t CRoadVectorizeBuffer::GetBytes() const
		{
			size_t numBytes = 0;

			for (size_t i = 0; i < _roadDS.size(); ++i)
			{
				VectorLineString* road = _roadDS.at(i);
				CHECK(road);

				numBytes += road->GetBytes();
			}

			return numBytes;
		}

		void* CRoadVectorizeBuffer::GetPtr() const
		{
			if (_roadDS.empty())
				return nullptr;

			return (void*)_roadDS.at(0);
		}

		const d3s::pcs::VectorLineString* CRoadVectorizeBuffer::GetRoad(size_t i) const
		{
			if (i < 0 || i >= _roadDS.size())
				return nullptr;

			return _roadDS.at(i);
		}

		void CRoadVectorizeBuffer::ReadShapefile(const std::string& filename)
		{
			Clear();

			return readRoadDataset(filename, _roadDS, _epsg);
		}

		void CRoadVectorizeBuffer::ReadCompressFile(const std::string& filename)
		{
			Clear();

			return readRoadDataset2(filename, _roadDS, _epsg);
		}

		void CRoadVectorizeBuffer::WriteCompressFile(const std::string& filename) 
		{
			writeRoadDataset(_roadDS, _epsg, filename);
		}

		void CRoadVectorizeBuffer::Clear() 
		{
			for (size_t i = 0; i < _roadDS.size(); ++i)
			{
				VectorLineString* road = _roadDS.at(i);

				// Ïú»ÙÄÚ´æ
				if (road)
					delete road;
			}

			_roadDS.clear();
		}

	}
}