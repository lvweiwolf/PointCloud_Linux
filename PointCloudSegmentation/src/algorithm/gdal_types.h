#ifndef GDAL_TYPES_H_
#define GDAL_TYPES_H_

#include <string>

namespace d3s {
	namespace pcs {
		namespace gdal {

			// GDAL Error wrap
			//////////////////////////////////////////////////////////////////////////
			enum class GDALError
			{
				None,
				NotOpen,
				CantOpen,
				NoData,
				InvalidBand,
				BadBand,
				NoTransform,
				NotInvertible,
				CantReadBlock,
				InvalidDriver,
				DriverNotFound,
				CantCreate,
				InvalidOption,
				CantWriteBlock,
				InvalidType
			};

			struct InvalidBand
			{
			};
			struct BadBand
			{
			};
			struct CantReadBlock
			{
			};
			struct CantWriteBlock
			{
				CantWriteBlock() {}

				CantWriteBlock(const std::string& w) : what(w) {}

				std::string what;
			};


			// GDAL Type wrap
			//////////////////////////////////////////////////////////////////////////
			enum class BaseType
			{
				None = 0x000,
				Signed = 0x100,
				Unsigned = 0x200,
				Floating = 0x400
			};

			inline BaseType fromName(std::string name)
			{
				if (name == "signed")
					return BaseType::Signed;
				else if (name == "unsigned")
					return BaseType::Unsigned;
				else if (name == "floating" || name == "float")
					return BaseType::Floating;
				return BaseType::None;
			}

			inline std::string toName(BaseType b)
			{
				switch (b)
				{
				case BaseType::Signed:
					return "signed";
				case BaseType::Unsigned:
					return "unsigned";
				case BaseType::Floating:
					return "floating";
				default:
					return "";
				}
			}

			enum class Type
			{
				None = 0,
				Unsigned8 = unsigned(BaseType::Unsigned) | 1,
				Signed8 = unsigned(BaseType::Signed) | 1,
				Unsigned16 = unsigned(BaseType::Unsigned) | 2,
				Signed16 = unsigned(BaseType::Signed) | 2,
				Unsigned32 = unsigned(BaseType::Unsigned) | 4,
				Signed32 = unsigned(BaseType::Signed) | 4,
				Unsigned64 = unsigned(BaseType::Unsigned) | 8,
				Signed64 = unsigned(BaseType::Signed) | 8,
				Float = unsigned(BaseType::Floating) | 4,
				Double = unsigned(BaseType::Floating) | 8
			};


			inline std::string interpretationName(Type dimtype)
			{
				switch (dimtype)
				{
				case Type::None:
					return "unknown";
				case Type::Signed8:
					return "int8_t";
				case Type::Signed16:
					return "int16_t";
				case Type::Signed32:
					return "int32_t";
				case Type::Signed64:
					return "int64_t";
				case Type::Unsigned8:
					return "uint8_t";
				case Type::Unsigned16:
					return "uint16_t";
				case Type::Unsigned32:
					return "uint32_t";
				case Type::Unsigned64:
					return "uint64_t";
				case Type::Float:
					return "float";
				case Type::Double:
					return "double";
				}
				return "unknown";
			}
		}
	}
}

#endif // GDAL_TYPES_H_