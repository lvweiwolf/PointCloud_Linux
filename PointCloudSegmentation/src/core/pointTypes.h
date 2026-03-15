#ifndef POINT_TYPES_H_
#define POINT_TYPES_H_

#if defined(_WIN32) || defined(_WIN64)
#define ALIGN_16 __declspec(align(16))
#define ALIGN_8 __declspec(align(8))
#elif defined(__linux__)
#define ALIGN_16 __attribute__((aligned(16)))
#define ALIGN_8 __attribute__((aligned(8)))
#else
#define ALIGN_16
#define ALIGN_8
#endif

#include <Eigen/Core>

#include <osg/Array>
#include <osg/Vec3d>
#include <osg/BoundingBox>


namespace d3s {
	namespace pcs {

		typedef int ClassificationType;

		typedef Eigen::Map<Eigen::Array3f> Array3fMap;
		typedef const Eigen::Map<const Eigen::Array3f> Array3fMapConst;
		typedef Eigen::Map<Eigen::Array4f, Eigen::Aligned> Array4fMap;
		typedef const Eigen::Map<const Eigen::Array4f, Eigen::Aligned> Array4fMapConst;
		typedef Eigen::Map<Eigen::Vector3f> Vector3fMap;
		typedef const Eigen::Map<const Eigen::Vector3f> Vector3fMapConst;
		typedef Eigen::Map<Eigen::Vector4f, Eigen::Aligned> Vector4fMap;
		typedef const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> Vector4fMapConst;

		struct PointRGBI
		{
			PointRGBI() : P(0, 0, 0), C(0, 0, 0), I(255) {}

			osg::Vec3d P;
			osg::Vec3ub C;
			unsigned char I;
		};

		struct _PointXYZ_RGB_L_HAG
		{
			inline _PointXYZ_RGB_L_HAG()
				: x(0.0f),
				  y(0.0f),
				  z(0.0f),
				  w(0.0f),
				  r(0),
				  g(0),
				  b(0),
				  a(255),
				  label(255),
				  hag(0.0f)
			{
			}

			// XYZ
			union ALIGN_16
			{
				float data[4];
				struct
				{

					float x;
					float y;
					float z;
					float w;
				};
			};

			inline Vector3fMap getVector3fMap() { return (Vector3fMap(data)); }
			inline Vector3fMapConst getVector3fMap() const { return (Vector3fMapConst(data)); }
			inline Vector4fMap getVector4fMap() { return (Vector4fMap(data)); }
			inline Vector4fMapConst getVector4fMap() const { return (Vector4fMapConst(data)); }
			inline Array3fMap getArray3fMap() { return (Array3fMap(data)); }
			inline Array3fMapConst getArray3fMap() const { return (Array3fMapConst(data)); }
			inline Array4fMap getArray4fMap() { return (Array4fMap(data)); }
			inline Array4fMapConst getArray4fMap() const { return (Array4fMapConst(data)); }

			// RGB
			union
			{
				union
				{
					struct
					{
						uint8_t b;
						uint8_t g;
						uint8_t r;
						uint8_t a;
					};
					float rgb;
				};
				uint32_t rgba;
			};

			// L
			uint32_t label;

			// HAG
			float hag; // height above ground



			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		struct ALIGN_16 PointPCLH : public _PointXYZ_RGB_L_HAG
		{
			inline PointPCLH()
			{
				x = y = z = 0.0f;
				data[3] = 0.0f;
				r = g = b = 0;
				a = 255;
				label = 255;
				hag = 0.0f;
			}

			inline PointPCLH(const _PointXYZ_RGB_L_HAG& p)
			{
				x = p.x;
				y = p.y;
				z = p.z;
				data[3] = 0.0f;
				rgba = p.rgba;
				label = p.label;
				hag = p.hag;
			}

			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};


		template <typename PointT>
		class PointCloudView
		{
		public:
			PointCloudView() : points(), normalized(false), hag_min(DBL_MAX), hag_max(-DBL_MAX) {}

			PointCloudView(PointCloudView<PointT>& pc) : points() { *this = pc; }
			PointCloudView(const PointCloudView<PointT>& pc) : points() { *this = pc; }
			PointCloudView(const PointCloudView<PointT>& pc, const std::vector<int>& indices)
				: points(indices.size())
			{
				CHECK(indices.size() <= pc.size());
				for (size_t i = 0; i < indices.size(); i++)
					points[i] = pc.points[indices[i]];

				normalized = pc.normalized;
				hag_min = pc.hag_min;
				hag_max = pc.hag_max;
			}

			virtual ~PointCloudView() {}

			typedef PointT PointType;
			typedef std::vector<PointT, Eigen::aligned_allocator<PointT>> PointCloud;

			// µü´úÆ÷
			typedef typename PointCloud::iterator iterator;
			typedef typename PointCloud::const_iterator const_iterator;
			inline iterator begin() { return (points.begin()); }
			inline iterator end() { return (points.end()); }
			inline const_iterator begin() const { return (points.begin()); }
			inline const_iterator end() const { return (points.end()); }

			// ÈÝÁ¿
			inline size_t size() const { return (points.size()); }
			inline void reserve(size_t n) { points.reserve(n); }
			inline bool empty() const { return points.empty(); }

			/**
			 *  @brief    ÖØÐÂÉèÖÃ´óÐ¡
			 *
			 *  @param    size_t n
			 *
			 *  @return   void
			 */
			inline void resize(size_t n) { points.resize(n); }

			// ÔªËØ·ÃÎÊ
			inline const PointT& operator[](size_t n) const { return (points[n]); }
			inline PointT& operator[](size_t n) { return (points[n]); }
			inline const PointT& at(size_t n) const { return (points.at(n)); }
			inline PointT& at(size_t n) { return (points.at(n)); }
			inline const PointT& front() const { return (points.front()); }
			inline PointT& front() { return (points.front()); }
			inline const PointT& back() const { return (points.back()); }
			inline PointT& back() { return (points.back()); }


			/**
			 *  @brief    ²åÈëÐÂµãµ½µãÔÆ
			 *
			 *  @param    const PointT & pt
			 *
			 *  @return   void
			 */
			inline void push_back(const PointT& pt) { points.push_back(pt); }

			inline iterator insert(iterator position, const PointT& pt)
			{
				iterator it = points.insert(position, pt);
				return (it);
			}

			inline void clear() { points.clear(); }

			PointCloud points;

			osg::BoundingBox bbox;
			osg::Vec3d offset_xyz;

			int epsg;
			bool normalized; // ÊÇ·ñ¹éÒ»»¯
			double hag_min;
			double hag_max;
		};

		typedef PointCloudView<PointPCLH>* PointCloudViewPtr;
		typedef const PointCloudView<PointPCLH>* PointCloudViewConstPtr;
	}
}

#endif // POINT_TYPES_H_