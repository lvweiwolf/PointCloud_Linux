//////////////////////////////////////////////////////////////////////
// 文件名称：boundingbox2d.h
// 功能描述：2D包围盒
// 创建标识：吕伟	2022/10/21
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef BOUNDINGBOX2D_H_
#define BOUNDINGBOX2D_H_

#include <osg/Vec2>
#include <osg/Vec2d>

namespace d3s {
	namespace pcs {

		template <typename VT>
		class BoundingBox2DImpl
		{
		public:
			typedef VT vec_type;
			typedef typename VT::value_type value_type;

			vec_type _min;
			vec_type _max;

			inline BoundingBox2DImpl() : _min(FLT_MAX, FLT_MAX), _max(-FLT_MAX, -FLT_MAX) {}

			template <typename BT>
			inline BoundingBox2DImpl(const BoundingBox2DImpl<BT>& bb) : _min(bb._min), _max(bb._max)
			{
			}

			inline BoundingBox2DImpl(value_type xmin,
									 value_type ymin,
									 value_type xmax,
									 value_type ymax)
				: _min(xmin, ymin), _max(xmax, ymax)
			{
			}

			inline BoundingBox2DImpl(const vec_type& min, const vec_type& max)
				: _min(min), _max(max)
			{
			}

			inline void init()
			{
				_min.set(FLT_MAX, FLT_MAX);
				_max.set(-FLT_MAX, -FLT_MAX);
			}

			inline bool operator==(const BoundingBox2DImpl& rhs) const
			{
				return _min == rhs._min && _max == rhs._max;
			}
			inline bool operator!=(const BoundingBox2DImpl& rhs) const
			{
				return _min != rhs._min || _max != rhs._max;
			}


			inline bool valid() const { return _max.x() >= _min.x() && _max.y() >= _min.y(); }

			inline void set(value_type xmin, value_type ymin, value_type xmax, value_type ymax)
			{
				_min.set(xmin, ymin);
				_max.set(xmax, ymax);
			}

			inline void set(const vec_type& min, const vec_type& max)
			{
				_min = min;
				_max = max;
			}

			inline value_type& xMin() { return _min.x(); }
			inline value_type xMin() const { return _min.x(); }

			inline value_type& yMin() { return _min.y(); }
			inline value_type yMin() const { return _min.y(); }

			inline value_type& xMax() { return _max.x(); }
			inline value_type xMax() const { return _max.x(); }

			inline value_type& yMax() { return _max.y(); }
			inline value_type yMax() const { return _max.y(); }

			inline const vec_type center() const { return (_min + _max) * 0.5; }

			inline value_type radius() const { return sqrt(radius2()); }

			inline value_type radius2() const { return 0.25 * ((_max - _min).length2()); }

			inline const vec_type corner(unsigned int pos) const
			{
				/*return vec_type(pos & 1 ? _max.x() : _min.x(), pos & 2 ? _max.y() : _min.y());*/

				if (pos == 0)
				{
					return vec_type(_min.x(), _min.y());
				}
				else if (pos == 1)
				{
					return vec_type(_max.x(), _min.y());
				}
				else if (pos == 2)
				{
					return vec_type(_max.x(), _max.y());
				}
				else if (pos == 3)
				{
					return vec_type(_min.x(), _max.y());
				}
				else
					return vec_type();
			}

			inline void expandBy(const vec_type& v)
			{
				if (v.x() < _min.x())
					_min.x() = v.x();
				if (v.x() > _max.x())
					_max.x() = v.x();

				if (v.y() < _min.y())
					_min.y() = v.y();
				if (v.y() > _max.y())
					_max.y() = v.y();
			}

			inline void expandBy(value_type x, value_type y)
			{
				if (x < _min.x())
					_min.x() = x;
				if (x > _max.x())
					_max.x() = x;

				if (y < _min.y())
					_min.y() = y;
				if (y > _max.y())
					_max.y() = y;
			}

			void expandBy(const BoundingBox2DImpl& bb)
			{
				if (!bb.valid())
					return;

				if (bb._min.x() < _min.x())
					_min.x() = bb._min.x();
				if (bb._max.x() > _max.x())
					_max.x() = bb._max.x();

				if (bb._min.y() < _min.y())
					_min.y() = bb._min.y();
				if (bb._max.y() > _max.y())
					_max.y() = bb._max.y();
			}


			BoundingBox2DImpl intersect(const BoundingBox2DImpl& bb) const
			{
				return BoundingBox2DImpl(osg::maximum(xMin(), bb.xMin()),
										 osg::maximum(yMin(), bb.yMin()),
										 osg::minimum(xMax(), bb.xMax()),
										 osg::minimum(yMax(), bb.yMax()));
			}

			bool intersects(const BoundingBox2DImpl& bb) const
			{
				return osg::maximum(xMin(), bb.xMin()) <= osg::minimum(xMax(), bb.xMax()) &&
					   osg::maximum(yMin(), bb.yMin()) <= osg::minimum(yMax(), bb.yMax());
			}

			inline bool contains(const vec_type& v) const
			{
				return valid() && (v.x() >= _min.x() && v.x() <= _max.x()) &&
					   (v.y() >= _min.y() && v.y() <= _max.y());
			}


			inline bool contains(const vec_type& v, value_type epsilon) const
			{
				return valid() &&
					   ((v.x() + epsilon) >= _min.x() && (v.x() - epsilon) <= _max.x()) &&
					   ((v.y() + epsilon) >= _min.y() && (v.y() - epsilon) <= _max.y());
			}
		};

		typedef BoundingBox2DImpl<osg::Vec2f> BoundingBox2Df;
		typedef BoundingBox2DImpl<osg::Vec2d> BoundingBox2Dd;

#ifdef OSG_USE_FLOAT_BOUNDINGBOX
		typedef BoundingBox2Df BoundingBox2D;
#else
		typedef BoundingBox2Dd BoundingBox2D;
#endif
	}
}


#endif // BOUNDINGBOX2D_H_