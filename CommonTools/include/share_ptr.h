
//////////////////////////////////////////////////////////////////////
// 文件名称：share_ptr.h
// 功能描述: 共享指针(复用开源代码)
// 创建标识：吴建峰 2025/01/17
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef SHAREPTR_H_
#define SHAREPTR_H_

#include <include/CommonToolsExport.h>

#include <memory>

namespace d3s {
	template <typename T>
	class observer_ptr;

	/** @addtogroup Platform
	 * @{
	 */

	/** Smart pointer for handling referenced counted objects.*/
	template <class T>
	class COMMONTOOLS_EXPORT share_ptr
	{
	public:
		typedef T element_type;

		share_ptr() : _ptr(0) {}
		share_ptr(T* ptr) : _ptr(ptr)
		{
			if (_ptr)
				_ptr->ref();
		}
		share_ptr(const share_ptr& rp) : _ptr(rp._ptr)
		{
			if (_ptr)
				_ptr->ref();
		}
		template <class Other>
		share_ptr(const share_ptr<Other>& rp) : _ptr(rp._ptr)
		{
			if (_ptr)
				_ptr->ref();
		}
		share_ptr(observer_ptr<T>& optr) : _ptr(0) { optr.lock(*this); }
		~share_ptr()
		{
			if (_ptr)
				_ptr->unref();
			_ptr = 0;
		}

		share_ptr& operator=(const share_ptr& rp)
		{
			assign(rp);
			return *this;
		}

		template <class Other>
		share_ptr& operator=(const share_ptr<Other>& rp)
		{
			assign(rp);
			return *this;
		}

		inline share_ptr& operator=(T* ptr)
		{
			if (_ptr == ptr)
				return *this;
			T* tmp_ptr = _ptr;
			_ptr = ptr;
			if (_ptr)
				_ptr->ref();
			// unref second to prevent any deletion of any object which might
			// be referenced by the other object. i.e rp is child of the
			// original _ptr.
			if (tmp_ptr)
				tmp_ptr->unref();
			return *this;
		}
		// comparison operators for share_ptr.
		bool operator==(const share_ptr& rp) const { return (_ptr == rp._ptr); }
		bool operator==(const T* ptr) const { return (_ptr == ptr); }
		friend bool operator==(const T* ptr, const share_ptr& rp) { return (ptr == rp._ptr); }

		bool operator!=(const share_ptr& rp) const { return (_ptr != rp._ptr); }
		bool operator!=(const T* ptr) const { return (_ptr != ptr); }
		friend bool operator!=(const T* ptr, const share_ptr& rp) { return (ptr != rp._ptr); }

		bool operator<(const share_ptr& rp) const { return (_ptr < rp._ptr); }

	private:
		typedef T* share_ptr::* unspecified_bool_type;

	public:
		// safe bool conversion
		operator unspecified_bool_type() const { return valid() ? &share_ptr::_ptr : 0; }

		T& operator*() const { return *_ptr; }
		T* operator->() const { return _ptr; }

		T* get() const { return _ptr; }

		bool operator!() const { return _ptr == 0; } // not required
		bool valid() const { return _ptr != 0; }

		/** release the pointer from ownership by this share_ptr<>, decrementing the objects
		 * refencedCount() via unref_nodelete() to prevent the Object object from being deleted even
		 * if the reference count goes to zero.  Use when using a local share_ptr<> to an Object
		 * that you want to return from a function/method via a C pointer, whilst preventing the
		 * normal share_ptr<> destructor from cleaning up the object. When using release() you are
		 * implicitly expecting other code to take over management of the object, otherwise a memory
		 * leak will result. */
		T* release()
		{
			T* tmp = _ptr;
			if (_ptr)
				_ptr->unref_nodelete();
			_ptr = 0;
			return tmp;
		}

		void swap(share_ptr& rp)
		{
			T* tmp = _ptr;
			_ptr = rp._ptr;
			rp._ptr = tmp;
		}

	private:
		template <class Other>
		void assign(const share_ptr<Other>& rp)
		{
			if (_ptr == rp._ptr)
				return;
			T* tmp_ptr = _ptr;
			_ptr = rp._ptr;
			if (_ptr)
				_ptr->ref();
			// unref second to prevent any deletion of any object which might
			// be referenced by the other object. i.e rp is child of the
			// original _ptr.
			if (tmp_ptr)
				tmp_ptr->unref();
		}

		template <class Other>
		friend class share_ptr;

		T* _ptr;
	};

	template <class T>
	inline void swap(share_ptr<T>& rp1, share_ptr<T>& rp2)
	{
		rp1.swap(rp2);
	}

	template <class T>
	inline T* get_pointer(const share_ptr<T>& rp)
	{
		return rp.get();
	}

	template <typename T>
	inline T* get_pointer(T* rp)
	{
		return rp;
	}

	template <typename T>
	inline T* get_pointer(const std::shared_ptr<T> rp)
	{
		return rp.get();
	}

	template <typename T>
	typename T::element_type* get_pointer(T rp)
	{
		return rp.get();
	}

	template <class T, class Y>
	inline share_ptr<T> static_pointer_cast(const share_ptr<Y>& rp)
	{
		return static_cast<T*>(rp.get());
	}

	template <class T, class Y>
	inline share_ptr<T> dynamic_pointer_cast(const share_ptr<Y>& rp)
	{
		return dynamic_cast<T*>(rp.get());
	}

	template <class T, class Y>
	inline share_ptr<T> const_pointer_cast(const share_ptr<Y>& rp)
	{
		return const_cast<T*>(rp.get());
	}

	/** * @} */

}
#endif // SHAREPTR_H_