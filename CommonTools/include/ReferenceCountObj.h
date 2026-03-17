//////////////////////////////////////////////////////////////////////
// 文件名称：ReferenceCountObj.h
// 功能描述: 引用对象类型(复用开源代码)
// 创建标识：刘庆仙 2020/10/9
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef REFERENCECOUNTOBJ_H_
#define REFERENCECOUNTOBJ_H_

#include <include/CommonToolsExport.h>

#include <OpenThreads/Atomic>
#include <OpenThreads/Mutex>


namespace d3s {
	/** template class to help enforce static initialization order. */
	template <typename T, T M()>
	struct depends_on
	{
		depends_on() { M(); }
	};

	/** @addtogroup Platform
	 * @{
	 */

	/** Base class for providing reference counted objects.*/
	class COMMONTOOLS_EXPORT ReferenceCountObj
	{

	public:
		ReferenceCountObj();
		virtual ~ReferenceCountObj();

		/** Deprecated, ReferenceCountObj is now always uses thread safe ref/unref, use default
		 * Refernced() constructor instead */
		explicit ReferenceCountObj(bool threadSafeRefUnref);

		ReferenceCountObj(const ReferenceCountObj&);

		inline ReferenceCountObj& operator=(const ReferenceCountObj&) { return *this; }

		/** Deprecated, ReferenceCountObj is always theadsafe so there method now has no effect and
		 * does not need to be called.*/
		virtual void setThreadSafeRefUnref(bool /*threadSafe*/) {}

		/** Get whether a mutex is used to ensure ref() and unref() are thread safe.*/
		bool getThreadSafeRefUnref() const { return true; }

		/** Get the mutex used to ensure thread safety of ref()/unref(). */
		OpenThreads::Mutex* getRefMutex() const { return getGlobalReferencedMutex(); }

		/** Get the optional global ReferenceCountObj mutex, this can be shared between all
		 * osg::ReferenceCountObj.*/
		static OpenThreads::Mutex* getGlobalReferencedMutex();

		/** Increment the reference count by one, indicating that
			this object has another pointer which is referencing it.*/
		inline int ref() const { return ++_refCount; };

		/** Decrement the reference count by one, indicating that
			a pointer to this object is no longer referencing it.  If the
			reference count goes to zero, it is assumed that this object
			is no longer referenced and is automatically deleted.*/
		inline int unref() const
		{
			bool needDelete = false;
			int newRef = --_refCount;
			needDelete = newRef == 0;
			if (needDelete)
			{
				delete this;
			}
			return newRef;
		};

		/** Decrement the reference count by one, indicating that
			a pointer to this object is no longer referencing it.  However, do
			not delete it, even if ref count goes to 0.  Warning, unref_nodelete()
			should only be called if the user knows exactly who will
			be responsible for, one should prefer unref() over unref_nodelete()
			as the latter can lead to memory leaks.*/
		int unref_nodelete() const;

		/** Return the number of pointers currently referencing this object. */
		inline int referenceCount() const { return _refCount; }


	protected:
		mutable OpenThreads::Atomic _refCount;
	};

	// intrusive_ptr_add_ref and intrusive_ptr_release allow
	// use of osg ReferenceCountObj classes with boost::intrusive_ptr
	inline void intrusive_ptr_add_ref(ReferenceCountObj* p) { p->ref(); }
	inline void intrusive_ptr_release(ReferenceCountObj* p) { p->unref(); }

	/** * @} */

}

#endif // REFERENCECOUNTOBJ_H_
