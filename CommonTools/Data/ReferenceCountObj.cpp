#include <include/ReferenceCountObj.h>

#include <stdlib.h>

namespace d3s {

	// #define ENFORCE_THREADSAFE
	// #define DEBUG_OBJECT_ALLOCATION_DESTRUCTION

	// specialized smart pointer, used to get round auto_ptr<>'s lack of the destructor resetting
	// itself to 0.
	template <typename T>
	struct ResetPointer
	{
		ResetPointer() : _ptr(0) {}

		ResetPointer(T* ptr) : _ptr(ptr) {}

		~ResetPointer()
		{
			delete _ptr;
			_ptr = 0;
		}

		inline ResetPointer& operator=(T* ptr)
		{
			if (_ptr == ptr)
				return *this;
			delete _ptr;
			_ptr = ptr;
			return *this;
		}

		void reset(T* ptr)
		{
			if (_ptr == ptr)
				return;
			delete _ptr;
			_ptr = ptr;
		}

		inline T& operator*() { return *_ptr; }

		inline const T& operator*() const { return *_ptr; }

		inline T* operator->() { return _ptr; }

		inline const T* operator->() const { return _ptr; }

		T* get() { return _ptr; }

		const T* get() const { return _ptr; }

		T* _ptr;
	};

	typedef ResetPointer<OpenThreads::Mutex> GlobalMutexPointer;

	OpenThreads::Mutex* ReferenceCountObj::getGlobalReferencedMutex()
	{
		static GlobalMutexPointer s_ReferencedGlobalMutext = new OpenThreads::Mutex;
		return s_ReferencedGlobalMutext.get();
	}

	// helper class for forcing the global mutex to be constructed when the library is loaded.
	struct InitGlobalMutexes
	{
		InitGlobalMutexes() { ReferenceCountObj::getGlobalReferencedMutex(); }
	};

	static InitGlobalMutexes s_initGlobalMutexes;

	// static std::auto_ptr<DeleteHandler> s_deleteHandler(0);

	ReferenceCountObj::ReferenceCountObj() : _refCount(0) {}

	ReferenceCountObj::ReferenceCountObj(bool /*threadSafeRefUnref*/) : _refCount(0) {}

	ReferenceCountObj::ReferenceCountObj(const ReferenceCountObj&) : _refCount(0) {}

	ReferenceCountObj::~ReferenceCountObj()
	{
		if (_refCount > 0)
		{
			// OSG_WARN << "Warning: deleting still referenced object " << this << " of type '" <<
			// typeid(this).name() << "'" << std::endl; OSG_WARN << "         the final reference
			// count was " << _refCount << ", memory corruption possible." << std::endl;
		}
	}

	int ReferenceCountObj::unref_nodelete() const { return --_refCount; }

}
