//////////////////////////////////////////////////////////////////////
// 文件名称：TBBParallel.h
// 功能描述：TBB并行线程
// 创建标识：王海民	2019/03/14
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////

#if 0

#ifndef TBBPARALLEL_H_
#define TBBPARALLEL_H_

#include <include/CommonToolsExport.h>

#include <memory>


class COMMONTOOLS_EXPORT CTBBParallel
{
private:
	class IteratorInterface
	{
	public:
		virtual ~IteratorInterface() {}

		//! Returns true if iterators wrapped by this and theOther are equal
		virtual bool IsEqual(const IteratorInterface& theOther) const = 0;

		//! Increments wrapped iterator
		virtual void Increment() = 0;

		//! Returns new instance of the wrapper containing copy
		//! of the wrapped iterator.
		virtual IteratorInterface* Clone() const = 0;
	};

	//! Implementation of polymorphic iterator wrapper suitable for basic
	//! types as well as for std iterators.
	//! Wraps instance of actual iterator type Type.
	template <class Type>
	class IteratorWrapper : public IteratorInterface
	{
	public:
		IteratorWrapper() {}
		IteratorWrapper(const Type& theValue) : myValue(theValue) {}

		virtual bool IsEqual(const IteratorInterface& theOther) const override
		{
			return myValue == dynamic_cast<const IteratorWrapper<Type>&>(theOther).myValue;
		}

		virtual void Increment() override { ++myValue; }

		virtual IteratorInterface* Clone() const override
		{
			return new IteratorWrapper<Type>(myValue);
		}

		const Type& Value() const { return myValue; }

	private:
		Type myValue;
	};

protected:
	// Note: UniversalIterator and FunctorInterface are made protected to be
	// accessible from specialization using threads (non-TBB).

	//! Fixed-type iterator, implementing STL forward iterator interface, used for
	//! iteration over objects subject to parallel processing.
	//! It stores pointer to instance of polymorphic iterator inheriting from
	//! IteratorInterface, which contains actual type-specific iterator.
	class UniversalIterator :
		// Note that TBB requires that value_type of iterator be copyable,
		// thus we use its own type for that
		public std::iterator<std::forward_iterator_tag,
							 UniversalIterator,
							 ptrdiff_t,
							 UniversalIterator*,
							 UniversalIterator&>
	{
	public:
		UniversalIterator() {}

		UniversalIterator(IteratorInterface* theOther) : myPtr(theOther) {}

		UniversalIterator(const UniversalIterator& theOther) : myPtr(theOther.myPtr->Clone()) {}

		UniversalIterator& operator=(const UniversalIterator& theOther)
		{
			myPtr.reset(theOther.myPtr->Clone());
			return *this;
		}

		bool operator!=(const UniversalIterator& theOther) const
		{
			return !myPtr->IsEqual(*theOther.myPtr);
		}

		bool operator==(const UniversalIterator& theOther) const
		{
			return myPtr->IsEqual(*theOther.myPtr);
		}

		UniversalIterator& operator++()
		{
			myPtr->Increment();
			return *this;
		}

		UniversalIterator operator++(int)
		{
			UniversalIterator aValue(*this);
			myPtr->Increment();
			return aValue;
		}

		const UniversalIterator& operator*() const { return *this; }
		UniversalIterator& operator*() { return *this; }

		const UniversalIterator* operator->() const { return this; }
		UniversalIterator* operator->() { return this; }

		// type cast to actual iterator
		template <typename Iterator>
		const Iterator& DownCast() const
		{
			return dynamic_cast<CTBBParallel::IteratorWrapper<Iterator>*>(myPtr.get())->Value();
		}

	private:
#if (defined(_MSC_VER) && (_MSC_VER < 1600))
		std::auto_ptr<IteratorInterface> myPtr;
#else
		std::unique_ptr<IteratorInterface> myPtr;
#endif
	};

	//! Interface class representing functor object.
	//! Intended to add polymorphic behavour to For and ForEach functionality
	//! enabling execution of arbitrary function in parallel mode.
	class FunctorInterface
	{
	public:
		virtual ~FunctorInterface() {}

		virtual void operator()(UniversalIterator& theIterator) const = 0;
	};

private:
	//! Wrapper for functors manipulating on std iterators.
	template <class Iterator, class Functor>
	class FunctorWrapperIter : public FunctorInterface
	{
	public:
		FunctorWrapperIter(const Functor& theFunctor) : myFunctor(theFunctor) {}

		virtual void operator()(UniversalIterator& theIterator) const override
		{
			const Iterator& anIt = theIterator.DownCast<Iterator>();
			myFunctor(*anIt);
		}

	private:
		FunctorWrapperIter(const FunctorWrapperIter&);
		void operator=(const FunctorWrapperIter&);
		const Functor& myFunctor;
	};

	//! Wrapper for functors manipulating on integer index.
	template <class Functor>
	class FunctorWrapperInt : public FunctorInterface
	{
	public:
		FunctorWrapperInt(const Functor& theFunctor) : myFunctor(theFunctor) {}

		virtual void operator()(UniversalIterator& theIterator) const override
		{
			int anIndex = theIterator.DownCast<int>();
			myFunctor(anIndex);
		}

	private:
		FunctorWrapperInt(const FunctorWrapperInt&);
		void operator=(const FunctorWrapperInt&);
		const Functor& myFunctor;
	};

private:
	//! Simple primitive for parallelization of "foreach" loops, e.g.:
	//! @code
	//!   for (std::iterator anIter = theBegin; anIter != theEnd; ++anIter) {}
	//! @endcode
	//! Implementation of framework-dependent functionality should be provided by
	//! forEach_impl function defined in opencascade::parallel namespace.
	//! @param theBegin   the first index (incusive)
	//! @param theEnd     the last  index (exclusive)
	//! @param theFunctor functor providing an interface "void operator(InputIterator theIter){}"
	//!                   performing task for the specified iterator position
	static void forEach(UniversalIterator& theBegin,
						UniversalIterator& theEnd,
						const FunctorInterface& theFunctor);

public: //! @name public methods
		//! Returns number of logical proccesrs.
	static int NbLogicalProcessors();

	//! Simple primitive for parallelization of "foreach" loops, equivalent to:
	//! @code
	//!   for (auto anIter = theBegin; anIter != theEnd; ++anIter) {
	//!     theFunctor(*anIter);
	//!   }
	//! @endcode
	//! @param theBegin   the first index (incusive)
	//! @param theEnd     the last  index (exclusive)
	//! @param theFunctor functor providing an interface "void operator(InputIterator theIter){}"
	//!                   performing task for specified iterator position
	//! @param isForceSingleThreadExecution if true, then no threads will be created
	template <typename InputIterator, typename Functor>
	static void ForEach(InputIterator theBegin,
						InputIterator theEnd,
						const Functor& theFunctor,
						const bool isForceSingleThreadExecution = false)
	{
		if (isForceSingleThreadExecution)
		{
			for (InputIterator it(theBegin); it != theEnd; ++it)
				theFunctor(*it);
		}
		else
		{
			UniversalIterator aBegin(new IteratorWrapper<InputIterator>(theBegin));
			UniversalIterator aEnd(new IteratorWrapper<InputIterator>(theEnd));
			FunctorWrapperIter<InputIterator, Functor> aFunctor(theFunctor);
			forEach(aBegin, aEnd, aFunctor);
		}
	}

	//! Simple primitive for parallelization of "for" loops, equivalent to:
	//! @code
	//!   for (int anIter = theBegin; anIter != theEnd; ++anIter) {
	//!     theFunctor(anIter);
	//!   }
	//! @endcode
	//! @param theBegin   the first index (incusive)
	//! @param theEnd     the last  index (exclusive)
	//! @param theFunctor functor providing an interface "void operator(int theIndex){}"
	//!                   performing task for specified index
	//! @param isForceSingleThreadExecution if true, then no threads will be created
	template <typename Functor>
	static void For(const int theBegin,
					const int theEnd,
					const Functor& theFunctor,
					const bool isForceSingleThreadExecution = false)
	{
		if (isForceSingleThreadExecution)
		{
			for (int it(theBegin); it != theEnd; ++it)
				theFunctor(it);
		}
		else
		{
			UniversalIterator aBegin(new IteratorWrapper<int>(theBegin));
			UniversalIterator aEnd(new IteratorWrapper<int>(theEnd));
			FunctorWrapperInt<Functor> aFunctor(theFunctor);
			forEach(aBegin, aEnd, aFunctor);
		}
	}
};


#endif // TBBPARALLEL_H_

#endif