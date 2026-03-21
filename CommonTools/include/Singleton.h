#ifndef SINGLETON_H_
#define SINGLETON_H_

template <typename T>
class Singleton
{
public:
	Singleton(const Singleton&) = delete;
	Singleton& operator=(const Singleton&) = delete;

	static T* GetInst()
	{
		static T instance;
		return &instance;
	}

protected:
	Singleton() = default;
	virtual ~Singleton() = default;
};

#endif // SINGLETON_H_