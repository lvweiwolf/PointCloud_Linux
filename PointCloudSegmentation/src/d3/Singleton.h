#ifndef D3S_SINGLETON_H_
#define D3S_SINGLETON_H_

// 单例定义
template <typename subclass>
class Singleton
{
public:
	Singleton() {}

	virtual ~Singleton() {}

	static subclass* GetInst() { return _pInstance; }

	static void FreeInst()
	{
		if (_pInstance != nullptr)
		{
			delete _pInstance;
			_pInstance = nullptr;
		}
	}

private:
	static subclass* _pInstance;
};

#endif // D3S_SINGLETON_H_