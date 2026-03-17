
#include <Tool/CriticalSection.h>

#include <cstring>


namespace toolkit {
	CCriticalSectionHandle::CCriticalSectionHandle(void)
	{
#ifdef _DEBUG
		// 调试模式下可以清零结构（非必须，但保持风格）
		memset(&_mutex, 0, sizeof(pthread_mutex_t));
#endif
		// 初始化互斥量，使用默认属性（快速互斥）
		int ret = pthread_mutex_init(&_mutex, NULL);
		assert(ret == 0); // 确保初始化成功
	}

	CCriticalSectionHandle::~CCriticalSectionHandle(void)
	{
		int ret = pthread_mutex_destroy(&_mutex);
		assert(ret == 0); // 确保销毁成功
	}

	void CCriticalSectionHandle::Enter(void)
	{
		try
		{
			int ret = pthread_mutex_lock(&_mutex);
			assert(ret == 0); // 加锁失败通常意味着严重错误
		}
		catch (...)
		{
			// 保持原有异常处理风格，但 Linux 下 pthread 函数不会抛出异常
			// 此处仅保留结构，实际可替换为日志或断言
			assert(false);
		}
	}

	void CCriticalSectionHandle::Leave(void)
	{
		int ret = pthread_mutex_unlock(&_mutex);
		assert(ret == 0);
	}

	CCriticalSectionSync::CCriticalSectionSync(CCriticalSectionHandle& csc)
	{
		_csc = &csc;
		_csc->Enter();
	}

	CCriticalSectionSync::~CCriticalSectionSync(void) { _csc->Leave(); }
}