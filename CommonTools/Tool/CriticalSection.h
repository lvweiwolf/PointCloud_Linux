//*****************************************************
//
//    @copyright      配网设计平台组
//    @version        v4.0
//    @file           CriticalSection.H
//    @author         fanHong
//    @date           2017/11/3 11:17
//    @brief          互斥控件器
//
//*****************************************************

#ifndef CRITICALSECTION_H_
#define CRITICALSECTION_H_

#include <include/CommonToolsExport.h>

#include <pthread.h> // Linux 线程互斥量
#include <assert.h>	 // 断言

namespace toolkit {
	//! 定义互斥控件器
	class COMMONTOOLS_EXPORT CCriticalSectionHandle
	{
	public:
		CCriticalSectionHandle(void);
		~CCriticalSectionHandle(void);
		void Enter(void);
		void Leave(void);

	protected:
		pthread_mutex_t _mutex; // Linux 互斥量
	};

	//! CCriticalSectionSync
	class COMMONTOOLS_EXPORT CCriticalSectionSync
	{
	public:
		CCriticalSectionSync(CCriticalSectionHandle& csc);
		~CCriticalSectionSync(void);

	protected:
		CCriticalSectionHandle* _csc;
	};
}

#endif // CRITICALSECTION_H_