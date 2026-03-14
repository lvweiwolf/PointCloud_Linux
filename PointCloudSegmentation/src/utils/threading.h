//////////////////////////////////////////////////////////////////////
// 文件名称：threading.h
// 功能描述：并发线程工具
// 创建标识：吕伟	2021/2/5
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////

#pragma once

#include <queue>
#include <future>
#include <unordered_map>
#include <functional>
#include <limits.h>
#include <list>
#include "timer.h"


namespace d3s {
	namespace pcs {

		/**
		 *  @brief    获得有效的线程数
		 *
		 *  @param    const int num_threads	手动指定线程数
		 *  @return   int	可用的线程数
		 */
		int GetEffectiveNumThreads(const int num_threads);

		/************************************************************************/
		/*					线程模型 Thread                                     */
		/************************************************************************/

		/*
			// 帮助类来创建具有简单控制和定时的单线程，例如：。

			class MyThread : public Thread {
			enum {
				PROCESSED_CALLBACK,
			};

			MyThread() { RegisterCallback(PROCESSED_CALLBACK); }
			void Run() {
				// 一些设置程序......注意，这是可选的。
				if (setup_valid) {
				SignalValidSetup();
				} else {
				SignalInvalidSetup();
				}

				// 一些预处理...
				for (const auto& item : items) {
				BlockIfPaused();
				if (IsStopped()) {
					// Tear down...
					break;
				}
				// 处理项目...
				Callback(PROCESSED_CALLBACK);
				}
			}
			};

			MyThread thread;
			thread.AddCallback(MyThread::PROCESSED_CALLBACK, []() {
			std::cout << "Processed item"; })
			thread.AddCallback(MyThread::STARTED_CALLBACK, []() {
			std::cout << "Start"; })
			thread.AddCallback(MyThread::FINISHED_CALLBACK, []() {
			std::cout << "Finished"; })
			thread.Start();
			// thread.CheckValidSetup();
			// Pause, resume, stop, ...
			thread.Wait();
			thread.Timer().PrintElapsedSeconds();
		*/

		class Thread
		{
		public:
			enum { STARTED_CALLBACK = INT_MIN, FINISHED_CALLBACK };

			Thread();

			virtual ~Thread() = default;

			/**
			 *  @brief   线程状态控制 Start: 启动 Stop: 停止 ...
			 *
			 *  @return   void
			 */
			virtual void Start();
			virtual void Stop();
			virtual void Pause();
			virtual void Resume();
			virtual void Wait();

			/**
			 *  @brief    检查线程状态
			 *
			 *  @return   bool
			 */
			bool IsStarted();
			bool IsStopped();
			bool IsPaused();
			bool IsRunning();
			bool IsFinished();

			/**
			 *  @brief    要从主运行函数内部调用。如果线程暂停，这将阻塞调用它的线程，直到线程恢复
			 *
			 *  @return   void
			 */
			void BlockIfPaused();

			/**
			 *  @brief 要从外部调用。这将阻止调用者，直到该线程被设置，即它发出了其设置是否有效的信号。
			 *			  如果它从未给出此信号，此呼叫将无限阻断呼叫者。检查是否设置有效。
			 *
			 *  @return   bool
			 */
			bool CheckValidSetup();

			/**
			 *  @brief    设置运行阶段的回调函数
			 *
			 *  @param    const int id
			 *  @param    const std::function<void> & func
			 *  @return   void
			 */
			void AddCallback(const int id, const std::function<void()>& func);

			/**
			 *  @brief    获取线程的计时信息，适当核算暂停时间。
			 *
			 *  @return   const toolset::Timer&
			 */
			const Timer& GetTimer() const;

		protected:
			/**
			 *  @brief    这是子类要实现的主运行函数。如果你正在循环处理数据，
			 *			  并希望支持暂停操作，请调用"BlockIfPaused"在循环的适当位置。
			 *			  为了支持停止操作，检查 "IsStopped "状态，并从该方法中提前返回。
			 *
			 *  @return   void
			 */
			virtual void Run() = 0;

			/**
			 *  @brief    注册一个新的回调。请注意，只有已注册的回调才能被设置/重置，并在线程中调用。
			 *			  因此，这个方法应该是从派生线程构造函数中调用。
			 *
			 *  @param    const int id
			 *  @return   void
			 */
			void RegisterCallback(const int id);

			/**
			 *  @brief    回调指定名称的函数，如果它存在的话
			 *
			 *  @param    const int id	回调函数注册ID
			 *  @return   void
			 */
			void Callback(const int id) const;

			/**
			 *  @brief    获取当前线程的唯一标识符
			 *
			 *  @return   std::thread::id
			 */
			std::thread::id GetThreadId() const;

			/**
			 *  @brief    发出线程被设置的信号。只调用此函数一次。
			 */
			void SignalValidSetup();
			void SignalInvalidSetup();

		private:
			/**
			 *  @brief    围绕主运行函数的包装，设置完成的标志
			 *
			 *  @return   void
			 */
			void RunFunc();

			std::thread _thread;
			std::mutex _mutex;
			std::condition_variable _pause_condition;
			std::condition_variable _setup_condition;

			Timer _timer;

			bool _started;
			bool _stoped;
			bool _paused;
			bool _pausing;
			bool _finished;
			bool _setup;
			bool _setup_valid;

			std::unordered_map<int, std::list<std::function<void()>>> _callbacks;
		};


		/************************************************************************/
		/*					线程池 ThreadPool                                   */
		/************************************************************************/

		/* 一个线程池类，用于向工作线程池提交通用任务（函数对象）

			ThreadPool thread_pool;
			thread_pool.AddTask([](){ 任务处理... });
			auto future = thread_pool.AddTask([](){ 任务处理...; return 1; });

			for (int i = 0; i <10; ++i) {
				thread_pool.AddTask([](const int i) { 任务处理...});
			}

			thread_pool.Wait();
		*/

		class ThreadPool
		{
		public:
			static const int kMaxNumThreads = -1;

			explicit ThreadPool(const int num_threads = kMaxNumThreads);
			~ThreadPool();

			inline size_t NumThreads() const { return _workers.size(); }

			// 添加一个新任务到线程池(std::future的使用，调用线程可阻塞获得工作线程的返回结果)
			template <typename func_t, class... args_t>
			auto AddTask(func_t&& f, args_t&&... args)
				-> std::future<typename std::result_of<func_t(args_t...)>::type>
			{
				typedef typename std::result_of<func_t(args_t...)>::type return_t;

				auto task = std::make_shared<std::packaged_task<return_t()>>(
					std::bind(std::forward<func_t>(f), std::forward<args_t>(args)...));

				std::future<return_t> result = task->get_future();

				{
					std::unique_lock<std::mutex> lock(_mutex);
					if (_stoped)
					{
						throw std::runtime_error("Cannot add task to stopped thread pool.");
					}

					_tasks.emplace([task]() { (*task)(); });
				}

				_task_condition.notify_one();
				return result;
			}

			// 停止所有的工作线程
			void Stop();

			// 等待所有任务完成
			void Wait();

			// 获取当前调用线程ID
			std::thread::id GetThreadId() const;

			// 获取当前线程的索引。
			// 在大小为N的线程池中，线程索引定义了池中线程的0基索引。
			// 换句话说，有线程索引0，...，N - 1。
			int GetThreadIndex();

		private:
			void WorkerFunc(const int index);

			std::vector<std::thread> _workers;
			std::queue<std::function<void()>> _tasks;

			std::mutex _mutex;
			std::condition_variable _task_condition;	 // 标志是否有任务可执行
			std::condition_variable _finished_condition; // 标志执行的任务是否执行

			bool _stoped;
			int _num_active_workers;

			std::unordered_map<std::thread::id, int> _thread_id_to_index;
		};

		/************************************************************************/
		/*					作业队列 JobQueue                                    */
		/************************************************************************/

		/* 生产者-消费者范式的作业队列类。

			JobQueue<int> job_queue;

			std::thread producer_thread([&job_queue]() {
				for (int i = 0; i < 10; ++i) {
					job_queue.Push(i);
				}
			});


			std::thread consumer_thread([&job_queue]() {
				for (int i=0; i < 10; ++i) {
					const auto job = job_queue.Pop();
					if (job.IsValid())
					{
						执行任务...
					}
					else
					{
						break;
					}
				}
			});


			producer_thread.join();
			consumer_thread.join();
		*/

		template <typename T>
		class JobQueue
		{
		public:
			class Job
			{
			public:
				Job() : _valid(false) {}

				explicit Job(const T& data) : _data(data), _valid(true) {}

				// 检查数据是否有效。
				bool IsValid() const { return _valid; }

				// 获得作业数据引用
				T& Data() { return _data; }
				const T& Data() const { return _data; }

			private:
				T _data;
				bool _valid;
			};

			explicit JobQueue(const size_t max_num_jobs) : _max_num_jobs(max_num_jobs), _stop(false) {}

			JobQueue() : JobQueue(std::numeric_limits<size_t>::max()) {}

			~JobQueue() { Stop(); }

			// 存留在作业队列中的作业数量
			size_t Size()
			{
				std::unique_lock<std::mutex> lock(_mutex);
				return _jobs.size();
			}

			// 将新作业推送到队列中。如果超过作业数量，则阻塞
			bool Push(const T& data)
			{
				std::unique_lock<std::mutex> lock(_mutex);

				while (_jobs.size() >= _max_num_jobs && !_stop)
				{
					_wait_pop_cond.wait(lock); // 队列满了，无法再添加，等待其他线程调用Pop
				}

				if (_stop)
				{
					return false;
				}
				else
				{
					_jobs.push(data);
					_wait_push_cond.notify_one(); // 激活其他因等待Push而导致的阻塞
					return true;
				}
			}

			// 从队列中弹出一个作业。如果队列中没有作业，则阻塞
			Job Pop()
			{
				std::unique_lock<std::mutex> lock(_mutex);

				while (_jobs.empty() && !_stop)
				{
					_wait_push_cond.wait(lock); // 队列空，阻塞，等待其他线程Push
				}

				if (_stop)
				{
					return Job(); // 返回无效作业
				}
				else
				{
					const T data = _jobs.front();
					_jobs.pop();
					_wait_pop_cond.notify_one(); // 激活其他因等待Pop而导致的阻塞

					if (_jobs.empty())
						_wait_empty_cond.notify_all(); // 激活其他因等待队列清空而导致的阻塞

					return Job(data);
				}
			}

			// 等待所有作业弹出，然后停止队列。
			void Wait()
			{
				std::unique_lock<std::mutex> lock(_mutex);
				while (!_jobs.empty())
				{
					_wait_empty_cond.wait(lock); // 队列不为空，阻塞，等待其他线程Pop最终使队列为空
				}
			}

			// 停止作业队列，从该队列的Push/Pop操作都将返回false
			void Stop()
			{
				_stop = true;
				_wait_push_cond.notify_all();
				_wait_pop_cond.notify_all();
			}

			// 清空作业队列
			void Clear()
			{
				std::unique_lock<std::mutex> lock(_mutex);
				std::queue<T> empty_jobs;
				std::swap(_jobs, empty_jobs);
			}

		private:
			size_t _max_num_jobs;
			std::atomic<bool> _stop; // 作业队列是否停止，原子类型

			std::queue<T> _jobs;
			std::mutex _mutex;
			std::condition_variable _wait_push_cond;
			std::condition_variable _wait_pop_cond;
			std::condition_variable _wait_empty_cond;
		};

	} 
}