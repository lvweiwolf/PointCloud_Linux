#include <src/utils/threading.h>
#include <src/utils/logging.h>

namespace d3s {
	namespace pcs {

		int GetEffectiveNumThreads(const int num_threads)
		{
			int num_effective_threads = num_threads;
			if (num_threads <= 0)
			{
				// 获得最多核心数
				num_effective_threads = std::thread::hardware_concurrency();
			}

			if (num_effective_threads <= 0)
			{
				num_effective_threads = 1;
			}

			return num_effective_threads;
		}

		// Thread
		//////////////////////////////////////////////////////////////////////////

		Thread::Thread()
			: _started(false),
			  _stoped(false),
			  _paused(false),
			  _pausing(false),
			  _finished(false),
			  _setup(false),
			  _setup_valid(false)
		{
			RegisterCallback(STARTED_CALLBACK);
			RegisterCallback(FINISHED_CALLBACK);
		}

		void Thread::Start()
		{
			std::unique_lock<std::mutex> lock(_mutex);
			CHECK(!_started || _finished);

			// 启动线程后等待结束
			Wait();

			_timer.Restart();
			_thread = std::thread(&Thread::RunFunc, this);
			_started = true;
			_stoped = false;
			_paused = false;
			_pausing = false;
			_finished = false;
			_setup = false;
			_setup_valid = false;
		}

		void Thread::Stop()
		{
			{
				std::unique_lock<std::mutex> lock(_mutex);
				_stoped = true;
			}

			Resume();
		}

		void Thread::Pause()
		{
			std::unique_lock<std::mutex> lock(_mutex);
			_paused = true;
		}

		void Thread::Resume()
		{
			std::unique_lock<std::mutex> lock(_mutex);

			if (_paused)
			{
				_paused = false;
				_pause_condition.notify_all();
			}
		}

		void Thread::Wait()
		{
			if (_thread.joinable())
				_thread.join();
		}

		bool Thread::IsStarted()
		{
			std::unique_lock<std::mutex> lock(_mutex);
			return _started;
		}

		bool Thread::IsStopped()
		{
			std::unique_lock<std::mutex> lock(_mutex);
			return _stoped;
		}

		bool Thread::IsPaused()
		{
			std::unique_lock<std::mutex> lock(_mutex);
			return _paused;
		}

		bool Thread::IsRunning()
		{
			std::unique_lock<std::mutex> lock(_mutex);
			return _started && !_pausing && !_finished;
		}

		bool Thread::IsFinished()
		{
			std::unique_lock<std::mutex> lock(_mutex);
			return _finished;
		}

		void Thread::BlockIfPaused()
		{
			std::unique_lock<std::mutex> lock(_mutex);

			if (_paused)
			{
				_pausing = true;
				_timer.Pause();
				_pause_condition.wait(lock); // 确认调用线程被暂停时，阻塞线程执行，并且停止计时
				_pausing = false;
				_timer.Resume();
			}
		}

		bool Thread::CheckValidSetup()
		{
			std::unique_lock<std::mutex> lock(_mutex);

			if (!_setup)
			{
				_setup_condition.wait(lock); // 只有在线程被setup的情况下才执行, 否则阻塞
			}

			return _setup_valid;
		}

		void Thread::AddCallback(const int id, const std::function<void()>& func)
		{
			CHECK(func);
			CHECK_MSG(_callbacks.count(id) > 0, "Callback not registered");
			_callbacks.at(id).push_back(func);
		}

		const Timer& Thread::GetTimer() const { return _timer; }

		void Thread::RegisterCallback(const int id)
		{
			_callbacks.emplace(id, std::list<std::function<void()>>());
		}

		void Thread::Callback(const int id) const
		{
			CHECK_MSG(_callbacks.count(id) > 0, "Callback not registered");

			for (const auto& callback : _callbacks.at(id))
			{
				callback();
			}
		}

		std::thread::id Thread::GetThreadId() const { return std::this_thread::get_id(); }

		void Thread::SignalValidSetup()
		{
			std::unique_lock<std::mutex> lock(_mutex);
			CHECK(!_setup);

			_setup = true;
			_setup_valid = true;
			_setup_condition.notify_all();
		}

		void Thread::SignalInvalidSetup()
		{
			std::unique_lock<std::mutex> lock(_mutex);
			CHECK(!_setup);

			_setup = true;
			_setup_valid = false;
			_setup_condition.notify_all();
		}

		void Thread::RunFunc()
		{
			Callback(STARTED_CALLBACK);
			Run();
			{
				std::unique_lock<std::mutex> lock(_mutex);
				_finished = true;
				_timer.Pause();
			}
			Callback(FINISHED_CALLBACK);
		}


		// ThreadPool
		//////////////////////////////////////////////////////////////////////////

		ThreadPool::ThreadPool(const int num_threads /*= kMaxNumThreads*/)
			: _stoped(false), _num_active_workers(0)
		{
			const int num_effective_threads = GetEffectiveNumThreads(num_threads);

			for (int index = 0; index < num_effective_threads; ++index)
			{
				// 绑定任务的处理函数到线程池对象的WorkerFunc，统一管理派发
				std::function<void()> worker = std::bind(&ThreadPool::WorkerFunc, this, index);

				_workers.emplace_back(worker);
			}
		}

		ThreadPool::~ThreadPool() { Stop(); }

		void ThreadPool::Stop()
		{
			{
				std::unique_lock<std::mutex> lock(_mutex);

				if (_stoped)
				{
					return;
				}

				_stoped = true;
				std::queue<std::function<void()>> empty_tasks;
				std::swap(_tasks, empty_tasks);
			}

			_task_condition.notify_all();

			for (auto& worker : _workers)
			{
				worker.join();
			}

			_finished_condition.notify_all();
		}

		void ThreadPool::Wait()
		{
			std::unique_lock<std::mutex> lock(_mutex);

			if (!_tasks.empty() || _num_active_workers > 0)
			{
				_finished_condition.wait(lock, [this]() {
					return _tasks.empty() && _num_active_workers == 0;
				});
			}
		}

		std::thread::id ThreadPool::GetThreadId() const { return std::this_thread::get_id(); }

		int ThreadPool::GetThreadIndex()
		{
			std::unique_lock<std::mutex> lock(_mutex);
			return _thread_id_to_index.at(GetThreadId());
		}

		void ThreadPool::WorkerFunc(const int index)
		{
			{
				std::lock_guard<std::mutex> lock(_mutex);
				_thread_id_to_index.emplace(GetThreadId(), index);
			}

			// 循环处理送进来的任务task
			while (true)
			{
				std::function<void()> task;
				{
					std::unique_lock<std::mutex> lock(_mutex);
					// 当没有可处理的任务时阻塞，避免CPU的浪费
					_task_condition.wait(lock, [this]() { return _stoped || !_tasks.empty(); });

					if (_stoped && _tasks.empty())
						return;

					task = std::move(_tasks.front());
					_tasks.pop();
					_num_active_workers += 1;
				}

				task();

				{
					std::unique_lock<std::mutex> lock(_mutex);
					_num_active_workers -= 1;
				}

				_finished_condition.notify_all();
			}
		}
	}
}