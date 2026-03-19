#include <LasFile/PointProcessThread.h>
#include <LasFile/PointCloudTool.h>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#ifndef MAX_MEMORY_POINT_NUM
#define MAX_MEMORY_POINT_NUM 40000000
#endif

static const int nSpliteSize = 2000000; // 单位线程处理点数量

/////////////////////////////////////////////////////////////////////
CSavePieceProcessThread::CSavePieceProcessThread(CPointCloudTool* fileProcessor)
	: _fileProcessor(fileProcessor),
	  _pointBuffer(nullptr),
	  _numPoints(0),
	  _idle(false),
	  _workAvailable(false),
	  _running(true) {};

CSavePieceProcessThread::~CSavePieceProcessThread() { WaitComplete(); };

void CSavePieceProcessThread::run()
{
	while (true)
	{
		std::unique_lock<std::mutex> lock(_mtx);
		_idle = true;
		// 标记线程空闲，通知主线程
		_cvIdle.notify_one();

		// 等待工作信号
		_cvWork.wait(lock, [this] { return _workAvailable || !_running; });

		if (!_running)
			break;

		// 重置工作标识
		_workAvailable = false;

		// 处理数据
		ProcessPointData();
	}
};

void CSavePieceProcessThread::ProcessPointData()
{
	typedef std::pair<pcl::PointXYZRGBA, size_t> PointCacheItem; // 点云点和对应的索引
	typedef std::unordered_map<size_t, std::vector<PointCacheItem>> PointPieceCache;

	// auto start = std::chrono::steady_clock::now();
	// std::cout << "[Read START]" << std::endl;

	// 计算当前数组启用的多线程数
	size_t nTmpPointCount = _numPoints % MAX_MEMORY_POINT_NUM;
	nTmpPointCount = (0 == nTmpPointCount) ? MAX_MEMORY_POINT_NUM : nTmpPointCount;
	const int nTbbCount =
		static_cast<int>(std::ceil(nTmpPointCount / static_cast<float>(nSpliteSize)));

	tbb::parallel_for(tbb::blocked_range<size_t>(0, nTbbCount),
					  [&](const tbb::blocked_range<size_t>& r) {
						  // 本地缓冲：按 PieceInfo 索引分组收集点，减少锁竞争
						  PointPieceCache localBuffer;

						  for (size_t i = r.begin(); i != r.end(); ++i)
						  {
							  size_t nStartPointIndex = nSpliteSize * i;
							  size_t nSpliteCount = nSpliteSize;

							  if (nStartPointIndex + nSpliteCount > nTmpPointCount)
							  {
								  nSpliteCount = nTmpPointCount - nStartPointIndex;
							  }

							  for (size_t nIndex = 0; nIndex < nSpliteCount; ++nIndex)
							  {
								  size_t buferIndex = nStartPointIndex + nIndex;
								  _fileProcessor->SavePoinToPieceInfo(_pointBuffer[buferIndex],
																	  buferIndex);
							  }
						  }
					  });
	// 移除不必要的 memset，只需重置计数即可
	_numPoints = 0;

	// auto end = std::chrono::steady_clock::now();
	// auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	// std::cout << "[Read DONE]  " << elapsed_ms.count() << " ms" << std::endl;
}

void CSavePieceProcessThread::SetArrPoint(pcl::PointXYZRGBA* pArrPoint, size_t nPointCount)
{
	std::unique_lock<std::mutex> lock(_mtx);
	// 等待线程进入空闲状态(等待上一轮处理完成)
	_cvIdle.wait(lock, [this] { return _idle; });

	// 更新任务数据
	_pointBuffer = pArrPoint;
	_numPoints = nPointCount;

	// 标记有工作可做，唤醒工作线程
	_workAvailable = true;
	_idle = false;
	_cvWork.notify_one();
}

void CSavePieceProcessThread::WaitComplete()
{
	{
		std::unique_lock<std::mutex> lock(_mtx);
		// 等待线程进入空闲状态
		_cvIdle.wait(lock, [this] { return _idle; });

		// 设置线程状态为停止，并唤醒工作线程
		_running = false;
		_workAvailable = true; // 让线程退出等待
		_cvWork.notify_one();
	}

	// 等待线程结束
	join();
};

////////////////////////////////////////////////////////////////////

#define IDEL_ELAPSE std::chrono::milliseconds(100)

CPointCloudWriteThread::CPointCloudWriteThread()
	: _running(true), _waiting(false), _stoped(false), _numProcessTasks(0), _numWritedPoints(0)
{
}

CPointCloudWriteThread::~CPointCloudWriteThread() { WaitComplete(); }

void CPointCloudWriteThread::run()
{
	while (_running)
	{
		if (_numProcessTasks == 0)
		{
			std::this_thread::sleep_for(IDEL_ELAPSE); // 避免空转
			_waiting = true;
			continue;
		}

		_waiting = false;

		std::vector<WriteTask> processTasks;

		// 线程临界区
		{
			std::unique_lock<std::mutex> lock(_mutex);

			while (!_tasks.empty())
			{
				WriteTask task = _tasks.front();
				_numWritedPoints += task.cloudPtr->size();
				_numProcessTasks--;
				_tasks.pop_front();
				processTasks.push_back(task);
			}
		}

		// 处理任务
		const size_t taskSize = processTasks.size();
		// auto start = std::chrono::steady_clock::now();
		// std::cout << "[Write START] Batch size: " << taskSize << std::endl;

		tbb::parallel_for(tbb::blocked_range<size_t>(0, taskSize),
						  [&](const tbb::blocked_range<size_t>& r) {
							  for (size_t i = r.begin(); i != r.end(); ++i)
							  {
								  const auto& task = processTasks[i];
								  pcl::io::savePCDFileBinary(task.filePath, *(task.cloudPtr));
							  }
						  });

		// auto end = std::chrono::steady_clock::now();
		// auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		// std::cout << "[Write DONE] " << elapsed_ms.count() << " ms" << std::endl;
	}

	_stoped = true;
}

void CPointCloudWriteThread::WaitComplete()
{
	// 任务未处理完
	while (_numProcessTasks > 0)
	{
		std::this_thread::sleep_for(IDEL_ELAPSE);
	}

	_running = false;

	// 等待任务完全处理完
	while (!_stoped)
	{
		std::this_thread::sleep_for(IDEL_ELAPSE);
	}

	join();
}

void CPointCloudWriteThread::WaitIdle()
{
	while (!IsWaiting())
		std::this_thread::sleep_for(IDEL_ELAPSE);
}

void CPointCloudWriteThread::AddTask(const std::string& filePath, CloudPtr cloudPtr)
{
	std::unique_lock<std::mutex> lock(_mutex);
	_tasks.emplace_back(std::string(filePath), cloudPtr);
	_numProcessTasks++;
}
