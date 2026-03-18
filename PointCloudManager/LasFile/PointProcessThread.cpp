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

	auto start = std::chrono::steady_clock::now();
	std::cout << "[Read START]" << std::endl;

	// 计算当前数组启用的多线程数
	size_t nTmpPointCount = _numPoints % MAX_MEMORY_POINT_NUM;
	nTmpPointCount = (0 == nTmpPointCount) ? MAX_MEMORY_POINT_NUM : nTmpPointCount;
	const int nTbbCount = static_cast<int>(std::ceil(nTmpPointCount / static_cast<float>(nSpliteSize)));

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

							  // 第一遍：收集点到本地缓冲
							  for (size_t ii = 0; ii < nSpliteCount; ++ii)
							  {
								  size_t bufferIdx = nStartPointIndex + ii;
								  _fileProcessor->CollectPointToLocalBuffer(_pointBuffer[bufferIdx],
																			 bufferIdx, localBuffer);
							  }
							  
							  // 第二遍：批量写入，大幅减少锁竞争
							  _fileProcessor->FlushLocalBuffer(localBuffer);
						  }
					  });

	// 移除不必要的 memset，只需重置计数即可
	_numPoints = 0;

	auto end = std::chrono::steady_clock::now();
	auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "[Read DONE]  " << elapsed_ms.count() << " ms" << std::endl;
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

CPointCloudWriteThread::CPointCloudWriteThread() : _running(true) 
{
	_taskBuffer.reserve(64);
}

CPointCloudWriteThread::~CPointCloudWriteThread() { WaitComplete(); }

bool CPointCloudWriteThread::FetchAndProcessTasks()
{
	_taskBuffer.clear();
	{
		std::lock_guard<std::mutex> lock(_mutex);
		while (!_tasks.empty())
		{
			_taskBuffer.push_back(std::move(_tasks.front()));
			_tasks.pop();
		}
	}

	if (_taskBuffer.empty())
		return false;

	const size_t taskCount = _taskBuffer.size();
	auto start = std::chrono::steady_clock::now();
	std::cout << "[Write START] Batch size: " << taskCount << std::endl;

	tbb::parallel_for(tbb::blocked_range<size_t>(0, taskCount),
					  [this](const tbb::blocked_range<size_t>& r) {
						  for (size_t i = r.begin(); i != r.end(); ++i)
						  {
							  const auto& task = _taskBuffer[i];
							  pcl::io::savePCDFileBinary(task.filePath, *(task.cloudPtr));
						  }
					  });

	auto end = std::chrono::steady_clock::now();
	auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "[Write DONE] " << elapsed_ms.count() << " ms" << std::endl;
	return true;
}

void CPointCloudWriteThread::run()
{
	while (_running)
	{
		{
			std::unique_lock<std::mutex> lock(_mutex);
			// 等待条件：任务达到阈值，或超时，或停止运行
			_cv.wait_for(lock, std::chrono::milliseconds(WAIT_TIMEOUT_MS),
				[this] { return _tasks.size() >= PARALLEL_THRESHOLD || !_running; });
		}

		// 处理任务（无论是因为阈值触发还是超时触发）
		FetchAndProcessTasks();
	}

	// 线程退出前处理剩余任务
	while (FetchAndProcessTasks())
	{
		// 循环处理直到队列为空
	}
}

void CPointCloudWriteThread::WaitComplete()
{
	_running = false;
	_cv.notify_all();
	join();
}

void CPointCloudWriteThread::AddTask(const std::string& filePath, CloudPtr cloudPtr)
{
	{
		std::lock_guard<std::mutex> lock(_mutex);
		_tasks.emplace(std::string(filePath), std::move(cloudPtr));
	}
	// 任务数量达到阈值时唤醒处理线程
	if (_tasks.size() >= PARALLEL_THRESHOLD)
		_cv.notify_one();
}

size_t CPointCloudWriteThread::GetTaskSize()
{
	std::lock_guard<std::mutex> lock(_mutex);
	return _tasks.size();
}