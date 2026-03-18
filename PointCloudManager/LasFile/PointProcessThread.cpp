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
	// 遍历处理所有点
	auto start = std::chrono::steady_clock::now();
	std::cout << "[Read START]" << std::endl;

	// 计算当前数组启用的多线程数
	size_t nTmpPointCount = _numPoints % MAX_MEMORY_POINT_NUM;
	nTmpPointCount = (0 == nTmpPointCount) ? MAX_MEMORY_POINT_NUM : nTmpPointCount;
	const int nTbbCount = std::ceil(nTmpPointCount / (float)nSpliteSize);

	tbb::parallel_for(tbb::blocked_range<size_t>(0, nTbbCount),
					  [&](const tbb::blocked_range<size_t>& r) {
						  for (size_t i = r.begin(); i != r.end(); ++i)
						  {
							  size_t nStartPointIndex = nSpliteSize * i;
							  size_t nSpliteCount = nSpliteSize;

							  if (nStartPointIndex + nSpliteCount > nTmpPointCount)
							  {
								  nSpliteCount = nTmpPointCount - nStartPointIndex;
							  }

							  for (size_t ii = 0; ii < nSpliteCount; ++ii)
							  {
								  size_t bufferIdx = nStartPointIndex + ii;
								  _fileProcessor->SavePoinToPieceInfo(_pointBuffer[bufferIdx],
																	  bufferIdx);
							  }
						  }
					  });

	// 清空数组
	memset(_pointBuffer, 0, sizeof(pcl::PointXYZRGBA) * MAX_MEMORY_POINT_NUM);

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

CPointCloudWriteThread::CPointCloudWriteThread() : _running(true) {}

CPointCloudWriteThread::~CPointCloudWriteThread() { WaitComplete(); }

void CPointCloudWriteThread::run()
{
	while (_running)
	{
		std::vector<std::shared_ptr<WriteTask>> batchTasks;

		{
			std::unique_lock<std::mutex> lock(_mutex);
			_cv.wait_for(lock, batchInterval, [this] { return !_tasks.empty() || !_running; });

			if (!_running)
				break;

			size_t batchCount = _tasks.size();
			for (size_t i = 0; i < batchCount; ++i)
			{
				batchTasks.push_back(std::move(_tasks.front()));
				_tasks.pop();
			}
		}

		// 执行任务
		if (!batchTasks.empty())
		{
			auto start = std::chrono::steady_clock::now();
			std::cout << "[Write START] " << " Batch size: " << batchTasks.size() << std::endl;

			// tbb::parallel_for(tbb::blocked_range<size_t>(0, batchTasks.size()),
			// 				  [&](const tbb::blocked_range<size_t>& r) {
			// 					  for (size_t i = r.begin(); i != r.end(); ++i)
			// 					  {
			// 						  auto& task = batchTasks.at(i);
			// 						  pcl::io::savePCDFileBinary(task->filePath, *(task->cloudPtr));
			// 					  }
			// 				  });

			for (const auto& task : batchTasks)
			{
				pcl::io::savePCDFileBinary(task->filePath, *(task->cloudPtr));
			}

			auto end = std::chrono::steady_clock::now();
			auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
			std::cout << "[Write DONE] " << elapsed_ms.count() << " ms" << std::endl;
		}
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
		_tasks.push(std::make_shared<WriteTask>(filePath, cloudPtr));
	}

	_cv.notify_one();
}

size_t CPointCloudWriteThread::GetTaskSize()
{
	std::lock_guard<std::mutex> lock(_mutex);
	return _tasks.size();
}