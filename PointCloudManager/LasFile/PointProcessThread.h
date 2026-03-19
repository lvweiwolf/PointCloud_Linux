#ifndef POINT_PROCESS_THREAD_H_
#define POINT_PROCESS_THREAD_H_

#include <LasFile/PointCloudPageLod.h>


#include <pcl/io/pcd_io.h>

#include <osg/Vec3d>
#include <osg/ref_ptr>
#include <OpenThreads/Thread>

#include <deque>
#include <mutex>
#include <condition_variable>


class CPointCloudTool;


// 点到切片处理线程
class CSavePieceProcessThread : public OpenThreads::Thread
{
public:
	CSavePieceProcessThread(CPointCloudTool* fileProcessor);

	~CSavePieceProcessThread();

	virtual void run(void) override;

	void ProcessPointData();

	void SetArrPoint(pcl::PointXYZRGBA* pArrPoint, size_t nPointCount);

	void WaitComplete();

private:
	CPointCloudTool* _fileProcessor;
	pcl::PointXYZRGBA* _pointBuffer;
	size_t _numPoints; // 当前数组已读取点数量

	std::mutex _mtx;
	std::condition_variable _cvWork; // 有任务进来，线程可以开始工作
	std::condition_variable _cvIdle; // 线程空闲

	bool _idle;			 // 线程是否空闲
	bool _workAvailable; // 有工作可做，唤醒线程
	bool _running;		 // 线程是否继续运行
};

class CPointCloudWriteThread : public OpenThreads::Thread
{
public:
	typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudPtr;

	struct WriteTask
	{
		std::string filePath;
		CloudPtr cloudPtr;

		WriteTask() = default;
		WriteTask(std::string&& filePath_, CloudPtr cloud_)
			: filePath(std::move(filePath_)), cloudPtr(std::move(cloud_))
		{
		}
	};

	CPointCloudWriteThread();
	~CPointCloudWriteThread();

	virtual void run() override;

	void WaitComplete();

	void WaitIdle();

	void AddTask(const std::string& filePath, CloudPtr cloudPtr);

	int GetTaskSize() const { return _numProcessTasks; }

	int GetWritedPointSize() const { return _numWritedPoints; }

	bool IsWaiting() const { return _waiting; }

private:
	// 取出任务并处理，返回是否处理了任务
	bool FetchAndProcessTasks();

private:
	std::deque<WriteTask> _tasks;
	std::mutex _mutex;

	std::atomic<bool> _running;
	std::atomic<bool> _waiting;
	std::atomic<bool> _stoped;
	std::atomic<int> _numProcessTasks;
	std::atomic<int> _numWritedPoints;
};



#endif // POINT_PROCESS_THREAD_H_