#ifndef POINT_PROCESS_THREAD_H_
#define POINT_PROCESS_THREAD_H_

#include <LasFile/PointCloudPageLod.h>


#include <pcl/io/pcd_io.h>

#include <osg/Vec3d>
#include <osg/ref_ptr>
#include <OpenThreads/Thread>

#include <queue>
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
		WriteTask(const std::string& filePath_, CloudPtr cloud_)
			: filePath(filePath_), cloudPtr(cloud_)
		{
		}

		std::string filePath;
		CloudPtr cloudPtr;
	};

	CPointCloudWriteThread();
	~CPointCloudWriteThread();

	virtual void run() override;

	void WaitComplete();

	void AddTask(const std::string& filePath, CloudPtr cloudPtr);

	size_t GetTaskSize();

private:
	std::queue<std::shared_ptr<WriteTask>> _tasks;
	std::mutex _mutex;
	std::condition_variable _cv;
	std::atomic<bool> _running;

	std::chrono::milliseconds batchInterval{1000}; // 每100ms检查一次任务队列
};



#endif // POINT_PROCESS_THREAD_H_