#ifndef BT_THREAD_SUPPORT_INTERFACE_H
#define BT_THREAD_SUPPORT_INTERFACE_H

class btCriticalSection
{
public:
	btCriticalSection() {}
	virtual ~btCriticalSection() {}

	virtual void lock() = 0;
	virtual void unlock() = 0;
};

class btThreadSupportInterface
{
public:
	virtual ~btThreadSupportInterface() {}

	virtual int getNumWorkerThreads() const = 0;            // number of worker threads (total number of logical processors - 1)
	virtual int getCacheFriendlyNumThreads() const = 0;     // the number of logical processors sharing a single L3 cache
	virtual int getLogicalToPhysicalCoreRatio() const = 0;  // the number of logical processors per physical processor (usually 1 or 2)
	virtual void runTask(int threadIndex, void* userData) = 0;
	virtual void waitForAllTasks() = 0;

	virtual btCriticalSection* createCriticalSection() = 0;
	virtual void deleteCriticalSection(btCriticalSection* criticalSection) = 0;

	typedef void (*ThreadFunc)(void* userPtr);

	struct ConstructionInfo
	{
		ConstructionInfo(const char* uniqueName,
						 ThreadFunc userThreadFunc,
						 int threadStackSize = 65535)
			: m_uniqueName(uniqueName),
			  m_userThreadFunc(userThreadFunc),
			  m_threadStackSize(threadStackSize)
		{
		}

		const char* m_uniqueName;
		ThreadFunc m_userThreadFunc;
		int m_threadStackSize;
	};

	static btThreadSupportInterface* create(const ConstructionInfo& info);
};

#endif  //BT_THREAD_SUPPORT_INTERFACE_H
