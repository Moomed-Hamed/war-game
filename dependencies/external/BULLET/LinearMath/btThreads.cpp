#include "btThreads.h"
#include "btQuickprof.h"
#include <algorithm>  // for min and max

// These should not be called ever
void btSpinMutex::lock()
{
	btAssert(!"unimplemented btSpinMutex::lock() called");
}

void btSpinMutex::unlock()
{
	btAssert(!"unimplemented btSpinMutex::unlock() called");
}

bool btSpinMutex::tryLock()
{
	btAssert(!"unimplemented btSpinMutex::tryLock() called");
	return true;
}

#define THREAD_LOCAL_STATIC static

struct ThreadsafeCounter
{
	unsigned int mCounter;
	btSpinMutex mMutex;

	ThreadsafeCounter()
	{
		mCounter = 0;
		--mCounter;  // first count should come back 0
	}

	unsigned int getNext()
	{
		// no need to optimize this with atomics, it is only called ONCE per thread!
		mMutex.lock();
		mCounter++;
		if (mCounter >= BT_MAX_THREAD_COUNT)
		{
			btAssert(!"thread counter exceeded");
			// wrap back to the first worker index
			mCounter = 1;
		}
		unsigned int val = mCounter;
		mMutex.unlock();
		return val;
	}
};

static btITaskScheduler* gBtTaskScheduler=0;
static int gThreadsRunningCounter = 0;  // useful for detecting if we are trying to do nested parallel-for calls
static btSpinMutex gThreadsRunningCounterMutex;
static ThreadsafeCounter gThreadCounter;

//
// BT_DETECT_BAD_THREAD_INDEX tries to detect when there are multiple threads assigned the same thread index.
//
// BT_DETECT_BAD_THREAD_INDEX is a developer option to test if
// certain assumptions about how the task scheduler manages its threads
// holds true.
// The main assumption is:
//   - when the threadpool is resized, the task scheduler either
//      1. destroys all worker threads and creates all new ones in the correct number, OR
//      2. never destroys a worker thread
//
// We make that assumption because we can't easily enumerate the worker threads of a task scheduler
// to assign nice sequential thread-indexes. We also do not get notified if a worker thread is destroyed,
// so we can't tell when a thread-index is no longer being used.
// We allocate thread-indexes as needed with a sequential global thread counter.
//
// Our simple thread-counting scheme falls apart if the task scheduler destroys some threads but
// continues to re-use other threads and the application repeatedly resizes the thread pool of the
// task scheduler.
// In order to prevent the thread-counter from exceeding the global max (BT_MAX_THREAD_COUNT), we
// wrap the thread counter back to 1. This should only happen if the worker threads have all been
// destroyed and re-created.
//
// BT_DETECT_BAD_THREAD_INDEX only works for Win32 right now,
// but could be adapted to work with pthreads
#define BT_DETECT_BAD_THREAD_INDEX 0

// return a unique index per thread, main thread is 0, worker threads are in [1, BT_MAX_THREAD_COUNT)
unsigned int btGetCurrentThreadIndex()
{
	const unsigned int kNullIndex = ~0U;
	THREAD_LOCAL_STATIC unsigned int sThreadIndex = kNullIndex;
	if (sThreadIndex == kNullIndex)
	{
		sThreadIndex = gThreadCounter.getNext();
		btAssert(sThreadIndex < BT_MAX_THREAD_COUNT);
	}

	return sThreadIndex;
}

bool btIsMainThread()
{
	return btGetCurrentThreadIndex() == 0;
}

void btResetThreadIndexCounter()
{
	// for when all current worker threads are destroyed
	btAssert(btIsMainThread());
	gThreadCounter.mCounter = 0;
}

btITaskScheduler::btITaskScheduler(const char* name)
{
	m_name = name;
	m_savedThreadCounter = 0;
	m_isActive = false;
}

void btITaskScheduler::activate()
{
	// gThreadCounter is used to assign a thread-index to each worker thread in a task scheduler.
	// The main thread is always thread-index 0, and worker threads are numbered from 1 to 63 (BT_MAX_THREAD_COUNT-1)
	// The thread-indexes need to be unique amongst the threads that can be running simultaneously.
	// Since only one task scheduler can be used at a time, it is OK for a pair of threads that belong to different
	// task schedulers to share the same thread index because they can't be running at the same time.
	// So each task scheduler needs to keep its own thread counter value
	if (!m_isActive)
	{
		gThreadCounter.mCounter = m_savedThreadCounter;  // restore saved thread counter
		m_isActive = true;
	}
}

void btITaskScheduler::deactivate()
{
	if (m_isActive)
	{
		m_savedThreadCounter = gThreadCounter.mCounter;  // save thread counter
		m_isActive = false;
	}
}

void btPushThreadsAreRunning()
{
	gThreadsRunningCounterMutex.lock();
	gThreadsRunningCounter++;
	gThreadsRunningCounterMutex.unlock();
}

void btPopThreadsAreRunning()
{
	gThreadsRunningCounterMutex.lock();
	gThreadsRunningCounter--;
	gThreadsRunningCounterMutex.unlock();
}

bool btThreadsAreRunning()
{
	return gThreadsRunningCounter != 0;
}

void btSetTaskScheduler(btITaskScheduler* ts)
{
	int threadId = btGetCurrentThreadIndex();  // make sure we call this on main thread at least once before any workers run
	if (threadId != 0)
	{
		btAssert(!"btSetTaskScheduler must be called from the main thread!");
		return;
	}
	if (gBtTaskScheduler)
	{
		// deactivate old task scheduler
		gBtTaskScheduler->deactivate();
	}
	gBtTaskScheduler = ts;
	if (ts)
	{
		// activate new task scheduler
		ts->activate();
	}
}

btITaskScheduler* btGetTaskScheduler()
{
	return gBtTaskScheduler;
}

void btParallelFor(int iBegin, int iEnd, int grainSize, const btIParallelForBody& body)
{
	// non-parallel version of btParallelFor
	btAssert(!"called btParallelFor in non-threadsafe build. enable BT_THREADSAFE");
	body.forLoop(iBegin, iEnd);
}

btScalar btParallelSum(int iBegin, int iEnd, int grainSize, const btIParallelSumBody& body)
{
	// non-parallel version of btParallelSum
	btAssert(!"called btParallelFor in non-threadsafe build. enable BT_THREADSAFE");
	return body.sumLoop(iBegin, iEnd);
}

///
/// btTaskSchedulerSequential -- non-threaded implementation of task scheduler
///                              (really just useful for testing performance of single threaded vs multi)
///
class btTaskSchedulerSequential : public btITaskScheduler
{
public:
	btTaskSchedulerSequential() : btITaskScheduler("Sequential") {}
	virtual int getMaxNumThreads() const BT_OVERRIDE { return 1; }
	virtual int getNumThreads() const BT_OVERRIDE { return 1; }
	virtual void setNumThreads(int numThreads) BT_OVERRIDE {}
	virtual void parallelFor(int iBegin, int iEnd, int grainSize, const btIParallelForBody& body) BT_OVERRIDE
	{
		BT_PROFILE("parallelFor_sequential");
		body.forLoop(iBegin, iEnd);
	}
	virtual btScalar parallelSum(int iBegin, int iEnd, int grainSize, const btIParallelSumBody& body) BT_OVERRIDE
	{
		BT_PROFILE("parallelSum_sequential");
		return body.sumLoop(iBegin, iEnd);
	}
};

// create a non-threaded task scheduler (always available)
btITaskScheduler* btGetSequentialTaskScheduler()
{
	static btTaskSchedulerSequential sTaskScheduler;
	return &sTaskScheduler;
}

// create an OpenMP task scheduler (if available, otherwise returns null)
btITaskScheduler* btGetOpenMPTaskScheduler()
{
	return NULL;
}

// create an Intel TBB task scheduler (if available, otherwise returns null)
btITaskScheduler* btGetTBBTaskScheduler()
{
	return NULL;
}

// create a PPL task scheduler (if available, otherwise returns null)
btITaskScheduler* btGetPPLTaskScheduler()
{
	return NULL;
}
