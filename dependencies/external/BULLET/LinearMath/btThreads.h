#ifndef BT_THREADS_H
#define BT_THREADS_H

#include "btScalar.h"  // has definitions like SIMD_FORCE_INLINE

#if defined(_MSC_VER) && _MSC_VER >= 1600
// give us a compile error if any signatures of overriden methods is changed
#define BT_OVERRIDE override
#endif

#ifndef BT_OVERRIDE
#define BT_OVERRIDE
#endif

// Don't set this to larger than 64, without modifying btThreadSupportPosix
// and btThreadSupportWin32. They use UINT64 bit-masks.
const unsigned int BT_MAX_THREAD_COUNT = 64;  // only if BT_THREADSAFE is 1

// for internal use only
bool btIsMainThread();
bool btThreadsAreRunning();
unsigned int btGetCurrentThreadIndex();
void btResetThreadIndexCounter();  // notify that all worker threads have been destroyed

///
/// btSpinMutex -- lightweight spin-mutex implemented with atomic ops, never puts
///               a thread to sleep because it is designed to be used with a task scheduler
///               which has one thread per core and the threads don't sleep until they
///               run out of tasks. Not good for general purpose use.
///
class btSpinMutex
{
	int mLock;

public:
	btSpinMutex()
	{
		mLock = 0;
	}
	void lock();
	void unlock();
	bool tryLock();
};

//
// NOTE: btMutex* is for internal Bullet use only
//
// If BT_THREADSAFE is undefined or 0, should optimize away to nothing.
// This is good because for the single-threaded build of Bullet, any calls
// to these functions will be optimized out.
//
// However, for users of the multi-threaded build of Bullet this is kind
// of bad because if you call any of these functions from external code
// (where BT_THREADSAFE is undefined) you will get unexpected race conditions.
//
SIMD_FORCE_INLINE void btMutexLock(btSpinMutex* mutex)
{
	(void)mutex;
}

SIMD_FORCE_INLINE void btMutexUnlock(btSpinMutex* mutex)
{
	(void)mutex;
}

SIMD_FORCE_INLINE bool btMutexTryLock(btSpinMutex* mutex)
{
	(void)mutex;
	return true;
}

//
// btIParallelForBody -- subclass this to express work that can be done in parallel
//
class btIParallelForBody
{
public:
	virtual ~btIParallelForBody() {}
	virtual void forLoop(int iBegin, int iEnd) const = 0;
};

//
// btIParallelSumBody -- subclass this to express work that can be done in parallel
//                       and produces a sum over all loop elements
//
class btIParallelSumBody
{
public:
	virtual ~btIParallelSumBody() {}
	virtual btScalar sumLoop(int iBegin, int iEnd) const = 0;
};

//
// btITaskScheduler -- subclass this to implement a task scheduler that can dispatch work to
//                     worker threads
//
class btITaskScheduler
{
public:
	btITaskScheduler(const char* name);
	virtual ~btITaskScheduler() {}
	const char* getName() const { return m_name; }

	virtual int getMaxNumThreads() const = 0;
	virtual int getNumThreads() const = 0;
	virtual void setNumThreads(int numThreads) = 0;
	virtual void parallelFor(int iBegin, int iEnd, int grainSize, const btIParallelForBody& body) = 0;
	virtual btScalar parallelSum(int iBegin, int iEnd, int grainSize, const btIParallelSumBody& body) = 0;
	virtual void sleepWorkerThreadsHint() {}  // hint the task scheduler that we may not be using these threads for a little while

	// internal use only
	virtual void activate();
	virtual void deactivate();

protected:
	const char* m_name;
	unsigned int m_savedThreadCounter;
	bool m_isActive;
};

// set the task scheduler to use for all calls to btParallelFor()
// NOTE: you must set this prior to using any of the multi-threaded "Mt" classes
void btSetTaskScheduler(btITaskScheduler* ts);

// get the current task scheduler
btITaskScheduler* btGetTaskScheduler();

// get non-threaded task scheduler (always available)
btITaskScheduler* btGetSequentialTaskScheduler();

// create a default task scheduler (Win32 or pthreads based)
btITaskScheduler* btCreateDefaultTaskScheduler();

// get OpenMP task scheduler (if available, otherwise returns null)
btITaskScheduler* btGetOpenMPTaskScheduler();

// get Intel TBB task scheduler (if available, otherwise returns null)
btITaskScheduler* btGetTBBTaskScheduler();

// get PPL task scheduler (if available, otherwise returns null)
btITaskScheduler* btGetPPLTaskScheduler();

// btParallelFor -- call this to dispatch work like a for-loop
//                 (iterations may be done out of order, so no dependencies are allowed)
void btParallelFor(int iBegin, int iEnd, int grainSize, const btIParallelForBody& body);

// btParallelSum -- call this to dispatch work like a for-loop, returns the sum of all iterations
//                 (iterations may be done out of order, so no dependencies are allowed)
btScalar btParallelSum(int iBegin, int iEnd, int grainSize, const btIParallelSumBody& body);

#endif
