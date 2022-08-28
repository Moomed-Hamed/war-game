/*

***************************************************************************************************
**
** profile.cpp
**
** Real-Time Hierarchical Profiling for Game Programming Gems 3
**
** by Greg Hjelstrom & Byon Garrabrant
**
***************************************************************************************************/

// Credits: The Clock class was inspired by the Timer classes in
// Ogre (www.ogre3d.org).

#include "btQuickprof.h"
#include "btThreads.h"

#define BT_USE_WINDOWS_TIMERS
#define WIN32_LEAN_AND_MEAN
#define NOWINRES
#define NOMCX
#define NOIME

#include <windows.h>

#include <time.h>

#define mymin(a, b) (a > b ? a : b)

struct btClockData
{
	LARGE_INTEGER mClockFrequency;
	LONGLONG mStartTick;
	LARGE_INTEGER mStartTime;
};

///The btClock is a portable basic clock that measures accurate time in seconds, use for profiling.
btClock::btClock()
{
	m_data = new btClockData;
	QueryPerformanceFrequency(&m_data->mClockFrequency);
	reset();
}

btClock::~btClock()
{
	delete m_data;
}

btClock::btClock(const btClock& other)
{
	m_data = new btClockData;
	*m_data = *other.m_data;
}

btClock& btClock::operator=(const btClock& other)
{
	*m_data = *other.m_data;
	return *this;
}

/// Resets the initial reference time.
void btClock::reset()
{
	QueryPerformanceCounter(&m_data->mStartTime);
	m_data->mStartTick = GetTickCount64();
}

/// Returns the time in ms since the last call to reset or since
/// the btClock was created.
unsigned long long int btClock::getTimeMilliseconds()
{
	LARGE_INTEGER currentTime;
	QueryPerformanceCounter(&currentTime);
	LONGLONG elapsedTime = currentTime.QuadPart - m_data->mStartTime.QuadPart;
	// Compute the number of millisecond ticks elapsed.
	unsigned long msecTicks = (unsigned long)(1000 * elapsedTime / m_data->mClockFrequency.QuadPart);

	return msecTicks;
}

/// Returns the time in us since the last call to reset or since
/// the Clock was created.
unsigned long long int btClock::getTimeMicroseconds()
{
	//see https://msdn.microsoft.com/en-us/library/windows/desktop/dn553408(v=vs.85).aspx
	LARGE_INTEGER currentTime, elapsedTime;

	QueryPerformanceCounter(&currentTime);
	elapsedTime.QuadPart = currentTime.QuadPart -
						   m_data->mStartTime.QuadPart;
	elapsedTime.QuadPart *= 1000000;
	elapsedTime.QuadPart /= m_data->mClockFrequency.QuadPart;

	return (unsigned long long)elapsedTime.QuadPart;
}

unsigned long long int btClock::getTimeNanoseconds()
{
	//see https://msdn.microsoft.com/en-us/library/windows/desktop/dn553408(v=vs.85).aspx
	LARGE_INTEGER currentTime, elapsedTime;

	QueryPerformanceCounter(&currentTime);
	elapsedTime.QuadPart = currentTime.QuadPart -
						   m_data->mStartTime.QuadPart;
	elapsedTime.QuadPart *= 1000000000;
	elapsedTime.QuadPart /= m_data->mClockFrequency.QuadPart;

	return (unsigned long long)elapsedTime.QuadPart;
}

/// Returns the time in s since the last call to reset or since
/// the Clock was created.
btScalar btClock::getTimeSeconds()
{
	static const btScalar microseconds_to_seconds = btScalar(0.000001);
	return btScalar(getTimeMicroseconds()) * microseconds_to_seconds;
}

void btEnterProfileZoneDefault(const char* name)
{
}
void btLeaveProfileZoneDefault()
{
}

unsigned int btQuickprofGetCurrentThreadIndex2()
{
	const unsigned int kNullIndex = ~0U;

	__declspec(thread) static unsigned int sThreadIndex = kNullIndex;

	static int gThreadCounter = 0;

	if (sThreadIndex == kNullIndex)
	{
		sThreadIndex = gThreadCounter++;
	}
	return sThreadIndex;
}

static btEnterProfileZoneFunc* bts_enterFunc = btEnterProfileZoneDefault;
static btLeaveProfileZoneFunc* bts_leaveFunc = btLeaveProfileZoneDefault;

void btEnterProfileZone(const char* name)
{
	(bts_enterFunc)(name);
}
void btLeaveProfileZone()
{
	(bts_leaveFunc)();
}

btEnterProfileZoneFunc* btGetCurrentEnterProfileZoneFunc()
{
	return bts_enterFunc;
}
btLeaveProfileZoneFunc* btGetCurrentLeaveProfileZoneFunc()
{
	return bts_leaveFunc;
}

void btSetCustomEnterProfileZoneFunc(btEnterProfileZoneFunc* enterFunc)
{
	bts_enterFunc = enterFunc;
}
void btSetCustomLeaveProfileZoneFunc(btLeaveProfileZoneFunc* leaveFunc)
{
	bts_leaveFunc = leaveFunc;
}

CProfileSample::CProfileSample(const char* name)
{
	btEnterProfileZone(name);
}

CProfileSample::~CProfileSample(void)
{
	btLeaveProfileZone();
}
