#include "stdafx.h"
#include "GameTimer.h"


GameTimer::GameTimer() :
secondsPerCount(0.0),
dt(-1.0),
baseTime(0.0),
pausedTime(0.0),
prevTime(0.0),
currTime(0.0),
stopped(false)
{
	__int64 countsPerSecond;
	QueryPerformanceFrequency( (LARGE_INTEGER*)&countsPerSecond );
	secondsPerCount = 1.0/(double)countsPerSecond;
}


void GameTimer::Start() {
	__int64 startTime;
	QueryPerformanceCounter( (LARGE_INTEGER*)&startTime );
	if( stopped ) {
		pausedTime += (startTime-stopTime);
		prevTime = startTime;
		stopTime = 0;
		stopped = false;
	}
}

void GameTimer::Stop() {
	if( !stopped ) {
		__int64 currTime;
		QueryPerformanceCounter( (LARGE_INTEGER*)&currTime );
		stopTime = currTime;
		stopped = true;
	}
}

void GameTimer::Tick() {
	if( stopped ) {
		dt = 0.0;
		return;
	}

	__int64 queryTime;
	QueryPerformanceCounter( (LARGE_INTEGER*)&queryTime );
	currTime = queryTime;
	dt = (currTime-prevTime)*secondsPerCount;
	prevTime = currTime;

	// Force nonnegative.  The DXSDK's CDXUTTimer mentions that if the 
	// processor goes into a power save mode or we get shuffled to another
	// processor, then mDeltaTime can be negative.
	if( dt<0.0 ) {
		dt = 0.0;
	}
}

void GameTimer::Reset() {
	__int64 queryTime;
	QueryPerformanceCounter( (LARGE_INTEGER*)&queryTime );
	baseTime = queryTime;
	prevTime = queryTime;
	stopTime = 0;
	stopped = false;
}

float GameTimer::DT() const {
	return (float)dt;
}

float GameTimer::TotalTime() const {
	if( stopped ) {
		return (float)(((stopTime-pausedTime)-baseTime)*secondsPerCount);
	} else {
		return (float)(((currTime-pausedTime)-baseTime)*secondsPerCount);
	}
}

