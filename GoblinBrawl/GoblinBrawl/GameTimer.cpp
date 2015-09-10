#include "stdafx.h"
#include "GameTimer.h"

GameTimer::GameTimer() :
dt( -1.0 ),
baseTime( 0 ),
pausedTime( 0 ),
prevTime( 0 ),
currTime( 0 ),
freq( 0 ),
stopped(false),
canTick(false)
{
	LARGE_INTEGER tempFreq;
	QueryPerformanceFrequency( &tempFreq );
	freq = tempFreq.QuadPart;
}


void GameTimer::Start() {
	LARGE_INTEGER startTime;
	QueryPerformanceCounter( &startTime );
	if( stopped ) {
		pausedTime += (startTime.QuadPart-stopTime);
		prevTime = startTime.QuadPart;
		stopTime = 0;
		stopped = false;
	}
}

void GameTimer::Stop() {
	if( !stopped ) {
		LARGE_INTEGER currTime;
		QueryPerformanceCounter( &currTime );
		stopTime = currTime.QuadPart;
		stopped = true;
	}
}

void GameTimer::Tick() {
	if( stopped ) {
		dt = 0.0;
		return;
	}

	LARGE_INTEGER queryTime;
	QueryPerformanceCounter( &queryTime );
	currTime = queryTime.QuadPart;

	__int64 elapsedMillisecond = (currTime-prevTime);
	dt = (double)elapsedMillisecond/freq;
	prevTime = currTime;
	canTick = true;

	// Force nonnegative.  The DXSDK's CDXUTTimer mentions that if the 
	// processor goes into a power save mode or we get shuffled to another
	// processor, then mDeltaTime can be negative.
	if( dt<0.0 ) {
		dt = 0.0;
	}
}

void GameTimer::Reset() {
	LARGE_INTEGER queryTime;
	QueryPerformanceCounter( &queryTime );
	baseTime = queryTime.QuadPart;
	prevTime = queryTime.QuadPart;
	stopTime = 0;
	stopped = false;
}

float GameTimer::DT() {
	return (float)dt;
}

float GameTimer::TotalTime() const {
	if( stopped ) {
		return (float)(((stopTime-pausedTime)-baseTime)/freq);
	} else {
		return (float)(((currTime-pausedTime)-baseTime)/freq);
	}
}

