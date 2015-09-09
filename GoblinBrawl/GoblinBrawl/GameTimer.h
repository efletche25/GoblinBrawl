#pragma once
class GameTimer {
public:
	GameTimer();
	
	//Returns elapsed time since reset noy including time when the clock is stopped
	float TotalTime() const;
	
	//Return delta time
	float DT() const;

	void Reset();
	void Start();
	void Stop();
	void Tick();
private:
	double dt;

	__int64 baseTime;
	__int64 pausedTime;
	__int64 stopTime;
	__int64 prevTime;
	__int64 currTime;
	__int64	freq;

	bool stopped;
};

