#ifndef MP3_H
#define MP3_H

#include "Arduino.h"

class MP3
{
public:
	void Init();
	void Run();
	void Start();
	void Stop();
	void Next();
  bool IsActive();
	void SetVolume(byte aVolume);
	void IncreaseVolume(byte aVolume);

private:
	void 	 execute(byte CMD, uint16_t Par);
	uint32_t readPlayer(byte* aCommand);
	
	byte mNumberOfTracks;
	byte mVolume;
	bool mActive;
};

#endif
