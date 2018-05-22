#ifndef DFC77_H
#define DFC77_H

#include "DateTime.h"

#define MSG_SIZE      		60
#define WEATHER_SIZE  		82
#define WEATHER_INFO_SIZE   3

#define DCF_STATE_SAMPLING    0
#define DCF_STATE_NEWSECOND   1
#define DCF_STATE_NEWMINUTE   2
#define DCF_STATE_NEWWEATHER  3

class Dcf77
{
public:
	void Init(int aPort);
	
	byte Run(void);
	
	bool GetWeatherInfo(byte* aWeather);
	byte GetWeatherArea();
	byte GetWeatherSection();
	
	DateTime GetTime(void);
  bool TimeIsValid();

private:

  void checkMessage(bool* aMessage);
  bool addToWeatherInfo(bool* aMessage);
  void copyWeatherInfo(bool* aMessage, byte aIndex);
  bool checkValidMessage(bool* aMessage);
  bool checkParity(bool* aMessage, byte aStart, byte aEnd);
  byte getMinute(bool* aMessage);
  byte flipByte(byte aByte);
  void CopyTimeToByteUint(byte* data, byte* key, struct DataContainer* container);
  void ShiftTimeRight(int round, struct DataContainer* container);
  void ExpandR(struct DataContainer* container);
  void CompressKey(struct DataContainer* container);
  void DoSbox(struct DataContainer* container);
  void DoPbox(struct DataContainer* container);
  void Decrypt(byte* cipher, byte* key, byte* result);
  
  byte getArea(DateTime aTime);
  byte getSection(DateTime aTime);
  unsigned int getMinutesSince2200(DateTime aTime);
  byte GetDecFromBcd(bool* aMessage,byte aStart);

  bool mPrevDcf;
  unsigned long mTimePosEdge;
  unsigned long mTimeNegEdge;
  byte mBitCounter;
  bool mBitReceived;
  unsigned long mTimePrevBit;
  
  bool mMessage[MSG_SIZE];
  bool mWeatherData[WEATHER_SIZE];
  byte mWeatherArea;
  byte mWeatherSection;
  
  int mPort;
  
  DateTime mTime;
  bool mTimeValid;
};

#endif
