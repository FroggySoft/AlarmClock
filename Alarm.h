#ifndef ALARM_H
#define ALARM_H

#include "Arduino.h"
#include "DateTime.h"

class Alarm
{
public:
	Alarm(uint8_t aIndex);
	Alarm(uint8_t aIndex, uint8_t  aHours, uint8_t aMinutes, uint8_t aDay);
  void StoreAlarm();
  void Restore();

  void NextDay(bool aDirection);
  void NextHour(bool aDirection);
  void NextMinute(bool aDirection);
  
  bool IsActive(DateTime aTime);
  bool IsEnabled();
	const char* GetString();

  uint8_t  mHours;
  uint8_t  mMinutes;
  uint8_t  mDay;   // 0..6 , 7=all weekdays, 8=all days, 9=deactivated

private:
  uint16_t mOffset;
};
#endif
