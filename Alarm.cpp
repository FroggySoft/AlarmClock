#include "ProjSettings.h"
#include "Alarm.h"
#include <EEPROM.h>

//#define DEBUG
#define NR_OF_ALARMS_DAYS 10
#define ALARM_WEEKDAYS    7
#define ALARM_EVERYDAY    8
#define ALARM_DISABLED    (NR_OF_ALARMS_DAYS-1)

const char alarmdays[NR_OF_ALARMS_DAYS][3] = {"ma","di","wo","do","vr","za","zo","wd"," *"," -"};

Alarm::Alarm(uint8_t aIndex):
  mOffset(aIndex*4),
  mHours(0),
  mMinutes(0),
  mDay(ALARM_DISABLED)
{
  Restore();
}

Alarm::Alarm(uint8_t aIndex, uint8_t  aHours, uint8_t aMinutes, uint8_t aDay):
  mOffset(aIndex*4),
  mHours(aHours),
  mMinutes(aMinutes),
  mDay(aDay)
{
}

void Alarm::NextDay(bool aDirection)
{
  if(aDirection)
  {
    mDay++;
    if(mDay >= NR_OF_ALARMS_DAYS)
      mDay = 0;
  }
  else if(mDay>0)
  {
    mDay--;
  }
}
void Alarm::NextHour(bool aDirection)
{
  if(aDirection)
  { 
    mHours++;
    if(mHours>=24)
      mHours=0;
  }
  else
  {
    if(mHours==0)
      mHours=24;
    mHours--;
  }
}

void Alarm::NextMinute(bool aDirection)
{
  if(aDirection)
  { 
    mMinutes+=5;
    if(mMinutes>59)
      mMinutes=0;
  }
  else
  {
    if(mMinutes<=4)
      mMinutes=60;
    mMinutes-=5;
  }
}

bool Alarm::IsActive(DateTime aTime)
{
  
  if( IsEnabled() &&
      (mHours == aTime.hh) &&
      (mMinutes == aTime.mm))
  {
     if((mDay==ALARM_EVERYDAY) ||  // every day
        (mDay==ALARM_WEEKDAYS && aTime.dayOfWeek()<6) ||   // weekdays
        (mDay==aTime.dayOfWeek()))
        {
          return true;
        }
  }
  return false;
}

bool Alarm::IsEnabled()
{
  return (mDay<ALARM_DISABLED);
}

void Alarm::StoreAlarm()
{
  EEPROM.update(mOffset, mHours);
  EEPROM.update(mOffset+1, mMinutes);
  EEPROM.update(mOffset+2, mDay);
}

void Alarm::Restore()
{
  mHours = 0;
  mMinutes = 0;
  mDay = ALARM_DISABLED;

  uint8_t lHours= EEPROM.read(mOffset);
  uint8_t lMinutes = EEPROM.read(mOffset+1);
  uint8_t lDay = EEPROM.read(mOffset+2);
  
  if ((lHours<24) && (lMinutes<60) && (lDay<NR_OF_ALARMS_DAYS))
  {
    mHours = lHours;
    mMinutes = lMinutes;
    mDay = lDay;
  }
}

static char mStr[10];
const char* Alarm::GetString()
{
  sprintf(mStr,"%2s %02d:%02d", alarmdays[mDay], mHours,mMinutes);
  return mStr; 
}	
