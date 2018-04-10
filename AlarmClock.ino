
//#define DEBUG
//#define DEBUG_INPUT

#include "Arduino.h"
#include "Adafruit_GFX.h"    // Core graphics library
#include "Adafruit_TftLCD.h" // Hardware-specific library
#include <Wire.h>
#include <EEPROM.h>
#include "DateTime.h"
#include "RTClib.h"
#include "Dcf77.h"
#include "Weather.h"
#include "Bmp180.h"
#include "Ads1015.h"
#include "MP3.h"
#include "images/images.h"

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define PIN_PIR     13  // also buildin led
#define PIR_ACTIVE  HIGH

#define PIN_DCF     12

#define PIN_RX  0   // used for debug
#define PIN_TX  1

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7

// Assign human-readable names to some common 16-bit color values:
// 5 bits red , 6 bits green, 5 bits blue
#define	BLACK   0x0000
#define	BLUE    0x001F
#define LIGHTBLUE    0x187F
#define	RED     0xF800
#define LIGHTRED 0xFACB
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GRAY1   0x38E7
#define GRAY    0xD69A
#define LIGHTGRAY 0xE71C

#define TFT_CHAR_WIDTH  6
#define TFT_CHAR_HEIGHT 8

#define TIME_SIZE   12
#define STRLEN_TIME 5
#define WIDTH_TIME  (6*TIME_SIZE*STRLEN_TIME)
#define HEIGHT_TIME (8*TIME_SIZE)
#define XPOS_TIME   10
#define YPOS_TIME   100

#define DATE_SIZE   2
#define STRLEN_DATE (10+12)
#define WIDTH_DATE  (6*DATE_SIZE*STRLEN_DATE)
#define HEIGHT_DATE (8*DATE_SIZE)
#define XPOS_DATE   ((480-WIDTH_DATE)/2)
#define YPOS_DATE   (YPOS_TIME+HEIGHT_TIME) // 300

#define NR_OF_ALARMS 6
#define ALARM_SIZE   2
#define STRLEN_ALARM 8
#define WIDTH_ALARM  (6*ALARM_SIZE*STRLEN_ALARM)
#define HEIGHT_ALARM (8*ALARM_SIZE)
#define XPOS_ALARM   (480-WIDTH_ALARM)
#define YPOS_ALARM   100

#define MIN_PRESSURE  960
#define MAX_PRESSURE  1040
#define NR_OF_PRESSURES (3*24*3)     // last 3 days, 3 values/hour
#define XPOS_PRES_GRAPH 0
#define YPOS_PRES_GRAPH 0
#define HEIGHT_PRES_GRAPH (MAX_PRESSURE-MIN_PRESSURE)
//#define WIDTH_PRES_GRAPH NR_OF_PRESSURES
#define WIDTH_PRES_GRAPH 220
#define GRAPHCOLOR   WHITE  //0x841F // RGB = 127,127,255 ->  1000 0100 0001 1111


#define TEMPPRES_SIZE   2
#define XPOS_PRES       (XPOS_PRES_GRAPH+WIDTH_PRES_GRAPH+20)
#define YPOS_PRES       20
#define XPOS_TEMP       XPOS_PRES
#define YPOS_TEMP       (YPOS_PRES+8*TEMPPRES_SIZE+4)
#define WIDTH_TEMPPRES  (6*TEMPPRES_SIZE*8)        //1040 mBar
#define HEIGHT_TEMPPRES HEIGHT_PRES_GRAPH

#define XPOS_WEATHER     0
#define YPOS_WEATHER     (YPOS_DATE+HEIGHT_DATE+20)
#define WIDTH_WEATHER    (XPOS_ALARM/8)
#define HEIGHT_WEATHER   (320-YPOS_WEATHER-1)

#define XPOS_SUNMOON     (XPOS_PRES+WIDTH_TEMPPRES+20)
#define XPOS_MOON        (XPOS_SUNMOON+20)
#define XPOS_SUN         (XPOS_SUNMOON+5*6*DATE_SIZE)
#define YPOS_SUN         0
#define YPOS_SUNMOON     10
#define WIDTH_SUNMOON    (480-XPOS_SUNMOON)
#define HEIGHT_SUNMOON   HEIGHT_PRES_GRAPH
#define MOON_RADIUS 16

#define ALARM_START_VOLUME 10

typedef struct
{
  uint8_t  hours;
  uint8_t  minutes;
  uint8_t  day;   // 0..6 , 7=all weekdays, 8=all days
  bool     active;
} sAlarm;

typedef enum
{
  MenuIdle,
  MenuSelectAlarm,
  MenuChangeAlarmDay,
  MenuChangeAlarmHour,
  MenuChangeAlarmMinute,
  MenuChangeAlarmActive,
  MenuStoreAlarm,
  MenuRestoreAlarm
} sMenuStates;

char mPrevMinute=0;
DateTime mPrevDate;
const char zondag[] PROGMEM = "zondag";
const char maandag[] PROGMEM = "maandag";
const char dinsdag[] PROGMEM = "dinsdag";
const char woensdag[] PROGMEM = "woensdag";
const char donderdag[] PROGMEM = "donderdag";
const char vrijdag[] PROGMEM = "vrijdag";
const char zaterdag[] PROGMEM = "zaterdag";
const char* const days_table[] PROGMEM = {zondag,maandag,dinsdag,woensdag,donderdag,vrijdag,zaterdag};

const char alarmdays[9][3] = {"zo","ma","di","wo","do","vr","za"," w"," *"};
sAlarm mAlarms[NR_OF_ALARMS];
bool mAlarmActive = false;
byte mAlarmTime = 0;
bool mForceUpdate = true;
bool mPirState = false;
bool mSpare1State = false;
bool mSpare2State = false;
bool mKeyUp = false;
bool mKeyDown = false;
bool mKeyLeft = false;
bool mKeyRight = false;

sMenuStates    mMenuState = MenuIdle;
unsigned int   mMenuTimeMs = 0;
byte           mMenuSelectedAlarm = -1;

byte mHistory[NR_OF_PRESSURES];
int  mHistoryIndex = 0;
byte mPrevQuarter = 0;
bool mDcfStatus = false;

Adafruit_TFTLCD mTft(LCD_CS, LCD_CD, LCD_WR, LCD_RD);
RTC_DS1307 mRtc;
Dcf77      mDcf;
Bmp180     mBmp180;
Ads1015    mAdc;
MP3        mMp3;


void setup(void)
{
#ifdef DEBUG
  Serial.begin(9600);
#endif

  pinMode(PIN_PIR,INPUT);

  Wire.begin();
  mRtc.begin();

  mTft.reset();
  delay(100);
  mTft.begin();//identifier);
  delay(10);
  mTft.fillScreen(BLACK);
  mTft.setTextSize(2);
  
  ReadAlarms();
  displayAlarms();

  mBmp180.Init();
  for( int i=0; i<NR_OF_PRESSURES; i++)
  {
    mHistory[i++]=0;
    mHistory[i]=HEIGHT_PRES_GRAPH-1;
  }
  mHistoryIndex = 0;
  displayPressureGraph();

  DateTime now = mRtc.now();
  handleTime(now,true);

  mDcf.Init(PIN_DCF);
  mMp3.Init();

  // Test Data
//  {0x84,0x08,0x40}    // weer_1 + weer_2 , 15% , -20   =  1000 0100 0000 100 0 01000000
//  {0xC2,0x08,0x30}    // weer_3 + weer_4 , N0-2 , -10 = 1100 0010 0000 100 0 0011 0000
//  {0xA6,0x0C,0x68}    // weer_5 + weer_6 , 45% , 0     = 1010 0110 0000 110 0 01101000
//  {0xE1,0x4C,0x04}    // weer_7 + weer_8 , E5-6, +10  = 1110 0001 0100 110 0 00000100
//  {0x95,0x04,0x44}    // weer_9 + weer_10 , 60% , +12  = 1001 0101 0000 001 0 01000100
//  {0xD3,0xCA,0x54}    // weer_11 + weer_12 , SE8, +20 = 1101 0011 1100 1010 01010100
//  {0xD7,0x0A,0xF4}    // weer_13 + weer_14 , 75% , +25 = 1011 0111 0000 101 0 11110100
//  {0xF8,0xAE,0x6C}    // weer_15 + weer_1 , SW >9 , +30 = 1111 1000 1010 1110 01101100

//  byte info0[] = {0x84,0x08,0x40};
//  byte info1[] = {0x84,0x08,0x30};;// {0xC2,0x08,0x30};
//  byte info2[] = {0xA6,0x0C,0x68};
//  byte info3[] = {0xA6,0x4C,0x04};//{0xE1,0x4C,0x04};
//  byte info4[] = {0x95,0x02,0x44};
//  byte info5[] = {0x95,0xCA,0x54};//{0xD3,0xCA,0x54};
//  byte info6[] = {0xD7,0x0A,0xF4};
//  byte info7[] = {0xF8,0xAE,0x6C};//{0xF8,0xAE,0x6C};
//
//  showWeatherForcast(0,info0);
//  showWeatherForcast(1,info1);
//  showWeatherForcast(2,info2);
//  showWeatherForcast(3,info3);
//  showWeatherForcast(4,info4);
//  showWeatherForcast(5,info5);
//  showWeatherForcast(6,info6);
//  showWeatherForcast(7,info7);
}


void loop(void)
{
  static unsigned long mPrevTimeMs = 0;
  
  int lDcfState = mDcf.Run();

  if (lDcfState!=DCF_STATE_SAMPLING)  // don't interrupt realtime task
  {
    unsigned long nowMs = millis();
    unsigned long diff = nowMs-mPrevTimeMs;
    if( diff>50)
    {
      handleKeys();
      handleMenu(diff);
      mPrevTimeMs = nowMs;
    }
  }
  
  if (lDcfState==DCF_STATE_NEWSECOND)  // new second passed
  {
    mTft.fillRect(0,YPOS_DATE,DATE_SIZE*6,HEIGHT_DATE, BLACK);
    if(mDcfStatus)
    {
      mTft.setCursor(0,YPOS_DATE);
      mTft.print("*");
    }
    mDcfStatus = !mDcfStatus;   // toggle indicator

    DateTime now = mRtc.now();

    if (now.IsValid() && mPrevMinute != now.minute())
    {
      TestAlarms(now.hour(), now.minute(), now.dayOfWeek());

      // add a new value to the graphics bar every 20 minutes
      bool lAddNewSample = false;
      byte lNowMinute = now.minute();
      if ((lNowMinute%20)==0 && mPrevQuarter!=lNowMinute)
      {
        lAddNewSample = true;
        mPrevQuarter = lNowMinute;
      }
      handleTempPressure(lAddNewSample);
    }
  
    // does the time and/or date on the screen need an update?
    if (mForceUpdate || mPrevMinute != now.minute())
    {
      handleTime(now,mForceUpdate);
      mForceUpdate = false;
      mPrevMinute = now.minute();
    }
    HandleAlarmSound();
  }
  if (lDcfState==DCF_STATE_NEWMINUTE)
  {
    // new time available
    if (mDcf.TimeIsValid())
    {
      DateTime lDcfTime = mDcf.GetTime();
      DateTime now = mRtc.now();
  
      if (now.IsValid() && lDcfTime!=now)
      {
//        Serial.print(F("My time "));
//        Serial.print(now.GetTimeStr());
//        Serial.print(F(" differs from DCF "));
//        Serial.println(lDcfTime.GetTimeStr());
        mRtc.adjust(lDcfTime);
        handleTime(lDcfTime,true);
        mPrevMinute = lDcfTime.minute();
      }
    }
  }
  else if (lDcfState==DCF_STATE_NEWWEATHER)
  {
    // new weather info available
    byte aInfo[WEATHER_INFO_SIZE];
    if (mDcf.GetWeatherInfo(aInfo))
    {
      byte area = mDcf.GetWeatherArea();
      byte section = mDcf.GetWeatherSection();
      if (area==42)    // Amsterdam
      {
        showWeatherForcast(section,aInfo);
      }
    }
  }
  
  #ifdef DEBUG_INPUT
  if(Serial.available())
  {
    handleDebugInput();
  }
#endif

  if(mPirState != digitalRead(PIN_PIR))
  {
    mPirState = digitalRead(PIN_PIR);  
  }

  if( mAlarmActive && mPirState)
  {
    if(mAlarmTime>1)    // sound the alarm at least one second
    {
      StopAlarmSound();
    }
  }

}

void StoreAlarm(uint8_t aIndex, sAlarm aAlarm)
{
  if( aIndex<NR_OF_ALARMS)
  {
    mAlarms[aIndex] = aAlarm;
    uint16_t offset = aIndex * sizeof(sAlarm);
    EEPROM.update(offset++, aAlarm.hours);
    EEPROM.update(offset++, aAlarm.minutes);
    EEPROM.update(offset++, aAlarm.day);
    EEPROM.update(offset, aAlarm.active?0x5A:0xA5);
  }
}

void ReadAlarms()
{
  for(uint8_t i=0; i<NR_OF_ALARMS; i++)
  {
    ReadAlarm(i,mAlarms[i]);
  }
}

void ReadAlarm(byte aIndex, sAlarm &aAlarm)
{
  uint16_t offset = aIndex * 4;
  uint8_t lHours= EEPROM.read(offset++);
  uint8_t lMinutes = EEPROM.read(offset++);
  uint8_t lDay = EEPROM.read(offset++);
  uint8_t lActive = EEPROM.read(offset++);
  
  if ((lActive==0x5A || lActive==0xA5) &&
      (lHours<24 && lMinutes<60 && lDay<9))
  {
    aAlarm.hours= lHours;
    aAlarm.minutes = lMinutes;
    aAlarm.day = lDay;
    aAlarm.active = (lActive==0x5A);
  }
  else
  {
    aAlarm.hours= 0;
    aAlarm.minutes = 0;
    aAlarm.day = 0;
    aAlarm.active = false;
  }
}

void TestAlarms(uint8_t aHour, uint8_t aMinute, uint8_t aDay)
{
  bool lNewAlarmActive = false;
  for(uint8_t i=0; i<NR_OF_ALARMS; i++)
  {
    if (TestAlarm(i,aHour,aMinute,aDay))
    {
      lNewAlarmActive = true;
    }
  }    
  
  if(lNewAlarmActive && !mAlarmActive)
  {
    StartAlarmSound();
  }
  else if(!lNewAlarmActive && mAlarmActive)
  {
    StopAlarmSound();
  }
}

void StartAlarmSound()
{
  mAlarmActive = true;
  mForceUpdate = true;
  mMp3.SetVolume(ALARM_START_VOLUME);
  mMp3.Start();
}

void StopAlarmSound()
{
  mAlarmActive = false;
  mForceUpdate = true;
  mMp3.Stop();  
}

bool TestAlarm(uint8_t aIndex, uint8_t aHour, uint8_t aMinute, uint8_t aDay)
{
  if( mAlarms[aIndex].active &&
      (mAlarms[aIndex].hours == aHour) &&
      (mAlarms[aIndex].minutes == aMinute))
  {
     if((mAlarms[aIndex].day==8) ||  // every day
        (mAlarms[aIndex].day==7 && aDay!=0 && aDay!=1) ||   // weekdays
        (mAlarms[aIndex].day==aDay))
        {
          return true;
        }
  }
  return false;
}

void displayAlarms()
{
  for(uint8_t i=0; i<NR_OF_ALARMS; i++)
  {
    displayAlarm(i,mAlarms[i]);
  }
}

void displayAlarm(byte aIndex, sAlarm aAlarm)
{
  char strTime[STRLEN_ALARM+1];

  uint16_t yPos = YPOS_ALARM + aIndex*((320-YPOS_ALARM)/NR_OF_ALARMS);
  mTft.fillRect(XPOS_ALARM,yPos,WIDTH_ALARM,HEIGHT_ALARM, BLACK);
  //mTft.setTextSize(2);
  mTft.setTextColor(aAlarm.active?LIGHTRED:GRAY);
  
  if(aIndex==mMenuSelectedAlarm)   // menu is active
  {
    switch(mMenuState)
    {
      case MenuSelectAlarm:
        mTft.fillRect(XPOS_ALARM,yPos,WIDTH_ALARM,HEIGHT_ALARM, BLUE);  
        break;
      case MenuChangeAlarmDay:
        mTft.fillRect(XPOS_ALARM+0,yPos,2*6*ALARM_SIZE,HEIGHT_ALARM, WHITE);  
        break;
      case MenuChangeAlarmHour:
        mTft.fillRect(XPOS_ALARM+3*6*ALARM_SIZE,yPos,2*6*ALARM_SIZE,HEIGHT_ALARM, WHITE);  
        break;
      case MenuChangeAlarmMinute:
        mTft.fillRect(XPOS_ALARM+6*6*ALARM_SIZE,yPos,2*6*ALARM_SIZE,HEIGHT_ALARM, WHITE);  
        break;
      case MenuChangeAlarmActive:
        mTft.fillRect(XPOS_ALARM,yPos,WIDTH_ALARM,HEIGHT_ALARM, WHITE);  
        break;
    }
  }
  mTft.setCursor(XPOS_ALARM,yPos);
  sprintf(strTime,"%2s %02d:%02d", alarmdays[aAlarm.day], aAlarm.hours,aAlarm.minutes);
  mTft.print(strTime);
}

bool getTempAndPressure(double &lTemp,double &lPress)
{
  if( mBmp180.StartTemperature()!=0 )
  {
    delay(5);
    if( mBmp180.GetTemperature(lTemp)==1 )
    {
      int d = mBmp180.StartPressure(1);    // start for next sample
      if( d>0 )
      {
        delay(d);
        mBmp180.GetPressure(lPress,lTemp);
      }
    }
  }
  return true;
}

void registerPressure(byte lPressure)
{
  if(mHistoryIndex>=NR_OF_PRESSURES)
  {
    // history is 'full', clear oldest value and shift everything else
    // could be faster using memcpy 
    for(int i=0; i<(NR_OF_PRESSURES-1); i++)
    {
      mHistory[i] = mHistory[i+1];
    }
    mHistoryIndex = NR_OF_PRESSURES-1;
  }
  mHistory[mHistoryIndex] = lPressure;
  mHistoryIndex++;
}

void displayPressureGraph()
{
  mTft.fillRect(XPOS_PRES_GRAPH,YPOS_PRES_GRAPH,WIDTH_PRES_GRAPH,HEIGHT_PRES_GRAPH, BLACK);
  for(int i=0; i<NR_OF_PRESSURES; i++)
  {
    mTft.drawPixel(XPOS_PRES_GRAPH+i, YPOS_PRES_GRAPH+HEIGHT_PRES_GRAPH-mHistory[i], GRAPHCOLOR);
  }
}

/*
 * show information of weatherforcast, 4 days with each:
 * 
 *   icon-day icon-night
 *   temp max temp-min
 *   wind     rain
 */
void makeWeatherForcastGrid(int aFirstDay)
{
  mTft.fillRect(XPOS_WEATHER, YPOS_WEATHER-20, 8*WIDTH_WEATHER, 20, BLACK);
  mTft.setTextColor(WHITE);

  for( int i=0; i<4; i++)
  {
    int xPos = XPOS_WEATHER+i*2*WIDTH_WEATHER+i+1;
    mTft.setCursor(xPos+30,YPOS_WEATHER-20);   
    mTft.print(alarmdays[(aFirstDay+i)%7]);
    mTft.drawLine(xPos, YPOS_WEATHER, xPos, YPOS_WEATHER+HEIGHT_WEATHER, WHITE);    
  }
}

void showWeatherForcast(byte aSection, byte aInfo[])
{
  #define LINE_HEIGHT 20

  char str[20];
  int xPos;
  int yPos;
  mTft.setTextColor(WHITE);

  if (aSection==7)    // Only wind data is valid 
  {
      xPos = XPOS_WEATHER+6*WIDTH_WEATHER+3+WIDTH_WEATHER-12;
      yPos = YPOS_WEATHER+52+LINE_HEIGHT;
      mTft.fillRect(xPos, yPos, WIDTH_WEATHER+12, LINE_HEIGHT, BLACK);      
      strcpy(str,Weather::GetWindDirection(aInfo));
      strcat(str,Weather::GetWindForce(aInfo));
    // for code efficiency moved outside if
//      mTft.setCursor(xPos,yPos);   
//      mTft.print(str);
  }
  else
  {
    bool lHighPart = ((aSection%2)==0);   // that's the high value part of the day
    
    yPos = YPOS_WEATHER;
    // 2 sections a day (high and low), each WIDTH_WEATHER
    // added seperation line bewteen each day
    int day = aSection/2;
    xPos = XPOS_WEATHER + day*(1+2*WIDTH_WEATHER) + 1;
    
    // weather at daytime
    showWeatherIcon(xPos, YPOS_WEATHER, Weather::GetWeatherDay(aInfo));
  
    // weather at nighttime
    showWeatherIcon(xPos+WIDTH_WEATHER, YPOS_WEATHER, Weather::GetWeatherNight(aInfo));
  
    yPos += 52;
  
    sprintf(str,"%dC",Weather::GetTemp(aInfo));
    mTft.fillRect(lHighPart?xPos:xPos+WIDTH_WEATHER, yPos, WIDTH_WEATHER, LINE_HEIGHT, BLACK);
    mTft.setCursor(lHighPart?xPos:xPos+WIDTH_WEATHER,yPos);   
    mTft.print(str);
  
    yPos += LINE_HEIGHT;
    if (lHighPart)   // that's the high value part of the day
    {
      mTft.fillRect(xPos, yPos, WIDTH_WEATHER, LINE_HEIGHT, BLACK);      
      sprintf(str,"%d%%",Weather::GetRain(aInfo));
    }
    else  // that's the low value part of the day
    {
      xPos+=WIDTH_WEATHER-12;
      mTft.fillRect(xPos, yPos, WIDTH_WEATHER+12, LINE_HEIGHT, BLACK);      
      strcpy(str,Weather::GetWindDirection(aInfo));
      strcat(str,Weather::GetWindForce(aInfo));
    }
    // for code efficiency moved outside if
//    mTft.setCursor(xPos,yPos);   
//    mTft.print(str);
  }
  mTft.setCursor(xPos,yPos);   
  mTft.print(str);
}
/* for memory eficiency, icons are a combination of two images:
 *  one top for the clouds (26 pixels high
 *  one a the bottom for the rain/snow (20 pixels high)
 */

const byte* WeatherImages[][2] = {
    //{bmpWeer_1,0},
    {bmpWeer_2,0},
    {bmpWeer_3,0},
    {bmpWeer_4,0},
    {bmpWeer_3,bmpWeer_5},
    {bmpWeer_4,bmpWeer_6},
    {bmpWeer_4,bmpWeer_7},
    {0,bmpWeer_8},
    {bmpWeer_3,bmpWeer_9},
    {bmpWeer_3,bmpWeer_6},
    {bmpWeer_4,bmpWeer_11},
    {bmpWeer_3,bmpWeer_7},
    {bmpWeer_4,bmpWeer_13},
    {bmpWeer_1,bmpWeer_8},
    {bmpWeer_4,bmpWeer_9}
  };


void showWeatherIcon(int aX, int aY, byte aIcon)
{
  mTft.fillRect(aX, aY, 48, 48, BLACK);

  if(aIcon==1)
  {
      mTft.drawBitmap(aX, aY,
              bmpWeer_1, 
              48,48,
              YELLOW,BLACK);

  }
  else if(aIcon>0 && aIcon<=15)
  {
    mTft.drawBitmap(aX, aY,
                WeatherImages[aIcon-2][0], 
                48,26,
                LIGHTGRAY,BLACK);
    mTft.drawBitmap(aX, aY+26,
                WeatherImages[aIcon-2][1], 
                48,20,
                LIGHTBLUE,BLACK);
  }
}

void showSunMoon(DateTime aDateTime)
{
  char str[16];

  // erase all old information
  mTft.fillRect(XPOS_SUNMOON,YPOS_SUNMOON,WIDTH_SUNMOON,HEIGHT_SUNMOON, BLACK);

  byte lMoonPhase = GetMoonPhase(aDateTime);

  // first draw 'full-moon' 
  mTft.fillCircle(XPOS_MOON,YPOS_SUNMOON+MOON_RADIUS+10, MOON_RADIUS, LIGHTGRAY);   // x,y is center of circle
  // then draw shadow of the earth over it
  int16_t lEarthPos = - (2*lMoonPhase*2*MOON_RADIUS/29);
  if(lMoonPhase>14)
  {
    lEarthPos += 4*MOON_RADIUS;
  }
  mTft.fillCircle(XPOS_MOON+lEarthPos,YPOS_SUNMOON+MOON_RADIUS+10, MOON_RADIUS, BLACK);   // x,y is center of circle

  mTft.setTextColor(WHITE);

  // Sun rise and set:
  // use top halve of sunny weather icon
  mTft.drawBitmap(XPOS_SUN+8, YPOS_SUN,
                  bmpWeer_1, 
                  48,24,
                  YELLOW,BLACK);
            
  mTft.setCursor(XPOS_SUN,0+36);  
  mTft.print(GetSunRise(aDateTime).GetTimeStr());
  mTft.setCursor(XPOS_SUN,36+20);  
  mTft.print(GetSunSet(aDateTime).GetTimeStr());
}

void handleTempPressure(bool aAddNewSample)
{
  double lTemp=0;
  double lPress=0;
  getTempAndPressure(lTemp,lPress);

  char str[10];
  char strValue[8];

  mTft.fillRect(XPOS_PRES,YPOS_PRES,WIDTH_TEMPPRES,HEIGHT_TEMPPRES, BLACK);
  mTft.setTextColor(WHITE);  
  
  mTft.setCursor(XPOS_PRES,YPOS_PRES);
  sprintf(str,"%4d mBar",(int)lPress);
  mTft.print(str);

//  dtostrf(lTemp,4,1,strValue);
//  sprintf(str,"%s C",strValue);
  sprintf(str,"%d.%1d C",(int)lTemp,(int)(lTemp*10)%10);
  //Serial.println(str);
  mTft.setCursor(XPOS_TEMP,YPOS_TEMP);
  mTft.print(str);

  // update pressure graphics 
  if(aAddNewSample)
  {
    //Serial.println("adding another pressure sample");
    byte   lPressure = 0;
    // scale the measured value to the available display area (in this case not needed)
    lPress -= MIN_PRESSURE;
    //lPress *= HEIGHT_PRES_GRAPH;
    //lPress /= (MAX_PRESSURE-MIN_PRESSURE);
    lPressure = (byte)lPress;
    registerPressure(lPressure);
    displayPressureGraph();
  }
}

void handleTime(DateTime aDateTime,bool aForcedUpdate)
{
  char str[12];
  char strDate[STRLEN_DATE+1];

  //sprintf(str,"%02d:%02d", aDateTime.hour(), aDateTime.minute());
  mTft.fillRect(XPOS_TIME,YPOS_TIME,WIDTH_TIME,HEIGHT_TIME, BLACK);
  mTft.setCursor(XPOS_TIME,YPOS_TIME);
  mTft.setTextColor(mAlarmActive?LIGHTRED:WHITE);  
  mTft.setTextSize(TIME_SIZE);
  //mTft.print(str);
  mTft.print(aDateTime.GetTimeStr());
  //Serial.print(str);
  mTft.setTextSize(2);  // terug naar default

  if ((aDateTime.dayOfWeek() != mPrevDate.dayOfWeek()) ||
      (aDateTime.day() != mPrevDate.day()) ||
      (aDateTime.month() != mPrevDate.month()) || 
      (aDateTime.year() != mPrevDate.year()) ||
      aForcedUpdate )
  {
    //Serial.print(" ");
//    sprintf(strDate,"%s %s", (char*)pgm_read_word(&(days_table[aDateTime.dayOfWeek()])), aDateTime.GetDateStr());
    strcpy_P(strDate, (char*)pgm_read_word(&(days_table[aDateTime.dayOfWeek()])));
    strcat(strDate,aDateTime.GetDateStr());
    //Serial.print(strDate);
    mTft.fillRect(XPOS_DATE,YPOS_DATE,WIDTH_DATE,HEIGHT_DATE, BLACK);
    mTft.setCursor(XPOS_DATE,YPOS_DATE);
    mTft.setTextColor(WHITE);  
    //mTft.setTextSize(DATE_SIZE);
    mTft.print(strDate);
    mPrevDate = aDateTime;

    showSunMoon(aDateTime);

    makeWeatherForcastGrid(aDateTime.dayOfWeek());

  }
  //Serial.println();
}

void handleKeys()
{
  static bool keyIdle=true;
  unsigned int lAdcValues[2];
  mAdc.ReadADC(lAdcValues,2);

  if(keyIdle)
  {
    mKeyUp   = (lAdcValues[1]<1000);
    mKeyDown = (lAdcValues[1]>25000);
    mKeyLeft = (lAdcValues[0]<1000);
    mKeyRight = (lAdcValues[0]>25000);

    if(mKeyUp || mKeyDown || mKeyLeft || mKeyRight)
      keyIdle = false;

//      if(mKeyUp) Serial.println(("Up"));
//      if(mKeyDown) Serial.println(("Down"));
//      if(mKeyLeft) Serial.println(("Left"));
//      if(mKeyRight) Serial.println(("Right"));
  }
  else   // wait until key has been released
  {
    if(lAdcValues[1]>1000 && lAdcValues[1]<25000 &&
       lAdcValues[0]>1000 && lAdcValues[0]<25000)
       {
         keyIdle = true;
       }
  }
}

void handleMenu(unsigned int aElapsedTimeMs)
{
  mMenuTimeMs += aElapsedTimeMs;
  handleMenuDo(mMenuState);
  sMenuStates lNextMenuState = handleMenuTransfer(mMenuState);
  if(lNextMenuState != mMenuState)
  {
    handleMenuExit(mMenuState);
    mMenuState = lNextMenuState;
    handleMenuEntry(mMenuState);
  }
  if (mKeyUp || mKeyDown || mKeyLeft || mKeyRight)
  {
    mMenuTimeMs = 0;
    mKeyUp = false;
    mKeyDown = false;
    mKeyLeft = false;
    mKeyRight = false;
  }
}

void handleMenuDo(sMenuStates aMenuState)
{  
  switch( aMenuState )
  {
    case MenuIdle:
      break;
    case MenuSelectAlarm:
      if(mKeyUp)
      {
        if(mMenuSelectedAlarm==0)
          mMenuSelectedAlarm = NR_OF_ALARMS;
        mMenuSelectedAlarm--;
        displayAlarms();
      }
      if(mKeyDown)
      {
        mMenuSelectedAlarm++;
        if(mMenuSelectedAlarm>=NR_OF_ALARMS)
          mMenuSelectedAlarm = 0;
        displayAlarms();
      }
      break;
    case MenuChangeAlarmDay:
      if(mKeyUp)
      {
        mAlarms[mMenuSelectedAlarm].day++;
        if(mAlarms[mMenuSelectedAlarm].day>8)
          mAlarms[mMenuSelectedAlarm].day=0;
        displayAlarm(mMenuSelectedAlarm,mAlarms[mMenuSelectedAlarm]);
      }
      if(mKeyDown)
      {
        if(mAlarms[mMenuSelectedAlarm].day==0)
          mAlarms[mMenuSelectedAlarm].day=9;
        mAlarms[mMenuSelectedAlarm].day--;
        displayAlarm(mMenuSelectedAlarm,mAlarms[mMenuSelectedAlarm]);
      }
      break;
    case MenuChangeAlarmHour:
      if(mKeyUp)
      {
        mAlarms[mMenuSelectedAlarm].hours++;
        if(mAlarms[mMenuSelectedAlarm].hours>23)
          mAlarms[mMenuSelectedAlarm].hours = 0;
        displayAlarm(mMenuSelectedAlarm,mAlarms[mMenuSelectedAlarm]);
      }
      if(mKeyDown)
      {
        if(mAlarms[mMenuSelectedAlarm].hours==0)
          mAlarms[mMenuSelectedAlarm].hours=24;
        mAlarms[mMenuSelectedAlarm].hours--;
        displayAlarm(mMenuSelectedAlarm,mAlarms[mMenuSelectedAlarm]);
      }
      break;
    case MenuChangeAlarmMinute:
      if(mKeyUp)
      {
        mAlarms[mMenuSelectedAlarm].minutes+=5;
        if(mAlarms[mMenuSelectedAlarm].minutes>59)
          mAlarms[mMenuSelectedAlarm].minutes = 0;
        displayAlarm(mMenuSelectedAlarm,mAlarms[mMenuSelectedAlarm]);
      }
      if(mKeyDown)
      {
        if(mAlarms[mMenuSelectedAlarm].minutes<5)
          mAlarms[mMenuSelectedAlarm].minutes = 60;
        mAlarms[mMenuSelectedAlarm].minutes-=5;
        displayAlarm(mMenuSelectedAlarm,mAlarms[mMenuSelectedAlarm]);
      }      
      break;
    case MenuChangeAlarmActive:
      if(mKeyDown || mKeyUp)
      {
        mAlarms[mMenuSelectedAlarm].active = !mAlarms[mMenuSelectedAlarm].active;
        displayAlarm(mMenuSelectedAlarm,mAlarms[mMenuSelectedAlarm]);
      }
      break;
    default: 
      break;
  }
}

sMenuStates handleMenuTransfer(sMenuStates aMenuState)
{
  sMenuStates lNewMenuState = aMenuState;
  switch( aMenuState )
  {
    case MenuIdle:
      if(mKeyRight)
        lNewMenuState = MenuSelectAlarm;
      break;
    case MenuSelectAlarm:
      if(mMenuTimeMs>15000)
        lNewMenuState = MenuIdle;
      else if(mKeyRight)
        lNewMenuState = MenuChangeAlarmDay;
      else if(mKeyLeft)
        lNewMenuState = MenuIdle;
      break;
    case MenuChangeAlarmDay:
      if(mMenuTimeMs>15000)
        lNewMenuState = MenuRestoreAlarm;
      else if(mKeyRight)
        lNewMenuState = MenuChangeAlarmHour;
      else if(mKeyLeft)
        lNewMenuState = MenuStoreAlarm;
      break;
    case MenuChangeAlarmHour:
      if(mMenuTimeMs>15000)
        lNewMenuState = MenuRestoreAlarm;
      else if(mKeyRight)
        lNewMenuState = MenuChangeAlarmMinute;
      else if(mKeyLeft)
        lNewMenuState = MenuChangeAlarmDay;
      break;
    case MenuChangeAlarmMinute:
      if(mMenuTimeMs>15000)
        lNewMenuState = MenuRestoreAlarm;
      else if(mKeyRight)
        lNewMenuState = MenuChangeAlarmActive;
      else if(mKeyLeft)
        lNewMenuState = MenuChangeAlarmHour;
      break;
    case MenuChangeAlarmActive:
      if(mMenuTimeMs>15000)
        lNewMenuState = MenuRestoreAlarm;
      else if(mKeyRight)
        lNewMenuState = MenuChangeAlarmDay;
      else if(mKeyLeft)
        lNewMenuState = MenuChangeAlarmMinute;
      break;
    case MenuStoreAlarm:
      lNewMenuState = MenuSelectAlarm;
      break;
    case MenuRestoreAlarm:
      lNewMenuState = MenuIdle;
      break;
    default: 
      break;
  }

//  if(lNewMenuState!=aMenuState)
//  {
//    Serial.print(F("state changed to "));
//    Serial.println(lNewMenuState);
//  }

  return lNewMenuState;
}

void handleMenuEntry(sMenuStates aMenuState)
{
  
  switch( aMenuState )
  {
    case MenuIdle:
      mMenuSelectedAlarm = -1;
      displayAlarms();
      break;
    case MenuSelectAlarm:
      displayAlarms();
      break;
    case MenuChangeAlarmDay:
      displayAlarm(mMenuSelectedAlarm,mAlarms[mMenuSelectedAlarm]);
      break;
    case MenuChangeAlarmHour:
      displayAlarm(mMenuSelectedAlarm,mAlarms[mMenuSelectedAlarm]);
      break;
    case MenuChangeAlarmMinute:
      displayAlarm(mMenuSelectedAlarm,mAlarms[mMenuSelectedAlarm]);
      break;
    case MenuChangeAlarmActive:
      displayAlarm(mMenuSelectedAlarm,mAlarms[mMenuSelectedAlarm]);
      break;
    case MenuStoreAlarm:
      StoreAlarm(mMenuSelectedAlarm, mAlarms[mMenuSelectedAlarm]);
      break;
    case MenuRestoreAlarm:
      ReadAlarm(mMenuSelectedAlarm, mAlarms[mMenuSelectedAlarm]);
    default: 
      break;
  }
}

void handleMenuExit(sMenuStates aMenuState)
{
  switch( aMenuState )
  {
    case MenuIdle:
      mMenuSelectedAlarm = 0;
      break;
    default: 
      break;
  }
}

void HandleAlarmSound()
{
  if (!mAlarmActive)
  {
    mAlarmTime = 0;
  }
  else
  {
    mAlarmTime++;

    mMp3.Run();
    if(!mMp3.IsActive())
    {
      mMp3.Next();
    }
    if ((mAlarmTime%3)==2)
    {
      mMp3.IncreaseVolume(1);
    }
  }
}

