#include "ProjSettings.h"
#include "Adafruit_GFX.h"    // Core graphics library
#include "TftSpfd5408.h" // Hardware-specific library
#include <Wire.h>
#include <EEPROM.h>
#include "DateTime.h"
#include "RTClib.h"
#include "Dcf77.h"
#include "Weather.h"
#include "Bmp180.h"
#include "Ads1015.h"
#include "MP3.h"
#include "Alarm.h"
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

#define PIN_MP3RX  11
#define PIN_MP3TX  10

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

//EEPROM layout:
// start with alarms
#define ALARMS_OFFSET         0
#define NR_OF_ALARMS          6
#define WEATHER_INFO_OFFSET   100

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

#define TFT_WIDTH  320
#define TFT_HEIGHT 240

#define TFT_CHAR_WIDTH  6
#define TFT_CHAR_HEIGHT 8

#define WEATHERICON_WIDTH      48
#define WEATHERICON_HEIGHT     48

/* four seperate 'windows':
 * 1. time/date
 * 2. alarms
 * 3. weather
 * 4 barometric chart
 */

typedef enum
{
  DISPLAY_TIME = 0,
  DISPLAY_WEATHER,
  DISPLAY_ALARMS,
  DISPLAY_PRESSURE
} DISPLAYS;
/*
 * 1. temp/press sun/moon
 *      time
 *   date-string
 *   active alarms
 */

#define XPOS_SUNMOON     (TFT_WIDTH-WIDTH_SUNMOON)
#define YPOS_SUNMOON     0
#define XPOS_MOON        (XPOS_SUNMOON+MOON_RADIUS)
#define XPOS_SUN         (XPOS_MOON+MOON_RADIUS*2+10)
#define YPOS_SUN         0
#define WIDTH_SUNMOON    (MOON_RADIUS*3+10+TFT_CHAR_WIDTH*DATE_SIZE*5)
#define HEIGHT_SUNMOON   (WEATHERICON_HEIGHT+2*TFT_CHAR_HEIGHT*DATE_SIZE)
#define MOON_RADIUS 16

#define TIME_SIZE   10
#define STRLEN_TIME 5
#define WIDTH_TIME  (TFT_CHAR_WIDTH*TIME_SIZE*STRLEN_TIME)
#define HEIGHT_TIME (TFT_CHAR_HEIGHT*TIME_SIZE)
#define XPOS_TIME   ((TFT_WIDTH-WIDTH_TIME)/2)
#define YPOS_TIME   (YPOS_SUNMOON+HEIGHT_SUNMOON)

#define DATE_SIZE   2
#define STRLEN_DATE (10+12)
#define WIDTH_DATE  (TFT_CHAR_WIDTH*DATE_SIZE*STRLEN_DATE)
#define HEIGHT_DATE (TFT_CHAR_HEIGHT*DATE_SIZE)
#define XPOS_DATE   ((TFT_WIDTH-WIDTH_DATE)/2)
#define YPOS_DATE   (YPOS_TIME+HEIGHT_TIME) // 300
#define XPOS_ACTIVE_ALARMS  XPOS_DATE
#define YPOS_ACTIVE_ALARMS  (YPOS_DATE+HEIGHT_DATE+14)

#define TEMPPRES_SIZE   2
#define XPOS_PRES       10
#define YPOS_PRES       10
#define XPOS_TEMP       XPOS_PRES
#define YPOS_TEMP       (YPOS_PRES+TFT_CHAR_HEIGHT*TEMPPRES_SIZE+4)
#define WIDTH_TEMPPRES  (TFT_CHAR_WIDTH*TEMPPRES_SIZE*8)        //1040 mBar
#define HEIGHT_TEMPPRES (YPOS_TEMP+TFT_CHAR_HEIGHT*TEMPPRES_SIZE+4)

/*
 * 2.  Alarms
 */
#define ALARM_SIZE   2
#define STRLEN_ALARM 8
#define WIDTH_ALARM  (TFT_CHAR_WIDTH*ALARM_SIZE*STRLEN_ALARM)
#define HEIGHT_ALARM (TFT_CHAR_HEIGHT*ALARM_SIZE)
#define XPOS_ALARM   ((TFT_WIDTH-WIDTH_ALARM)/2)
#define YPOS_ALARM   0

/*
 * 3.  Weather forcast
 *   day1   day2
 *   day3   day4
 */


#define XPOS_WEATHER     0
#define YPOS_WEATHER     0
#define WIDTH_WEATHER    (TFT_WIDTH/4)
#define HEIGHT_WEATHER   (TFT_HEIGHT/2)
#define NR_OF_WEATHER_ICONS 15
/*
 * 1. temp/press sun/moon
 * 4.  barometric chart
 */
#define MIN_PRESSURE  960
#define MAX_PRESSURE  1040
#define MIN_PRESSURE_STR  "960"
#define MAX_PRESSURE_STR  "1040"
//#define NR_OF_PRESSURES (3*24*3)     // last 3 days, 3 values/hour
#define NR_OF_PRESSURES (3*24*2)     // last 3 days, 2 values/hour
#define XPOS_PRES_GRAPH ((TFT_WIDTH-WIDTH_PRES_GRAPH)/2)
#define YPOS_PRES_GRAPH ((TFT_HEIGHT-HEIGHT_PRES_GRAPH)/2)
#define HEIGHT_PRES_GRAPH (MAX_PRESSURE-MIN_PRESSURE)
#define WIDTH_PRES_GRAPH NR_OF_PRESSURES
//#define WIDTH_PRES_GRAPH 220
#define GRAPHCOLOR   WHITE

#define XPOS_LDR  10
#define YPOS_LDR  (TFT_HEIGHT-TFT_CHAR_HEIGHT*2)
#define WIDTH_LDR (6*TFT_CHAR_WIDTH)

#define ALARM_START_VOLUME 10

typedef enum
{
  MenuSelectAlarm,
  MenuChangeAlarmDay,
  MenuChangeAlarmHour,
  MenuChangeAlarmMinute
} sMenuStates;

char mPrevMinute=0;
DateTime mNow;
DateTime mPrevDate;
const char maandag[] PROGMEM = "maandag";
const char dinsdag[] PROGMEM = "dinsdag";
const char woensdag[] PROGMEM = "woensdag";
const char donderdag[] PROGMEM = "donderdag";
const char vrijdag[] PROGMEM = "vrijdag";
const char zaterdag[] PROGMEM = "zaterdag";
const char zondag[] PROGMEM = "zondag";
const char* const days_table[] PROGMEM = {maandag,dinsdag,woensdag,donderdag,vrijdag,zaterdag,zondag};

Alarm* mAlarms[NR_OF_ALARMS];
bool mAlarmActive = false;
byte mAlarmTime = 0;
bool mPirState = false;
bool mKeyUp = false;
bool mKeyDown = false;
bool mKeyLeft = false;
bool mKeyRight = false;

sMenuStates    mMenuState = MenuSelectAlarm;
byte           mMenuSelectedAlarm = -1;

uint8_t mActiveDisplay = 0;
#ifdef BAROMETER
byte mHistory[NR_OF_PRESSURES];
int  mHistoryIndex = 0;
#endif
byte mPrevSampleTime = 0;
bool mDcfStatus = false;

double mCurrentTemp=0;
double mCurrentPress=0;

unsigned int mLDR = 0;

TftSpfd5408 mTft(LCD_CS, LCD_CD, LCD_WR, LCD_RD);
#ifdef USE_RTC
RTC_DS1307 mRtc;
#endif
Dcf77      mDcf;
Bmp180     mBmp180;
Ads1015    mAdc;
MP3        mMp3;

#define BUILDTM_YEAR (\
    __DATE__[7] == '?' ? 1900 \
    : (((__DATE__[7] - '0') * 1000 ) \
    + (__DATE__[8] - '0') * 100 \
    + (__DATE__[9] - '0') * 10 \
    + __DATE__[10] - '0'))

#define BUILDTM_MONTH (\
    __DATE__ [2] == '?' ? 1 \
    : __DATE__ [2] == 'n' ? (__DATE__ [1] == 'a' ? 1 : 6) \
    : __DATE__ [2] == 'b' ? 2 \
    : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 3 : 4) \
    : __DATE__ [2] == 'y' ? 5 \
    : __DATE__ [2] == 'l' ? 7 \
    : __DATE__ [2] == 'g' ? 8 \
    : __DATE__ [2] == 'p' ? 9 \
    : __DATE__ [2] == 't' ? 10 \
    : __DATE__ [2] == 'v' ? 11 \
    : 12)

#define BUILDTM_DAY (\
    __DATE__[4] == '?' ? 1 \
    : ((__DATE__[4] == ' ' ? 0 : \
    ((__DATE__[4] - '0') * 10)) + __DATE__[5] - '0'))

#define BUILDTM_HOUR (\
    __TIME__[0] == '?' ? 1 \
    : ((__TIME__[0] == ' ' ? 0 : \
    ((__TIME__[0] - '0') * 10)) + __TIME__[1] - '0'))

#define BUILDTM_MIN (\
    __TIME__[3] == '?' ? 1 \
    : ((__TIME__[3] == ' ' ? 0 : \
    ((__TIME__[3] - '0') * 10)) + __TIME__[4] - '0'))

void setup(void)
{
#ifdef DEBUG
  Serial.begin(9600);
#endif

  pinMode(PIN_PIR,INPUT);

  Wire.begin();
  #ifdef USE_RTC
  mRtc.begin();
  #endif
  
  mTft.init();

#ifdef ENABLE_ALARMS
  for(byte i=0; i<NR_OF_ALARMS; i++)
  {
    mAlarms[i] = new Alarm(i);
  }
#endif

  mBmp180.Init();
  #ifdef BAROMETER
  for( int i=0; i<NR_OF_PRESSURES; i++)
  {
    mHistory[i++]=0;
    mHistory[i]=HEIGHT_PRES_GRAPH-1;
  }
  mHistoryIndex = 0;
  #endif
  #ifdef USE_RTC
  mNow = mRtc.now();
  #endif
  
  mDcf.Init(PIN_DCF);
  mMp3.Init();

#ifdef MAKE_TEST_DATA
  byte lInfo[3];
  // make test data, first day is tuesday
  lInfo[0]=0b11010110;   lInfo[1]=0b00000110;   lInfo[2]=0b10010110;
  setWeatherForcast(0,lInfo,2);
  lInfo[0]=0b01001000;   lInfo[1]=0b01100100;   lInfo[2]=0b00010110;
  setWeatherForcast(1,lInfo,2);
  lInfo[0]=0b11000110;   lInfo[1]=0b00000110;   lInfo[2]=0b11001110;
  setWeatherForcast(2,lInfo,3);
  lInfo[0]=0b10001011;   lInfo[1]=0b00000100;   lInfo[2]=0b01100110;
  setWeatherForcast(3,lInfo,3);
  lInfo[0]=0b01000010;   lInfo[1]=0b00001000;   lInfo[2]=0b00110110;
  setWeatherForcast(4,lInfo,4);
  lInfo[0]=0b01011100;   lInfo[1]=0b10101100;   lInfo[2]=0b10000110;
  setWeatherForcast(5,lInfo,4);
  lInfo[0]=0b10001000;   lInfo[1]=0b00000000;   lInfo[2]=0b11001110;
  setWeatherForcast(6,lInfo,5);
  lInfo[0]=0b01100100;   lInfo[1]=0b10100100;   lInfo[2]=0b11010110;
  setWeatherForcast(7,lInfo,5);
#endif

#ifdef ADJUST_TIME
  #ifdef DEBUG
  Serial.print("Adjust RTC to ");
  Serial.print(__DATE__); Serial.print(__TIME__);
  #endif
  DateTime nu(BUILDTM_YEAR,BUILDTM_MONTH,BUILDTM_DAY,BUILDTM_HOUR,BUILDTM_MIN);
  #ifdef USE_RTC
  mRtc.adjust(nu);
  #endif
#endif

  ActivateDisplay(DISPLAY_TIME);
}


void loop(void)
{
#ifdef TEST_WEATHER_ICONS
  TestWeatherIcons();
#else
  static unsigned long mPrevTimeCheckKeysMs = 0;
  static unsigned long mPrevTimeTimeMs = 0;
  
  #ifdef ENABLE_DCF
  int lDcfState = mDcf.Run();
  #endif
  
  unsigned long nowMs = millis();
  unsigned long diff = nowMs-mPrevTimeCheckKeysMs;
  if( diff>100)
  {
  #ifdef ENABLE_DCF
    if ((lDcfState!=DCF_STATE_SAMPLING) ||  // don't interrupt realtime task
        (diff>5000))                        // DCF is not available
  #endif
    {
      handleKeys();
      #ifdef HANDLE_MENU
      handleMenu(diff);
      #endif
      mPrevTimeCheckKeysMs = nowMs;

      #ifdef ENABLE_ALARMS
      HandleAlarmSound();
      #endif
      HandleBacklight();
    }
  }

  diff = nowMs-mPrevTimeTimeMs;
  if ((diff>10000) &&     // once every 10 seconds
      (lDcfState!=DCF_STATE_SAMPLING))
  {
#ifdef USE_RTC
    mNow = mRtc.now();
#else
    mNow = mDcf.GetTime();
#endif
    if (mPrevMinute != mNow.mm)
    {
      handleTempPressure();
   
      // add a new value to the graphics bar every 30 minutes
      byte lNowMinute = mNow.mm;
      if ((lNowMinute==0 || lNowMinute==30))
      {
        registerPressure((byte)mCurrentPress);
      }
  
      // does the time and/or date on the screen need an update?
      #ifdef ENABLE_ALARMS
      TestAlarms(mNow);
      #endif

      UpdateDisplay(DISPLAY_TIME);
      mPrevMinute = lNowMinute;  // mNow.minute();
    }
    mPrevTimeTimeMs = nowMs;
  }

#ifdef ENABLE_DCF
// TODO now every loop the same questionis asked, better only ask once
// although it looks like it only appears once a second
  if (lDcfState==DCF_STATE_NEWSECOND)  // new second passed
  {
    // show indicator of DCF signal
    mTft.fillRect(0,YPOS_DATE,DATE_SIZE*6,HEIGHT_DATE, BLACK);
    if(mDcfStatus)
    {
      mTft.setCursor(0,YPOS_DATE);
      mTft.print(mDcf.TimeIsValid()?"*":"+");
    }
    mDcfStatus = !mDcfStatus;   // toggle indicator
  }
  
  #ifdef UPDATE_RTC
  // TODO now every loop the same questionis asked, better only ask once
  if (lDcfState==DCF_STATE_NEWMINUTE)
  {
    // new time available
    if (mDcf.TimeIsValid())
    {
      DateTime lDcfTime = mDcf.GetTime();
      //DateTime lRtcTime = mRtc.now();

      if (lDcfTime!=mNow)
      {
        #ifdef DEBUG
        Serial.print(F("Updating RTC to "));
        Serial.print(lDcfTime.GetTimeStr());
        Serial.print(" ");
        Serial.println(lDcfTime.GetDateStr());
        #endif
        mRtc.adjust(lDcfTime);
        // no need to update tie and display, will be done next loop
        //UpdateDisplay(DISPLAY_TIME);
        //mPrevMinute = lDcfTime.minute();
      }
    }
  }
  #endif
  
  // TODO now every loop the same questionis asked, better only ask once
  if (lDcfState==DCF_STATE_NEWWEATHER)
  {
    // new weather info available
    byte area = mDcf.GetWeatherArea();
    if (area==42)    // Amsterdam
    {
      byte lSection = mDcf.GetWeatherSection();
      byte aInfo[WEATHER_INFO_SIZE];
      if (mDcf.GetWeatherInfo(aInfo))
      {
        int lDay = lSection/2 + mNow.d;
        lDay %= 7;
        setWeatherForcast(lSection,aInfo,lDay);
      }
      else
      {
        clearWeatherForcast(lSection);        // remove previous info, new is invalid
      }
      UpdateDisplay(DISPLAY_WEATHER);
    }
  }
#endif ENABLE_DCF
  
  if(mPirState != digitalRead(PIN_PIR))
  {
    mPirState = digitalRead(PIN_PIR);  
  }

#ifdef ENABLE_ALARMS
  if( mAlarmActive && mPirState)
  {
    if(mAlarmTime>1)    // sound the alarm at least one second
    {
      StopAlarmSound();
    }
  }
#endif
#endif
}

void UpdateDisplay(byte aDisplay)
{
  if(mActiveDisplay==aDisplay)
  {
    ShowDisplay();
  }
}

void ActivateDisplay(byte aDisplay)
{
  mActiveDisplay = aDisplay;
  mTft.fillScreen(BLACK);
  ShowDisplay();
}

void ShowDisplay()
{
  if(mActiveDisplay==DISPLAY_TIME)
  {
    mPrevDate.Clear();  // force update of date
    showTime(mNow);
    showTempPressure();
    showActiveAlarms();    
  }
  else if(mActiveDisplay==DISPLAY_WEATHER)
  {
    showWeatherForcast(); 
  }
  else if(mActiveDisplay==DISPLAY_ALARMS)
  {
    displayAlarms();
  }
  else if(mActiveDisplay==DISPLAY_PRESSURE)
  {
    #ifdef BAROMETER
    displayPressureGraph();
    #endif
  }
}

void TestAlarms(DateTime aTime)
{
#ifdef ENABLE_ALARMS
  bool lNewAlarmActive = false;
  for(uint8_t i=0; i<NR_OF_ALARMS; i++)
  {
    if( mAlarms[i]->IsActive(aTime))
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
#endif
}

void StartAlarmSound()
{
  mAlarmActive = true;
  mMp3.SetVolume(ALARM_START_VOLUME);
  mMp3.Start();
}

void StopAlarmSound()
{
  mAlarmActive = false;
  mMp3.Stop();  
}


void displayAlarms()
{
  #ifdef ENABLE_ALARMS
  for(uint8_t i=0; i<NR_OF_ALARMS; i++)
  {
    displayAlarm(i);
  }
  #endif
}

void displayAlarm(byte aIndex)
{
  #ifdef ENABLE_ALARMS
  uint16_t yPos = YPOS_ALARM + aIndex*((TFT_HEIGHT-YPOS_ALARM)/NR_OF_ALARMS);
  mTft.fillRect(XPOS_ALARM,yPos,WIDTH_ALARM,HEIGHT_ALARM, BLACK);
  
  if(aIndex==mMenuSelectedAlarm)   // menu is active
  {
    switch(mMenuState)
    {
      case MenuSelectAlarm:
        mTft.fillRect(XPOS_ALARM,yPos,WIDTH_ALARM,HEIGHT_ALARM, BLUE);  
        break;
      case MenuChangeAlarmDay:
        mTft.fillRect(XPOS_ALARM+0,yPos,2*6*ALARM_SIZE,HEIGHT_ALARM, BLUE);  
        break;
      case MenuChangeAlarmHour:
        mTft.fillRect(XPOS_ALARM+3*6*ALARM_SIZE,yPos,2*6*ALARM_SIZE,HEIGHT_ALARM, BLUE);  
        break;
      case MenuChangeAlarmMinute:
        mTft.fillRect(XPOS_ALARM+6*6*ALARM_SIZE,yPos,2*6*ALARM_SIZE,HEIGHT_ALARM, BLUE);  
        break;
    }
  }

  mTft.setCursor(XPOS_ALARM,yPos);
  mTft.print(mAlarms[aIndex]->GetString());
#endif
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
#ifdef BAROMETER
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
#endif
}

void displayPressureGraph()
{
#ifdef BAROMETER

  showTempPressure();

  mTft.drawLine(XPOS_PRES_GRAPH, YPOS_PRES_GRAPH, XPOS_PRES_GRAPH+NR_OF_PRESSURES, YPOS_PRES_GRAPH, GRAY1);    
  mTft.drawLine(XPOS_PRES_GRAPH, YPOS_PRES_GRAPH+HEIGHT_PRES_GRAPH, XPOS_PRES_GRAPH+NR_OF_PRESSURES, YPOS_PRES_GRAPH+HEIGHT_PRES_GRAPH, GRAY1);     
  mTft.setCursor(XPOS_PRES_GRAPH,YPOS_PRES_GRAPH-2*TFT_CHAR_HEIGHT);   
  mTft.print(MIN_PRESSURE_STR);
  mTft.setCursor(XPOS_PRES_GRAPH+NR_OF_PRESSURES-4*TFT_CHAR_WIDTH,YPOS_PRES_GRAPH+HEIGHT_PRES_GRAPH+2);   
  mTft.print(MAX_PRESSURE_STR);
  
  for(int i=0; i<NR_OF_PRESSURES; i++)
  {
    mTft.drawPixel(XPOS_PRES_GRAPH+i, YPOS_PRES_GRAPH+HEIGHT_PRES_GRAPH-mHistory[i], GRAPHCOLOR);
  }
#endif
}

/*
 * show information of weatherforcast, 4 days with each:
 * 
 *   icon-day icon-night
 *   temp max temp-min
 *   wind     rain
 */

void setWeatherForcast(byte aSection, byte aInfo[],byte aDay)
{
  if( aSection<8)
  {
    uint16_t offset = WEATHER_INFO_OFFSET + aSection * (WEATHER_INFO_SIZE+2);
    for( uint8_t i=0; i<WEATHER_INFO_SIZE; i++)
    {
      EEPROM.update(offset++, aInfo[i]);
    }
    EEPROM.update(offset, aDay);
  }
}

void clearWeatherForcast(byte aSection)
{
  if( aSection<8)
  {
//    uint16_t offset = WEATHER_INFO_OFFSET + aSection * (WEATHER_INFO_SIZE+2);
//    for( uint8_t i=0; i<WEATHER_INFO_SIZE; i++)
//    {
//      EEPROM.update(offset++, 0x00);
//    }
    uint16_t offset = WEATHER_INFO_OFFSET + aSection * (WEATHER_INFO_SIZE+2) + WEATHER_INFO_SIZE;
    EEPROM.update(offset, 0xFF);  // indication of invalid
  }
}

bool getWeatherForcast(byte aSection, byte aInfo[])
{
  bool lValid = false;
  if( aSection<8)
  {
    uint16_t offset = WEATHER_INFO_OFFSET + aSection * (WEATHER_INFO_SIZE+2);
    for(uint8_t i=0; i<WEATHER_INFO_SIZE; i++)
    {
      aInfo[i] = EEPROM.read(offset++);
    }
    aInfo[WEATHER_INFO_SIZE] = EEPROM.read(offset);   // day
    if(aInfo[WEATHER_INFO_SIZE]<8)
      lValid = true;
  }
  return lValid;
}
void showWeatherForcast(void)
{
  #define LINE_HEIGHT 20

  char str[10];
  int xPos;
  int yPos;
  byte lInfo[WEATHER_INFO_SIZE+1];

  mTft.fillScreen(BLACK);
  //mTft.setTextColor(WHITE);
  mTft.drawLine(0, TFT_HEIGHT/2, TFT_WIDTH, TFT_HEIGHT/2, WHITE);    
  mTft.drawLine(TFT_WIDTH/2, 0, TFT_WIDTH/2, TFT_HEIGHT, WHITE);    

  // there are in total 8 sections, two for each day (min and max)
  int day = 0;
  
  for( uint8_t lSection=0; lSection<8; lSection++)
  {
    yPos = (day<2)?0:(1+TFT_HEIGHT/2);
    xPos = (day==0 || day==2)?10:(11+TFT_WIDTH/2);

    // first the high-values of the day:
    if (getWeatherForcast(lSection,lInfo))
    {
      mTft.setCursor(xPos+20,yPos);
      strcpy_P(str, (char*)pgm_read_word(&(days_table[lInfo[WEATHER_INFO_SIZE]])));
      mTft.print(str);
      yPos += LINE_HEIGHT;
      // weather at daytime
      showWeatherIcon(xPos, yPos, Weather::GetWeatherDay(lInfo));    
      // weather at nighttime
      showWeatherIcon(xPos+WIDTH_WEATHER, yPos, Weather::GetWeatherNight(lInfo) + NR_OF_WEATHER_ICONS);
    
      yPos += WEATHERICON_HEIGHT+4;
    
      sprintf(str,"%dC",Weather::GetTemp(lInfo));
      mTft.setCursor(xPos,yPos);   
      mTft.print(str);
    
      yPos += LINE_HEIGHT;
      sprintf(str,"%d%%",Weather::GetRain(lInfo));
      mTft.setCursor(xPos,yPos);   
      mTft.print(str);
    }
    
    // than the low-values
    // (unknown: again weather info for day and night? skip this info)
    lSection++;
    if (getWeatherForcast(lSection,lInfo))
    {
      yPos = (day<2)?0:(1+TFT_HEIGHT/2);
      yPos += LINE_HEIGHT;    
      yPos += WEATHERICON_HEIGHT+4;
      xPos += WIDTH_WEATHER;

      if (lSection!=7)    // Only wind data is valid 
      {
        sprintf(str,"%dC",Weather::GetTemp(lInfo));
        mTft.setCursor(xPos,yPos);   
        mTft.print(str);
      }
      yPos += LINE_HEIGHT;
      strcpy(str,Weather::GetWindDirection(lInfo));
      strcat(str,Weather::GetWindForce(lInfo));
      mTft.setCursor(xPos,yPos);   
      mTft.print(str);
    }
    day++;
  }  
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
    {bmpWeer_4,bmpWeer_9},
    // for night time
    {bmpWeer_1N,0},
    {bmpWeer_2N,0},
    {bmpWeer_3N,0},
    {bmpWeer_4,0},
    {bmpWeer_3N,bmpWeer_5},
    {bmpWeer_4,bmpWeer_6},
    {bmpWeer_4,bmpWeer_7},
    {0,bmpWeer_8},
    {bmpWeer_3N,bmpWeer_9},
    {bmpWeer_3N,bmpWeer_6},
    {bmpWeer_4,bmpWeer_11},
    {bmpWeer_3N,bmpWeer_7},
    {bmpWeer_4,bmpWeer_13},
    {bmpWeer_1,bmpWeer_8},
    {bmpWeer_4,bmpWeer_9}
  };


void showWeatherIcon(int aX, int aY, byte aIcon)
{
  #ifdef USE_WEATHER

  mTft.fillRect(aX, aY, WEATHERICON_WIDTH, WEATHERICON_HEIGHT, BLACK);

  if(aIcon==1)
  {
      mTft.drawBitmap(aX, aY,
              bmpWeer_1, 
              WEATHERICON_WIDTH,WEATHERICON_HEIGHT,
              YELLOW,BLACK);

  }
  else if(aIcon>0 && aIcon<=31)
  {
    const byte* pIcon = WeatherImages[aIcon-2][0];
    if (pIcon!=0)
    {
      mTft.drawBitmap(aX, aY,
                  pIcon, 
                  WEATHERICON_WIDTH,26,
                  LIGHTGRAY,BLACK);
    }
    pIcon = WeatherImages[aIcon-2][1];
    if (pIcon!=0)
    {
      mTft.drawBitmap(aX, aY+26,
                  pIcon,
                  WEATHERICON_WIDTH,20,
                  LIGHTBLUE,BLACK);
    }
  }
#endif
}

void showSunMoon(DateTime aDateTime)
{
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

  //mTft.setTextColor(WHITE);

  // Sun rise and set:
  // use top halve of sunny weather icon
  mTft.drawBitmap(XPOS_SUN+8, YPOS_SUN,
                  bmpWeer_1, 
                  WEATHERICON_WIDTH,24,
                  YELLOW,BLACK);
            
  mTft.setCursor(XPOS_SUN,0+36);  
  mTft.print(GetSunRise(aDateTime).GetTimeStr());
  mTft.setCursor(XPOS_SUN,36+20);  
  mTft.print(GetSunSet(aDateTime).GetTimeStr());
}

void handleTempPressure()
{
  getTempAndPressure(mCurrentTemp,mCurrentPress);
}

//  #ifdef BAROMETER
//  // update pressure graphics 
//  if(aAddNewSample)
//  {
//    // scale the measured value to the available display area (in this case not needed)
//    registerPressure((byte)mCurrentPress);
//  }
//  #endif
//}

void showTempPressure()
{
  char str[10];

  mTft.fillRect(XPOS_PRES,YPOS_PRES,WIDTH_TEMPPRES,HEIGHT_TEMPPRES, BLACK);
  //mTft.setTextColor(WHITE);  
  
  mTft.setCursor(XPOS_PRES,YPOS_PRES);
  sprintf(str,"%4d mBar",(int)mCurrentPress);
  mTft.print(str);

  sprintf(str,"%d.%1d C",(int)mCurrentTemp,(int)(mCurrentTemp*10)%10);
  mTft.setCursor(XPOS_TEMP,YPOS_TEMP);
  mTft.print(str);
}

void showTime(DateTime aDateTime)
{
  char str[12];
  char strDate[STRLEN_DATE+1];

  mTft.fillRect(XPOS_TIME,YPOS_TIME,WIDTH_TIME,HEIGHT_TIME, BLACK);
  mTft.setCursor(XPOS_TIME,YPOS_TIME);
  mTft.setTextColor(mAlarmActive?LIGHTRED:WHITE);  
  mTft.setTextSize(TIME_SIZE);
  mTft.print(aDateTime.GetTimeStr());
  #ifdef DEBUG
  Serial.println(aDateTime.GetTimeStr());
  #endif
  mTft.setTextSize(2);  // terug naar default
  mTft.setTextColor(WHITE);  

  if ((aDateTime.dayOfWeek() != mPrevDate.dayOfWeek()) ||
      (aDateTime.d != mPrevDate.d) ||
      (aDateTime.m != mPrevDate.m) || 
      (aDateTime.year() != mPrevDate.year()))
  {
    mTft.fillRect(0,YPOS_DATE,TFT_WIDTH,HEIGHT_DATE, BLACK);
    mTft.setCursor(XPOS_DATE,YPOS_DATE);
    strcpy_P(strDate, (char*)pgm_read_word(&(days_table[aDateTime.dayOfWeek()])));
    strcat(strDate,aDateTime.GetDateStr());
    mTft.print(strDate);
    
    mPrevDate = aDateTime;

    showSunMoon(aDateTime);
  }
}

void showActiveAlarms()
{
  int xPos = XPOS_ACTIVE_ALARMS;
  int yPos = YPOS_ACTIVE_ALARMS;

  for(byte i=0; i<NR_OF_ALARMS; i++)
  {
    if(mAlarms[i]->IsEnabled())
    {
      mTft.setCursor(xPos,yPos);
      mTft.print(mAlarms[i]->GetString());
      yPos += HEIGHT_ALARM;
      if(yPos>=(TFT_HEIGHT-HEIGHT_ALARM))
      {
         xPos += (TFT_WIDTH/2);
         yPos = YPOS_ACTIVE_ALARMS;
      }
    }
  }
}

void handleKeys()
{
  static bool keyIdle=true;
  unsigned int lAdcValues[4];
  mAdc.ReadADC(lAdcValues,4);

  if(keyIdle)
  {
    mKeyUp   = (lAdcValues[1]<1000);
    mKeyDown = (lAdcValues[1]>25000);
    mKeyLeft = (lAdcValues[0]<1000);
    mKeyRight = (lAdcValues[0]>25000);

    if(mKeyUp || mKeyDown || mKeyLeft || mKeyRight)
      keyIdle = false;

    #ifdef DEBUG
      if(mKeyUp) Serial.println(F("Up"));
      if(mKeyDown) Serial.println(F("Down"));
      if(mKeyLeft) Serial.println(F("Left"));
      if(mKeyRight) Serial.println(F("Right"));
    #endif
  }
  else   // wait until key has been released
  {
    if(lAdcValues[1]>1000 && lAdcValues[1]<25000 &&
       lAdcValues[0]>1000 && lAdcValues[0]<25000)
       {
         keyIdle = true;
       }
  }

  if(lAdcValues[2]<1000)
  {
    #ifdef DEBUG
    Serial.println(F("Next screen"));
    #endif
    if(mActiveDisplay==DISPLAY_ALARMS)
    {
      for(byte i=0; i<NR_OF_ALARMS; i++)
      {
        mAlarms[i]->StoreAlarm();
      }
    }
    mMenuState=0;
    mMenuSelectedAlarm = 0;
    mActiveDisplay++;
    mActiveDisplay %= 4;
    ActivateDisplay(mActiveDisplay);
  }
  
  mLDR = lAdcValues[3];
}

#ifdef HANDLE_MENU
void handleMenu(unsigned int aElapsedTimeMs)
{
  if(mActiveDisplay==DISPLAY_ALARMS)
  {
    handleMenuDo(mMenuState);
    sMenuStates lNextMenuState = handleMenuTransfer(mMenuState);
    if(lNextMenuState != mMenuState)
    {
      handleMenuExit(mMenuState);
      mMenuState = lNextMenuState;
      handleMenuEntry(mMenuState);
    }
  }
  if (mKeyUp || mKeyDown || mKeyLeft || mKeyRight)
  {
    mKeyUp = false;
    mKeyDown = false;
    mKeyLeft = false;
    mKeyRight = false;
  }
}

void handleMenuDo(sMenuStates aMenuState)
{
  if(mKeyUp)
  {
    if(aMenuState==MenuSelectAlarm)
    {
      if(mMenuSelectedAlarm>0)
      {
        mMenuSelectedAlarm--;
        displayAlarms();
      }
    }
    else if(aMenuState==MenuChangeAlarmDay)
    {
      mAlarms[mMenuSelectedAlarm]->NextDay(true);        
      displayAlarm(mMenuSelectedAlarm);
    }
    else if(aMenuState==MenuChangeAlarmHour)
    {
      mAlarms[mMenuSelectedAlarm]->NextHour(true);        
      displayAlarm(mMenuSelectedAlarm);
    }
    else if(aMenuState==MenuChangeAlarmMinute)
    {
      mAlarms[mMenuSelectedAlarm]->NextMinute(true);        
      displayAlarm(mMenuSelectedAlarm);
    }
  }
  if(mKeyDown)
  {
    if(aMenuState==MenuSelectAlarm)
    {
      if(mMenuSelectedAlarm<(NR_OF_ALARMS-1))
      {
        mMenuSelectedAlarm++;
        displayAlarms();
      }
    }
    else if(aMenuState==MenuChangeAlarmDay)
    {
      mAlarms[mMenuSelectedAlarm]->NextDay(false);        
      displayAlarm(mMenuSelectedAlarm);
    }
    else if(aMenuState==MenuChangeAlarmHour)
    {
      mAlarms[mMenuSelectedAlarm]->NextHour(false);        
      displayAlarm(mMenuSelectedAlarm);
    }
    else if(aMenuState==MenuChangeAlarmMinute)
    {
      mAlarms[mMenuSelectedAlarm]->NextMinute(false);        
      displayAlarm(mMenuSelectedAlarm);
    }
  }
}

sMenuStates handleMenuTransfer(sMenuStates aMenuState)
{
  byte lNewMenuState = aMenuState;

    if(mKeyRight)
    {
      lNewMenuState++;
    }
    else if(mKeyLeft)
    {
      lNewMenuState--;
    }
//    if(lNewMenuState>MenuChangeAlarmMinute)
//      lNewMenuState = MenuChangeAlarmMinute;
    lNewMenuState %= (MenuChangeAlarmMinute+1);
      
#ifdef DEBUG
  if(lNewMenuState!=aMenuState)
  {
    Serial.print(F("state changed to "));
    Serial.println(lNewMenuState);
  }
#endif

  return lNewMenuState;
}

void handleMenuEntry(sMenuStates aMenuState)
{
  //if (aMenuState==MenuIdle || aMenuState==MenuSelectAlarm)
  if (aMenuState==MenuSelectAlarm)
      displayAlarms();
      
  if (aMenuState==MenuChangeAlarmDay || 
      aMenuState==MenuChangeAlarmHour ||
      aMenuState==MenuChangeAlarmMinute)
    displayAlarm(mMenuSelectedAlarm);
}

void handleMenuExit(sMenuStates aMenuState)
{
//  if (aMenuState == MenuIdle)
//      mMenuSelectedAlarm = 0;
}
#endif

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

void HandleBacklight()
{
  unsigned int level = 100 - mLDR/328;
  ShowBacklight();
}

void ShowBacklight()
{
  #ifdef SHOW_LDR
  if(mActiveDisplay==DISPLAY_PRESSURE)
  {
    char str[6];
    unsigned int level = 100 - mLDR/328;
    sprintf(str,"%d%%",level);
    mTft.fillRect(XPOS_LDR,YPOS_LDR,TFT_WIDTH,2*TFT_CHAR_HEIGHT, BLACK);
    mTft.setCursor(XPOS_LDR,YPOS_LDR);
    mTft.print(str);
    if(mPirState)
    {
      mTft.setCursor(XPOS_LDR+5*2*TFT_CHAR_WIDTH,YPOS_LDR);
      mTft.print("PIR");    
    }
  }
  #endif
}

void TestWeatherIcons()
{ 
  int xPos = 0;
  int yPos = 0;
  uint8_t i=0;
    mTft.fillScreen(BLACK);

  for( ;i<8; i++)
  {
      showWeatherIcon(xPos, yPos, i);
      xPos += (TFT_WIDTH/4);
      if (xPos>TFT_WIDTH)
      {
        xPos=0;
        yPos += (TFT_HEIGHT/2);
      }
  }
  delay(5000);  
  xPos = 0;
  yPos = 0;
    mTft.fillScreen(BLACK);

  for( ; i<16; i++)
  {
      showWeatherIcon(xPos, yPos, i);
      xPos += (TFT_WIDTH/4);
      if (xPos>TFT_WIDTH)
      {
        xPos=0;
        yPos += (TFT_HEIGHT/2);
      }
  }
  delay(5000);  

  xPos = 0;
  yPos = 0;
    mTft.fillScreen(BLACK);

  for( ; i<24; i++)
  {
      showWeatherIcon(xPos, yPos, i);
      xPos += (TFT_WIDTH/4);
      if (xPos>TFT_WIDTH)
      {
        xPos=0;
        yPos += (TFT_HEIGHT/2);
      }
  }
  delay(5000);  

  xPos = 0;
  yPos = 0;
    mTft.fillScreen(BLACK);

  for( ; i<32; i++)
  {
      showWeatherIcon(xPos, yPos, i);
      xPos += (TFT_WIDTH/4);
      if (xPos>TFT_WIDTH)
      {
        xPos=0;
        yPos += (TFT_HEIGHT/2);
      }
  }
  delay(5000);  
}
