#include "DateTime.h"
#include <Arduino.h> // capital A so it is error prone on case-sensitive filesystems

#define DS1307_ADDRESS 0x68
#define SECONDS_PER_DAY 86400L

#define SECONDS_FROM_1970_TO_2000 946684800

////////////////////////////////////////////////////////////////////////////////
// utility code, some of this could be exposed in the DateTime API if needed

#define LEAP_YEAR(_year) ((_year%4)==0)
const uint8_t daysInMonth [] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 }; //has to be const or compiler compaints
//const float longitude PROGMEM =  -5.73;      // Lengtegraad in decimale graden, <0 voor OL, >0 voor WL
//const float latitude PROGMEM = 51.81;     // Breedtegraad in decimale graden, >0 voor NB, <0 voor ZB
#define longitude -5.73      // Lengtegraad in decimale graden, <0 voor OL, >0 voor WL
#define latitude 51.81     // Breedtegraad in decimale graden, >0 voor NB, <0 voor ZB

// number of days since 2000/01/01, valid for 2001..2099

static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d)
{
    if (y >= 2000)
        y -= 2000;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i)
        days += pgm_read_byte(daysInMonth + i - 1);
    if (m > 2 && y % 4 == 0)
        ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
}
static uint16_t date2days(DateTime aDate)
{
  return date2days(aDate.yOff, aDate.m, aDate.d);
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
    return ((days * 24L + h) * 60 + m) * 60 + s;
}

static unsigned long makeTime(DateTime aDate)
  {
  // converts time components to time_t
  // note year argument is full four digit year (or digits since 2000), i.e.1975, (year 8 is 2008)

     int i;
     int year = aDate.yOff + 2000;
     unsigned long seconds;

      // seconds from 1970 till 1 jan 00:00:00 this year
      seconds= (year-1970)*(60*60*24L*365);

      // add extra days for leap years
      for (i=1970; i<year; i++)
      {
          if (LEAP_YEAR(i))
          {
              seconds+= 60*60*24L;
          }
      }
      // add days for this year
      for (i=0; i<aDate.m; i++)
      {
        if (i==1 && LEAP_YEAR(year))
        {
          seconds+= 60*60*24L*29;
        }
        else
        {
          seconds+= 60*60*24L*daysInMonth[i];
        }
      }

      seconds+= (aDate.d-1)*3600*24L;
      seconds+= aDate.hh*3600L;
      seconds+= aDate.mm*60L;
      seconds+= aDate.ss;
      return seconds;
  }
  
//  static DateTime HoursMinutes(float aTime)
//  {
//    DateTime time;
//
//    float hrs;
//    float min = modff(aTime, &hrs);
//
//    min = round(60.0*min);
//    if (min == 60)        // Voorkom dat er 16:60 verschijnt ipv 17:00
//    {
//      min = 0;
//      hrs += 1;
//      if (hrs > 24 )
//        hrs = 0;
//    }
//
//    time.hh = hrs;
//    time.mm = min;
//    time.ss = 0;
//    return time;
//  }
static int DiffinDays(DateTime aDate1, DateTime aDate2)
{
  uint16_t days1 = date2days(aDate1);
  uint16_t days2 = date2days(aDate2);
  if(days1<days2)
    return days2-days1;
  return days1-days2;
}

byte GetMoonPhase(DateTime aDate)
{
  // Note that new moon is 1 i.s.o. zero
	float MaanCyclus = 29.530589;
	DateTime NieuweMaan(5,5,7,0,0,0);  //"June 7, 2005 00:00:00");  2005-2000=5
	//float AantalDagen = (float)DiffinDays(aDate,NieuweMaan);
	//float CyclusDeel = AantalDagen/MaanCyclus - floor(AantalDagen/MaanCyclus); // Alleen de decimalen
  //return round(MaanCyclus*CyclusDeel);
  int AantalDagen = DiffinDays(aDate,NieuweMaan);
	float CyclusDeel = AantalDagen/MaanCyclus - int(AantalDagen/MaanCyclus); // Alleen de decimalen
  return int(MaanCyclus*CyclusDeel+0.5);
}

DateTime getSunTime(DateTime aDate, bool aRiseNotSet)
{
	float eqtime,hars;
	float doy = date2days(aDate);//aDate.yOff+2000, aDate.m, aDate.d);

	doy += 0.5;

	// equation of time (in minutes)
	float x = doy*2*PI/365; // fractional year in radians
	eqtime = 229.18*(0.000075+0.001868*cos(x) - 0.032077*sin(x) - 0.014615*cos(2*x) - 0.040849*sin(2*x));

	// declination (in degrees)
	float declin = 0.006918-0.399912*cos(x) + 0.070257*sin(x) - 0.006758*cos(2*x);
	declin = declin + 0.000907*sin(2*x) - 0.002697*cos(3*x) + 0.00148*sin(3*x);
	declin = declin * 180/PI;

	// solar azimuth angle for sunrise and sunset corrected for atmospheric refraction (in degrees),
	x = PI/180;
	hars = cos(x*90.833) / (cos(x*latitude) * cos(x*declin));
	hars = hars - tan(x*latitude)*tan(x*declin);
	hars = acos(hars)/x;

	// sunrise
	x = 720 - eqtime;
	if (aRiseNotSet)  // Sunrise
	  x += 4*(longitude-hars);
	else    // sunset
	  x += 4*(longitude+hars);

	x = x/60 + 1;  // from UTC to CET
	if (aDate.dst) // extra daylight saving time correction
	  x++;

    DateTime lSunTime;

/**
    float hrs;
    float min = modff(x, &hrs);

    min = round(60.0*min);
**/
    int hrs = (int)(x);
    x -= hrs; // remove integral part
    x *= 60;  
    int min = (int)x;

    if (min == 60)        // Voorkom dat er 16:60 verschijnt ipv 17:00
    {
      min = 0;
      hrs += 1;
      if (hrs > 24 )
        hrs = 0;
    }

    lSunTime.hh = hrs;
    lSunTime.mm = min;
    lSunTime.ss = 0;
    return lSunTime;
}

DateTime GetSunRise(DateTime aDate)
{
  return getSunTime(aDate, true);
}
DateTime GetSunSet(DateTime aDate)
{
  return getSunTime(aDate, false);
}


////////////////////////////////////////////////////////////////////////////////
// DateTime implementation - ignores time zones and DST changes
// NOTE: also ignores leap seconds, see http://en.wikipedia.org/wiki/Leap_second

DateTime::DateTime (uint32_t t)
{
  t -= SECONDS_FROM_1970_TO_2000;    // bring to 2000 timestamp from 1970

    ss = t % 60;
    t /= 60;
    mm = t % 60;
    t /= 60;
    hh = t % 24;
    uint16_t days = t / 24;
    uint8_t leap;
    for (yOff = 0; ; ++yOff) {
        leap = yOff % 4 == 0;
        if (days < 365 + leap)
            break;
        days -= 365 + leap;
    }
    for (m = 1; ; ++m) {
        uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
        if (leap && m == 2)
            ++daysPerMonth;
        if (days < daysPerMonth)
            break;
        days -= daysPerMonth;
    }
    d = days + 1;
}

DateTime::DateTime (uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec)
{
    if (year >= 2000)
        year -= 2000;
    yOff = year;
    if((month<=12) &&
       (day<=31) &&
       (hour<=24) &&
       (min<=60) &&
       (sec<=60))
    {
      m = month;
      d = day;
      hh = hour;
      mm = min;
      ss = sec;
    }
}

void DateTime::Clear()
{
    yOff = 0;
    m = 0;
    d = 0;
    hh = 0;
    mm = 0;
    ss = 0;
}

bool DateTime::IsValid()
{
  return (d != 0);
}

static char mStr[12];
const char* DateTime::GetTimeStr()
{
  sprintf(mStr,"%02d:%02d",hh,mm);
  return mStr;
}
const char* DateTime::GetDateStr()
{
  sprintf(mStr," %02d-%02d-%4d",d,m,yOff+2000);
  return mStr;
}
bool DateTime::operator==(const DateTime theOther)
{
  return ((yOff == theOther.yOff) &&
          (m == theOther.m) &&
          (d == theOther.d) &&
          (hh == theOther.hh) &&
          (mm == theOther.mm));
}
bool DateTime::operator!=(const DateTime theOther)
{
  return !(*this==theOther);
}

static uint8_t conv2d(const char* p)
{
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}

// A convenient constructor for using "the compiler's time":
//   DateTime now (__DATE__, __TIME__);
// NOTE: using PSTR would further reduce the RAM footprint
DateTime::DateTime (const char* date, const char* time)
{
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    yOff = conv2d(date + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec 
    switch (date[0]) {
        case 'J': m = date[1] == 'a' ? 1 : m = date[2] == 'n' ? 6 : 7; break;
        case 'F': m = 2; break;
        case 'A': m = date[2] == 'r' ? 4 : 8; break;
        case 'M': m = date[2] == 'r' ? 3 : 5; break;
        case 'S': m = 9; break;
        case 'O': m = 10; break;
        case 'N': m = 11; break;
        case 'D': m = 12; break;
    }
    d = conv2d(date + 4);
    hh = conv2d(time);
    mm = conv2d(time + 3);
    ss = conv2d(time + 6);
}

uint8_t DateTime::dayOfWeek() const {    
    uint16_t day = date2days(yOff, m, d);
    return (day + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
}

uint32_t DateTime::unixtime(void) const {
  uint32_t t;
  uint16_t days = date2days(yOff, m, d);
  t = time2long(days, hh, mm, ss);
  t += SECONDS_FROM_1970_TO_2000;  // seconds from 1970 to 2000

  return t;
}
 
