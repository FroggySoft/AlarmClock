#include "ProjSettings.h"

#include <Wire.h>
#include <avr/pgmspace.h>
#include "RTClib.h"

#define DS1307_ADDRESS 0x68
#define SECONDS_PER_DAY 86400L

int i = 0; //The new wire library needs to take an int when you are sending for the zero register

////////////////////////////////////////////////////////////////////////////////
// RTC_DS1307 implementation

static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }

uint8_t RTC_DS1307::begin(void) {
  return 1;
}


//uint8_t RTC_DS1307::isrunning(void) {
//  Wire.beginTransmission(DS1307_ADDRESS);
//  Wire.write(i);	
//  Wire.endTransmission();
//
//  Wire.requestFrom(DS1307_ADDRESS, 1);
//  uint8_t ss = Wire.read();
//  return !(ss>>7);
//}

void RTC_DS1307::adjust(const DateTime& dt)
{
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write(i);
    //Wire.write(bin2bcd(dt.second()));
    Wire.write(bin2bcd(0));
    Wire.write(bin2bcd(dt.mm));
    Wire.write(bin2bcd(dt.hh));
    Wire.write(bin2bcd(0));
    Wire.write(bin2bcd(dt.d));
    Wire.write(bin2bcd(dt.m));
    Wire.write(bin2bcd(dt.year() - 2000));
    Wire.write(i);
    Wire.endTransmission();
}

DateTime RTC_DS1307::now()
{
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(i);	
  Wire.endTransmission();
  
  Wire.requestFrom(DS1307_ADDRESS, 7);
  uint8_t ss = bcd2bin(Wire.read() & 0x7F);
  uint8_t mm = bcd2bin(Wire.read());
  uint8_t hh = bcd2bin(Wire.read());
  Wire.read();
  uint8_t d = bcd2bin(Wire.read());
  uint8_t m = bcd2bin(Wire.read());
  uint16_t y = bcd2bin(Wire.read());    // no need to convert +2000
  DateTime now(y, m, d, hh, mm);
  return now;
}


