
#include "Weather.h"

byte swabNibble(byte aValue)
{
  byte value = 0 ;
  for(byte i=0; i<4; i++)
  {
    value <<= 1;
    value |= (aValue&0x01);
    aValue >>= 1;
  }
  return value;
}
/*
HIGH:- Tag..............................: 0010            4: 0..3
     - Nacht............................: 0010            4: 4..7
     - Wetterextreme....................: 0000            4: 8..11
     - Niederschlagswahrscheinlichkeit..: 010             3: 12..14
     - Wetteranomalie...................: 0               1: 15
     - Temperatur.......................: 011101          5: 16..21

Low: - Tag..............................: 1000            4
     - Nacht............................: 1100            4
     - Windrichtung.....................: 0010            4
     - Windstärke.......................: 010             3
     - Wetteranomalie...................: 0               1
     - Temperatur.......................: 110101          5
*/

/*
 * 0   --
 * 1   sonnig
 * 2   leicht bewölkt
 * 3   vorwiegend bewölkt
 * 4   bedeckt
 * 5   Wärmegewitter
 * 6   starker Regen
 * 7   Schneefall
 * 8   Nebel
 * 9   Schneeregen
 * 10   Regenschauer
 * 11   leichter Regen
 * 12   Schneeschauer
 * 13   Frontengewitter
 * 14   Hochnebel
 * 15   Schneeregen-schauer
 */

/*
const char weer0[] PROGMEM = "-";
const char weer1[] PROGMEM = "zonnig";
const char weer2[] PROGMEM = "licht bew.";
const char weer3[] PROGMEM = "bewolkt";
const char weer4[] PROGMEM = "zwaar bew.";
const char weer5[] PROGMEM = "warmteonweer";
const char weer6[] PROGMEM = "zware regen";
const char weer7[] PROGMEM = "sneeuw";
const char weer8[] PROGMEM = "mist";
const char weer9[] PROGMEM = "sneeuwregen";
const char weer10[] PROGMEM = "buien";
const char weer11[] PROGMEM = "lichte regen";
const char weer12[] PROGMEM = "sneeuwbuien";
const char weer13[] PROGMEM = "onweer";
const char weer14[] PROGMEM = "hoge mist";
const char weer15[] PROGMEM = "natte sneeuw";
const char* const weer_table[] PROGMEM = {weer0,weer1,weer2,weer3,weer4,weer5,weer6,weer7,weer8,weer9,weer10,weer11,weer12,weer13,weer14,weer15};
*/
const char winddir0[] PROGMEM = "N";
const char winddir1[] PROGMEM = "NO";
const char winddir2[] PROGMEM = "O";
const char winddir3[] PROGMEM = "ZO";
const char winddir4[] PROGMEM = "Z";
const char winddir5[] PROGMEM = "ZW";
const char winddir6[] PROGMEM = "W";
const char winddir7[] PROGMEM = "NW";
const char* const winddir_table[] PROGMEM = {winddir0,winddir1,winddir2,winddir3,winddir4,winddir5,winddir6,winddir7};

const char windforce0[] PROGMEM = "?";
const char windforce1[] PROGMEM = "0-2";
const char windforce2[] PROGMEM = "3-4";
const char windforce3[] PROGMEM = "5-6";
const char windforce4[] PROGMEM = "7";
const char windforce5[] PROGMEM = "8";
const char windforce6[] PROGMEM = "9";
const char windforce7[] PROGMEM = ">9";
const char* const windforce_table[] PROGMEM = {windforce0,windforce1,windforce2,windforce3,windforce4,windforce5,windforce6,windforce7};
//const char dirs[8][3]   = {"N","NO","O","SO","S","SW","W","NW"};
//const char forces[8][4] = {"?","0-2","3-4","5-6","7","8","9",">9"};

/* generic function for both highest and lowest sections */
byte Weather::GetWeatherDay(const byte* aInfo)
{
  byte v = (aInfo[0] >> 4);
  return swabNibble(v);
}
byte Weather::GetWeatherNight(const byte* aInfo)
{
  byte v = (aInfo[0] & 0x0F);
  return swabNibble(v);
}

static char str[15];
//const char* Weather::GetWeatherDayStr(const byte* aInfo)
//{
//  byte v = GetWeatherDay(aInfo);
//  strcpy_P(str, (char*)pgm_read_word(&(weer_table[v]))); 
//  return str;
//}
//const char* Weather::GetWeatherNightStr(const byte* aInfo)
//{
//  //static char str[15];
//  byte v = GetWeatherNight(aInfo);
//  strcpy_P(str, (char*)pgm_read_word(&(weer_table[v]))); 
//  return str;
//}
byte Weather::GetAnomalie(const byte* aInfo)
{
  byte v = (aInfo[1]&0x01);
  return v;
}
int Weather::GetTemp(const byte* aInfo)
{
  byte t = 0 ;
  byte info = aInfo[2] >> 2;
  for(byte i=0; i<6; i++)
  {
    t <<= 1;
    t |= (info&0x01);
    info >>= 1;
  }
  
  return (t - 22);
}

/* function for highest sections */
byte Weather::GetExtreme(const byte* aInfo)
{
  byte v =  (aInfo[1] >> 4);
  return swabNibble(v);
}

byte Weather::GetRain(const byte* aInfo)
{
  byte v = (aInfo[1] & 0x0E);
  v = swabNibble(v);
  v *= 15;
  if (v>100)
    v = 100;
  return v;
}

/* function for lowest sections */
const char* Weather::GetWindDirection(const byte* aInfo)
{
  // wind direction overlaps extreme
  byte d = GetExtreme(aInfo);
  strcpy_P(str, (char*)pgm_read_word(&(winddir_table[d]))); 
  return str;
}
const char* Weather::GetWindForce(const byte* aInfo)
{
  byte v = (aInfo[1] & 0x0E);
  v = swabNibble(v);
  strcpy_P(str, (char*)pgm_read_word(&(windforce_table[v]))); 
  return str;
}


