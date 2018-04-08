#ifndef WEATHER_H
#define WEATHER_H

#include "Arduino.h"

class Weather
{
public:
	static byte GetWeatherDay(const byte* aInfo);
	static byte GetWeatherNight(const byte* aInfo);
  static const char* Weather::GetWeatherDayStr(const byte* aInfo);
  static const char* Weather::GetWeatherNightStr(const byte* aInfo);
	static byte GetAnomalie(const byte* aInfo);
	static int GetTemp(const byte* aInfo);
	static byte GetExtreme(const byte* aInfo);
	static byte GetRain(const byte* aInfo);
	static const char* GetWindDirection(const byte* aInfo);
	static const char* GetWindForce(const byte* aInfo);
private:
};

#endif
