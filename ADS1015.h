#ifndef ADS1015_H
#define ADS1015_H

#include "Arduino.h"

class Ads1015
{
public:
	void ReadADC(unsigned int *aAdcValues, byte aCount);
	float GetAdc(byte aChannel);

private:
	const float cAdcRange = (125.0/1000000.0);    // one lsb is 125 uV

	void writeAdcRegister(uint8_t reg, uint16_t value);
	uint16_t readAdcRegister(uint8_t reg);
	uint16_t readADC_SingleEnded(uint8_t channel);
};

#endif