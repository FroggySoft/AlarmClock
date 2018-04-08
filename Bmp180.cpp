
#include "Bmp180.h"
#include "Arduino.h"
#include <Wire.h>

#define BMP180_ADDR 0x77 // 7-bit address
#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT 0xF6

#define BMP180_COMMAND_TEMPERATURE 0x2E
#define BMP180_COMMAND_PRESSURE0 0x34
#define BMP180_COMMAND_PRESSURE1 0x74
#define BMP180_COMMAND_PRESSURE2 0xB4
#define BMP180_COMMAND_PRESSURE3 0xF4

#define ADS_I2C_ADDRESS 0x48

/*=========================================================================
    CONVERSION DELAY (in mS)
    -----------------------------------------------------------------------*/
    //#define ADS1015_CONVERSIONDELAY         (1)
    #define ADS1115_CONVERSIONDELAY         (8)
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER
    -----------------------------------------------------------------------*/
    #define ADS1015_REG_POINTER_MASK        (0x03)
    #define ADS1015_REG_POINTER_CONVERT     (0x00)
    #define ADS1015_REG_POINTER_CONFIG      (0x01)
    #define ADS1015_REG_POINTER_LOWTHRESH   (0x02)
    #define ADS1015_REG_POINTER_HITHRESH    (0x03)
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER
    -----------------------------------------------------------------------*/
    #define ADS1015_REG_CONFIG_OS_MASK      (0x8000)
    #define ADS1015_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion
    #define ADS1015_REG_CONFIG_OS_BUSY      (0x0000)  // Read: Bit = 0 when conversion is in progress
    #define ADS1015_REG_CONFIG_OS_NOTBUSY   (0x8000)  // Read: Bit = 1 when device is not performing a conversion

    #define ADS1015_REG_CONFIG_MUX_MASK     (0x7000)
    #define ADS1015_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
    #define ADS1015_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
    #define ADS1015_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
    #define ADS1015_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
    #define ADS1015_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
    #define ADS1015_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
    #define ADS1015_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
    #define ADS1015_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3

    #define ADS1015_REG_CONFIG_PGA_MASK     (0x0E00)
    #define ADS1015_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3
    #define ADS1015_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
    #define ADS1015_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2 (default)
    #define ADS1015_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4
    #define ADS1015_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8
    #define ADS1015_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16

    #define ADS1015_REG_CONFIG_MODE_MASK    (0x0100)
    #define ADS1015_REG_CONFIG_MODE_CONTIN  (0x0000)  // Continuous conversion mode
    #define ADS1015_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)

    #define ADS1015_REG_CONFIG_DR_MASK      (0x00E0)  
    #define ADS1015_REG_CONFIG_DR_128SPS    (0x0000)  // 128 samples per second
    #define ADS1015_REG_CONFIG_DR_250SPS    (0x0020)  // 250 samples per second
    #define ADS1015_REG_CONFIG_DR_490SPS    (0x0040)  // 490 samples per second
    #define ADS1015_REG_CONFIG_DR_920SPS    (0x0060)  // 920 samples per second
    #define ADS1015_REG_CONFIG_DR_1600SPS   (0x0080)  // 1600 samples per second (default)
    #define ADS1015_REG_CONFIG_DR_2400SPS   (0x00A0)  // 2400 samples per second
    #define ADS1015_REG_CONFIG_DR_3300SPS   (0x00C0)  // 3300 samples per second

    #define ADS1015_REG_CONFIG_CMODE_MASK   (0x0010)
    #define ADS1015_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default)
    #define ADS1015_REG_CONFIG_CMODE_WINDOW (0x0010)  // Window comparator

    #define ADS1015_REG_CONFIG_CPOL_MASK    (0x0008)
    #define ADS1015_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
    #define ADS1015_REG_CONFIG_CPOL_ACTVHI  (0x0008)  // ALERT/RDY pin is high when active

    #define ADS1015_REG_CONFIG_CLAT_MASK    (0x0004)  // Determines if ALERT/RDY pin latches once asserted
    #define ADS1015_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default)
    #define ADS1015_REG_CONFIG_CLAT_LATCH   (0x0004)  // Latching comparator

    #define ADS1015_REG_CONFIG_CQUE_MASK    (0x0003)
    #define ADS1015_REG_CONFIG_CQUE_1CONV   (0x0000)  // Assert ALERT/RDY after one conversions
    #define ADS1015_REG_CONFIG_CQUE_2CONV   (0x0001)  // Assert ALERT/RDY after two conversions
    #define ADS1015_REG_CONFIG_CQUE_4CONV   (0x0002)  // Assert ALERT/RDY after four conversions
    #define ADS1015_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)
/*=========================================================================*/

typedef enum
{
  GAIN_TWOTHIRDS    = ADS1015_REG_CONFIG_PGA_6_144V,
  GAIN_ONE          = ADS1015_REG_CONFIG_PGA_4_096V,
  GAIN_TWO          = ADS1015_REG_CONFIG_PGA_2_048V,
  GAIN_FOUR         = ADS1015_REG_CONFIG_PGA_1_024V,
  GAIN_EIGHT        = ADS1015_REG_CONFIG_PGA_0_512V,
  GAIN_SIXTEEN      = ADS1015_REG_CONFIG_PGA_0_256V
} adsGain_t;

void Bmp180::Init()
{
  double c3,c4,b1;
  
  // The BMP180 includes factory calibration data stored on the device.
  // Each device has different numbers, these must be retrieved and
  // used in the calculations when taking pressure measurements.

  // Retrieve calibration data from device:
  
  if (readInt(0xAA,AC1) &&
    readInt(0xAC,AC2) &&
    readInt(0xAE,AC3) &&
    readUInt(0xB0,AC4) &&
    readUInt(0xB2,AC5) &&
    readUInt(0xB4,AC6) &&
    readInt(0xB6,VB1) &&
    readInt(0xB8,VB2) &&
    readInt(0xBA,MB) &&
    readInt(0xBC,MC) &&
    readInt(0xBE,MD))
  {

    // Compute floating-point polynominals:

    c3 = 160.0 * pow(2,-15) * AC3;
    c4 = pow(10,-3) * pow(2,-15) * AC4;
    b1 = pow(160,2) * pow(2,-30) * VB1;
    c5 = (pow(2,-15) / 160) * AC5;
    c6 = AC6;
    mc = (pow(2,11) / pow(160,2)) * MC;
    md = MD / 160.0;
    x0 = AC1;
    x1 = 160.0 * pow(2,-13) * AC2;
    x2 = pow(160,2) * pow(2,-25) * VB2;
    my0 = c4 * pow(2,15);
    my1 = c4 * c3;
    my2 = c4 * b1;
    p0 = (3791.0 - 8.0) / 1600.0;
    p1 = 1.0 - 7357.0 * pow(2,-20);
    p2 = 3038.0 * 100.0 * pow(2,-36);

    // Success!
  }
  else
  {
    // Error reading calibration data; bad component or connection?
  }
}

char Bmp180::readBytesBmp180(unsigned char *values, char length)
// Read an array of bytes from device
// values: external array to hold data. Put starting register in values[0].
// length: number of bytes to read
{
  char x;

  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(values[0]);
  _error = Wire.endTransmission();
  if (_error == 0)
  {
    Wire.requestFrom(BMP180_ADDR,length);
    while(Wire.available() != length) ; // wait until bytes are ready
    for(x=0;x<length;x++)
    {
      values[x] = Wire.read();
    }
    return(1);
  }
  return(0);
}


char Bmp180::writeBytes(unsigned char *values, char length)
// Write an array of bytes to device
// values: external array of data to write. Put starting register in values[0].
// length: number of bytes to write
{
  char x;
  
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(values,length);
  _error = Wire.endTransmission();
  if (_error == 0)
    return(1);
  else
    return(0);
}


char Bmp180::readInt(char address, int &value)
// Read a signed integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
  unsigned char data[2];

  data[0] = address;
  if (readBytesBmp180(data,2))
  {
    value = (((int)data[0]<<8)|(int)data[1]);
    if (value & 0x8000) value |= 0xFFFF0000; // sign extend if negative
    return(1);
  }
  value = 0;
  return(0);
}


char Bmp180::readUInt(char address, unsigned int &value)
// Read an unsigned integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
  unsigned char data[2];

  data[0] = address;
  if (readBytesBmp180(data,2))
  {
    value = (((unsigned int)data[0]<<8)|(unsigned int)data[1]);
    return(1);
  }
  value = 0;
  return(0);
}



char Bmp180::StartTemperature(void)
// Begin a temperature reading.
// Will return delay in ms to wait, or 0 if I2C error
{
  unsigned char data[2], result;
  
  data[0] = BMP180_REG_CONTROL;
  data[1] = BMP180_COMMAND_TEMPERATURE;
  result = writeBytes(data, 2);
  if (result) // good write?
    return(5); // return the delay in ms (rounded up) to wait before retrieving data
  else
    return(0); // or return 0 if there was a problem communicating with the BMP
}


char Bmp180::GetTemperature(double &T)
// Retrieve a previously-started temperature reading.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startTemperature() to have been called prior and sufficient time elapsed.
// T: external variable to hold result.
// Returns 1 if successful, 0 if I2C error.
{
  unsigned char data[2];
  char result;
  double tu, a;
  
  data[0] = BMP180_REG_RESULT;

  result = readBytesBmp180(data, 2);
  if (result) // good read, calculate temperature
  {
    tu = (data[0] * 256.0) + data[1];
    a = c5 * (tu - c6);
    T = a + (mc / (a + md));
  }
  return(result);
}


char Bmp180::StartPressure(char oversampling)
// Begin a pressure reading.
// Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
// Will return delay in ms to wait, or 0 if I2C error.
{
  unsigned char data[2], result, delay;
  
  data[0] = BMP180_REG_CONTROL;

  switch (oversampling)
  {
    case 0:
      data[1] = BMP180_COMMAND_PRESSURE0;
      delay = 5;
    break;
    case 1:
      data[1] = BMP180_COMMAND_PRESSURE1;
      delay = 8;
    break;
    case 2:
      data[1] = BMP180_COMMAND_PRESSURE2;
      delay = 14;
    break;
    case 3:
      data[1] = BMP180_COMMAND_PRESSURE3;
      delay = 26;
    break;
    default:
      data[1] = BMP180_COMMAND_PRESSURE0;
      delay = 5;
    break;
  }
  result = writeBytes(data, 2);
  if (result) // good write?
    return(delay); // return the delay in ms (rounded up) to wait before retrieving data
  else
    return(0); // or return 0 if there was a problem communicating with the BMP
}


char Bmp180::GetPressure(double &P, double &T)
// Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startPressure() to have been called prior and sufficient time elapsed.
// Requires recent temperature reading to accurately calculate pressure.

// P: external variable to hold pressure.
// T: previously-calculated temperature.
// Returns 1 for success, 0 for I2C error.
{
  unsigned char data[3];
  char result;
  double pu,s,x,y,z;
  
  data[0] = BMP180_REG_RESULT;

  result = readBytesBmp180(data, 3);
  if (result) // good read, calculate pressure
  {
    pu = (data[0] * 256.0) + data[1] + (data[2]/256.0);

    s = T - 25.0;
    x = (x2 * pow(s,2)) + (x1 * s) + x0;
    y = (my2 * pow(s,2)) + (my1 * s) + my0;
    z = (pu - x) / y;
    P = (p2 * pow(z,2)) + (p1 * z) + p0;
  }
  return(result);
}
