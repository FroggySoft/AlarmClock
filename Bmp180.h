#ifndef BMP180_H
#define BMP180_H

class Bmp180
{
public:
	void Init();

	// Begin a temperature reading.
	// Will return delay in ms to wait, or 0 if I2C error
	char StartTemperature(void);

	// Retrieve a previously-started temperature reading.
	// Requires begin() to be called once prior to retrieve calibration parameters.
	// Requires startTemperature() to have been called prior and sufficient time elapsed.
	// T: external variable to hold result.
	// Returns 1 if successful, 0 if I2C error.
	char GetTemperature(double &T);

	// Begin a pressure reading.
	// Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
	// Will return delay in ms to wait, or 0 if I2C error.
	char StartPressure(char oversampling);

	// Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
	// Requires begin() to be called once prior to retrieve calibration parameters.
	// Requires startPressure() to have been called prior and sufficient time elapsed.
	// Requires recent temperature reading to accurately calculate pressure.

	// P: external variable to hold pressure.
	// T: previously-calculated temperature.
	// Returns 1 for success, 0 for I2C error.
	char GetPressure(double &P, double &T);

private:
	int AC1,AC2,AC3,VB1,VB2,MB,MC,MD;
	unsigned int AC4,AC5,AC6; 
	double c5,c6,mc,md,x0,x1,x2,my0,my1,my2,p0,p1,p2;
	char _error;

	char readBytesBmp180(unsigned char *values, char length);
	char writeBytes(unsigned char *values, char length);
	char readUInt(char address, unsigned int &value);
	char readInt(char address, int &value);
};

#endif