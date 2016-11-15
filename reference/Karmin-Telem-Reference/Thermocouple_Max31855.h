// Thermocouple_Max31855.h

#ifndef _THERMOCOUPLE_MAX31855_h
#define _THERMOCOUPLE_MAX31855_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <SPI.h>

class Thermocouple_Max31855
{
private:
	uint8_t chipselect;
	uint32_t spiread32(void);
	SPISettings mySettings;

public:
	Thermocouple_Max31855();
	Thermocouple_Max31855(uint8_t select, SPISettings mySettings);
	void getTemperature(float &dataOut);
	
};

#endif

