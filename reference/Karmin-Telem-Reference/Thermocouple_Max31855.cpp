// 
// 
// 

#include "Thermocouple_Max31855.h"

// Default constructor
Thermocouple_Max31855::Thermocouple_Max31855():mySettings()
{
	chipselect = 4;
	//define pin modes
	pinMode(chipselect, OUTPUT);

	digitalWrite(chipselect, HIGH);
}

Thermocouple_Max31855::Thermocouple_Max31855(uint8_t select, SPISettings inputSettings) : mySettings(inputSettings)
{
	chipselect = select;
	//define pin modes
	pinMode(chipselect, OUTPUT);

	digitalWrite(chipselect, HIGH);
}

void Thermocouple_Max31855::getTemperature(float &dataOut)
{
	int32_t v;

	v = spiread32();

	if (v & 0x7) {
		// uh oh, a serious problem!
		dataOut = 0x7FFFFFFFFF;
	}

	if (v & 0x80000000) {
		// Negative value, drop the lower 18 bits and explicitly extend sign bits.
		v = 0xFFFFC000 | ((v >> 18) & 0x00003FFFF);
	}
	else {
		// Positive value, just drop the lower 18 bits.
		v >>= 18;
	}

	float centigrade = (float)v;

	// LSB = 0.25 degrees C
	centigrade *= 0.25;
	dataOut = centigrade;
	return;
}

uint32_t Thermocouple_Max31855::spiread32(void) {
	int i;
	// easy conversion of four uint8_ts to uint32_t
	union bytes_to_uint32 {
		uint8_t bytes[4];
		uint32_t integer;
	} buffer;

	SPI.beginTransaction(mySettings);

	digitalWrite(chipselect, LOW);
	delay(1);

	for (i = 3; i >= 0; i--) {
		buffer.bytes[i] = SPI.transfer(0x00);
	}

	digitalWrite(chipselect, HIGH);
	SPI.endTransaction();

	return buffer.integer;

}
