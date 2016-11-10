#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <SPI.h>
#include <SD.h>

/*****************************************************************************************
* This routine is designed to collect various sensor outputs from a 10DOF sensor
* integrated into the subscale. This is accomplished by collecting outputs from each
* available sensor at 100 ms intervals, and outputting the data line my line in a CSV
* format compliant file. Ideally output should be importable into any program that handles
* CSV Files.
******************************************************************************************/

/*
  The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 10
 */

const int sdCardCS = 10;

// Generate Random File Name for log file
// Need to adjust to generate sequential files later

String current_csv_name = (String(random(1000000)) + ".csv");

String current_initlog_name = "Sensor Information.txt";

File csvFile;

File logFile;

void setup() {
	// Initialize SD card (if present), exit if it is not
	if (!SD.begin(chipSelect)) {
    	return;
  	}

  	logFile = SD.open(current_initlog_name, FILE_WRITE);

  	// Test Output, Will be adjusted
  	if (logFile){
  		displaySensorDetails();
  		logFile.close();
  	}

}

void loop() {
	File csvFile = SD.open(current_csv_name, FILE_WRITE);

	// Test Output, Will be adjusted to collum headers
  	if (csvFile){
  		csvFile.println(genSensorOutput())
  		csvFile.close();
  	}
  	
  	(delay 100);	
}

String genSensorOutput(){
	/* Get a new sensor event */
  sensors_event_t event;
  String output = millis() + ",";
   
  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&event);
  output += String(event.acceleration.x) + ",";
  output += String((event.acceleration.y) + ",";
  output += String((event.acceleration.z) + ",";

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  mag.getEvent(&event);
  
  output += String((event.magnetic.x) + ",";
  output += String((event.magnetic.y) + ",";
  output += String((event.magnetic.z) + ",";

  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  output += String((event.gyro.x) + ",";
  output += String((event.gyro.y) + ",";
  output += String((event.gyro.z) + ",";

  /* Display the pressure sensor results (barometric pressure is measure in hPa) */
  bmp.getEvent(&event);
  if (event.pressure)
  {
  	/* Display atmospheric pressure in hPa */
    output += String((event.pressure) + ",";
    /* Display ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    output += String((temperature) + ",";

    /* Then convert the atmospheric pressure, SLP and temp to altitude    */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    output += String((bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure,
                                        temperature)); 
  }
  else{
  	output += ",,";
  }

  return output

}


// Displays Data about sensors in setup
void displaySensorDetails()
{
  sensor_t sensor;
  
  accel.getSensor(&sensor);
  logFile.println(F("----------- ACCELEROMETER ----------"));
  logFile.print  (F("Sensor:       ")); logFile.println(sensor.name);
  logFile.print  (F("Driver Ver:   ")); logFile.println(sensor.version);
  logFile.print  (F("Unique ID:    ")); logFile.println(sensor.sensor_id);
  logFile.print  (F("Max Value:    ")); logFile.print(sensor.max_value); logFile.println(F(" m/s^2"));
  logFile.print  (F("Min Value:    ")); logFile.print(sensor.min_value); logFile.println(F(" m/s^2"));
  logFile.print  (F("Resolution:   ")); logFile.print(sensor.resolution); logFile.println(F(" m/s^2"));
  logFile.println(F("------------------------------------"));
  logFile.println(F(""));

  gyro.getSensor(&sensor);
  logFile.println(F("------------- GYROSCOPE -----------"));
  logFile.print  (F("Sensor:       ")); logFile.println(sensor.name);
  logFile.print  (F("Driver Ver:   ")); logFile.println(sensor.version);
  logFile.print  (F("Unique ID:    ")); logFile.println(sensor.sensor_id);
  logFile.print  (F("Max Value:    ")); logFile.print(sensor.max_value); logFile.println(F(" rad/s"));
  logFile.print  (F("Min Value:    ")); logFile.print(sensor.min_value); logFile.println(F(" rad/s"));
  logFile.print  (F("Resolution:   ")); logFile.print(sensor.resolution); logFile.println(F(" rad/s"));
  logFile.println(F("------------------------------------"));
  logFile.println(F(""));
  
  mag.getSensor(&sensor);
  logFile.println(F("----------- MAGNETOMETER -----------"));
  logFile.print  (F("Sensor:       ")); logFile.println(sensor.name);
  logFile.print  (F("Driver Ver:   ")); logFile.println(sensor.version);
  logFile.print  (F("Unique ID:    ")); logFile.println(sensor.sensor_id);
  logFile.print  (F("Max Value:    ")); logFile.print(sensor.max_value); logFile.println(F(" uT"));
  logFile.print  (F("Min Value:    ")); logFile.print(sensor.min_value); logFile.println(F(" uT"));
  logFile.print  (F("Resolution:   ")); logFile.print(sensor.resolution); logFile.println(F(" uT"));  
  logFile.println(F("------------------------------------"));
  logFile.println(F(""));

  bmp.getSensor(&sensor);
  logFile.println(F("-------- PRESSURE/ALTITUDE ---------"));
  logFile.print  (F("Sensor:       ")); logFile.println(sensor.name);
  logFile.print  (F("Driver Ver:   ")); logFile.println(sensor.version);
  logFile.print  (F("Unique ID:    ")); logFile.println(sensor.sensor_id);
  logFile.print  (F("Max Value:    ")); logFile.print(sensor.max_value); logFile.println(F(" hPa"));
  logFile.print  (F("Min Value:    ")); logFile.print(sensor.min_value); logFile.println(F(" hPa"));
  logFile.print  (F("Resolution:   ")); logFile.print(sensor.resolution); logFile.println(F(" hPa"));  
  logFile.println(F("------------------------------------"));
  logFile.println(F(""));
  
  delay(500);
}


