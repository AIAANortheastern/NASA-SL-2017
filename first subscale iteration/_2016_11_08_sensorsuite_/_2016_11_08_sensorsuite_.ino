#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Time.h>

#include <SPI.h>
#include <SD.h>

 /** SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 10
*/
/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
Adafruit_10DOF                dof   = Adafruit_10DOF();

float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

int chipselect = 10;

File data01;

void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    data01.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    data01.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    data01.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
    displaySensorDetails();
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  
  accel.getSensor(&sensor);
  data01.println(F("----------- ACCELEROMETER ----------"));
  data01.print  (F("Sensor:       ")); data01.println(sensor.name);
  data01.print  (F("Driver Ver:   ")); data01.println(sensor.version);
  data01.print  (F("Unique ID:    ")); data01.println(sensor.sensor_id);
  data01.print  (F("Max Value:    ")); data01.print(sensor.max_value); data01.println(F(" m/s^2"));
  data01.print  (F("Min Value:    ")); data01.print(sensor.min_value); data01.println(F(" m/s^2"));
  data01.print  (F("Resolution:   ")); data01.print(sensor.resolution); data01.println(F(" m/s^2"));
  data01.println(F("------------------------------------"));
  data01.println(F(""));

  gyro.getSensor(&sensor);
  data01.println(F("------------- GYROSCOPE -----------"));
  data01.print  (F("Sensor:       ")); data01.println(sensor.name);
  data01.print  (F("Driver Ver:   ")); data01.println(sensor.version);
  data01.print  (F("Unique ID:    ")); data01.println(sensor.sensor_id);
  data01.print  (F("Max Value:    ")); data01.print(sensor.max_value); data01.println(F(" rad/s"));
  data01.print  (F("Min Value:    ")); data01.print(sensor.min_value); data01.println(F(" rad/s"));
  data01.print  (F("Resolution:   ")); data01.print(sensor.resolution); data01.println(F(" rad/s"));
  data01.println(F("------------------------------------"));
  data01.println(F(""));
  
  mag.getSensor(&sensor);
  data01.println(F("----------- MAGNETOMETER -----------"));
  data01.print  (F("Sensor:       ")); data01.println(sensor.name);
  data01.print  (F("Driver Ver:   ")); data01.println(sensor.version);
  data01.print  (F("Unique ID:    ")); data01.println(sensor.sensor_id);
  data01.print  (F("Max Value:    ")); data01.print(sensor.max_value); data01.println(F(" uT"));
  data01.print  (F("Min Value:    ")); data01.print(sensor.min_value); data01.println(F(" uT"));
  data01.print  (F("Resolution:   ")); data01.print(sensor.resolution); data01.println(F(" uT"));  
  data01.println(F("------------------------------------"));
  data01.println(F(""));

  bmp.getSensor(&sensor);
  data01.println(F("-------- PRESSURE/ALTITUDE ---------"));
  data01.print  (F("Sensor:       ")); data01.println(sensor.name);
  data01.print  (F("Driver Ver:   ")); data01.println(sensor.version);
  data01.print  (F("Unique ID:    ")); data01.println(sensor.sensor_id);
  data01.print  (F("Max Value:    ")); data01.print(sensor.max_value); data01.println(F(" hPa"));
  data01.print  (F("Min Value:    ")); data01.print(sensor.min_value); data01.println(F(" hPa"));
  data01.print  (F("Resolution:   ")); data01.print(sensor.resolution); data01.println(F(" hPa"));  
  data01.println(F("------------------------------------"));
  data01.println(F(""));
  
}

void setup() {
  // put your setup code here, to run once:
  initSensors();

  SD.begin(chipselect);

  data01 = SD.open("data01.txt", FILE_WRITE);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t event;
 //

  getpitchrollheading();
  get_accel_mag_gyro_alt();

  delay(100);

 
}

void getpitchrollheading() {
   sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

  /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  /* Use the new fusionGetOrientation function to merge accel/mag data */  
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    data01.print(F("Orientation: "));
    data01.print(orientation.roll);
    data01.print(F(" "));
    data01.print(orientation.pitch);
    data01.print(F(" "));
    data01.print(orientation.heading);
    data01.println(F(""));
  }

  /* Previous code removed handling accel and mag data separately */
  //  /* Calculate pitch and roll from the raw accelerometer data */
  //  data01.print(F("Orientation: "));
  //  accel.getEvent(&accel_event);
  //  if (dof.accelGetOrientation(&accel_event, &orientation))
  //  {
  //    /* 'orientation' should have valid .roll and .pitch fields */
  //    data01.print(orientation.roll);
  //    data01.print(F(" "));
  //    data01.print(orientation.pitch);
  //    data01.print(F(" "));
  //  }
  //  
  //  /* Calculate the heading using the magnetometer */
  //  mag.getEvent(&mag_event);
  //  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  //  {
  //    /* 'orientation' should have valid .heading data now */
  //    data01.print(orientation.heading);
  //  }
  //  data01.println(F(""));
}
}

void get_accel_mag_gyro_alt() {
 sensors_event_t event;
   
  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&event);
  data01.print(F("ACCEL "));
  data01.print("X: "); data01.print(event.acceleration.x); data01.print("  ");
  data01.print("Y: "); data01.print(event.acceleration.y); data01.print("  ");
  data01.print("Z: "); data01.print(event.acceleration.z); data01.print("  ");data01.println("m/s^2 ");

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  mag.getEvent(&event);
  data01.print(F("MAG   "));
  data01.print("X: "); data01.print(event.magnetic.x); data01.print("  ");
  data01.print("Y: "); data01.print(event.magnetic.y); data01.print("  ");
  data01.print("Z: "); data01.print(event.magnetic.z); data01.print("  ");data01.println("uT");

  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  data01.print(F("GYRO  "));
  data01.print("X: "); data01.print(event.gyro.x); data01.print("  ");
  data01.print("Y: "); data01.print(event.gyro.y); data01.print("  ");
  data01.print("Z: "); data01.print(event.gyro.z); data01.print("  ");data01.println("rad/s ");  

  /* Display the pressure sensor results (barometric pressure is measure in hPa) */
  bmp.getEvent(&event);
  if (event.pressure)
  {
    /* Display atmospheric pressure in hPa */
    data01.print(F("PRESS "));
    data01.print(event.pressure);
    data01.print(F(" hPa, "));
    /* Display ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    data01.print(temperature);
    data01.print(F(" C, "));
    /* Then convert the atmospheric pressure, SLP and temp to altitude    */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    data01.print(bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure,
                                        temperature)); 
    data01.println(F(" m"));
  }
}

/*String get_filename() {
  String filename = "";


  return filename;
}*/
