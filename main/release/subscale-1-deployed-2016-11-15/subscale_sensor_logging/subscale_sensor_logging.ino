#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <SPI.h>
#include <SD.h>

/******************************************************************************************
* This routine is designed to collect various sensor outputs from a 10DOF sensor           
* integrated into the subscale. This is accomplished by collecting outputs from each
* available sensor at 100 ms intervals, and outputting the data line my line in a CSV
* format compliant file. Ideally output should be importable into any program that handles
* CSV Files.
******************************************************************************************/

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

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

bool firstFlag = true;
String current_csv_name;

void setup() {
    randomSeed((analogRead(3)) * (analogRead(4)) - (analogRead(5)));
    current_csv_name = String(random(0,100000)) + ".csv";
    
    Serial.begin(115200);

    /*while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }*/

    if (!SD.begin(sdCardCS)) {
        Serial.println("failed!");
        //return;
    }
    Serial.println("done.");

    /* Initialise the sensors */
    if(!accel.begin())
    {
        /* There was a problem detecting the ADXL345 ... check your connections */
        //Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
        while(1);
    }
    if(!mag.begin())
    {
        /* There was a problem detecting the LSM303 ... check your connections */
        //Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while(1);
    }
    if(!bmp.begin())
    {
        /* There was a problem detecting the BMP085 ... check your connections */
        //Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    if(!gyro.begin())
    {
        /* There was a problem detecting the L3GD20 ... check your connections */
        //Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    
    /* Assign a name to the csv file being exported to the SD card, making note to avoid
       overwriting existing data entries 

    String outname = "subscale_launch_1_results_trial_";
    int trialNum = 1;

    while(true) {
        if(SD.exists(outname + String(trialNum) + ".csv")) {
            trialNum++;
        }
        else {
            current_csv_name = outname + String(trialNum) + ".csv";
            break;
        }
    } */

    /* Add headers to the first line of the file so a user knows what is in each collum 
    File csvFile = SD.open(current_csv_name, FILE_WRITE);

    if(csvFile){
      csvFile.println("millis From Start, Accel X (m/s^2), Accel Y (m/s^2), Accel Z (m/s^2), 
              Mag Vector X (uT), Mag Vector Y (uT), Mag Vector Z (uT), 
              Gyro X (rad/s), Gyro Y (rad/s), Gyro Z (rad/s), 
              Atmospheric Pressure (hPa), Temp (C), Estimated altitude (m)");
      csvFile.close();
    }*/
    
}

void loop() {
    File csvFile = SD.open(current_csv_name, FILE_WRITE);

    String outline = genSensorOutput();

    if (csvFile) {
        csvFile.println(outline);
        csvFile.close();
    }

    //Serial.println(current_csv_name);
    Serial.println(outline);

    delay(100);
}

// REFORMAT

String genSensorOutput() {
    /* Get a new sensor event */
    sensors_event_t event;
    String output = String(millis()) + ",";

    /* Display the results (acceleration is measured in m/s^2) */
    accel.getEvent(&event);
    output += String(event.acceleration.x) + ",";
    output += String(event.acceleration.y) + ",";
    output += String(event.acceleration.z) + ",";

    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    mag.getEvent(&event);

    output += String(event.magnetic.x) + ",";
    output += String(event.magnetic.y) + ",";
    output += String(event.magnetic.z) + ",";

    /* Display the results (gyrocope values in rad/s) */
    gyro.getEvent(&event);
    output += String(event.gyro.x) + ",";
    output += String(event.gyro.y) + ",";
    output += String(event.gyro.z) + ",";

    /* Display the pressure sensor results (barometric pressure is measure in hPa) */
    bmp.getEvent(&event);
    if (event.pressure)
    {
        /* Display atmospheric pressure in hPa */
        output += String(event.pressure) + ",";
        /* Display ambient temperature in C */
        float temperature;
        bmp.getTemperature(&temperature);
        output += String(temperature) + ",";

        /* Then convert the atmospheric pressure, SLP and temp to altitude    */
        /* Update this next line with the current SLP for better results      */
        float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
        output += String(bmp.pressureToAltitude(seaLevelPressure,
                                                event.pressure,
                                                temperature));
    }
    else {
        output += ",,";
    }

    return output;
}


