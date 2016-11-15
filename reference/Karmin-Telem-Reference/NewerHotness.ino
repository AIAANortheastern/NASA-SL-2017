/*
Name:      NewerHotness.ino
Created:   8/2/2016 7:40:04 PM
Author:    Andrew Kaster
*/

#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>
#include "Thermocouple_Max31855.h"
#include <string.h>
#include <stdlib.h>
#include <SPI.h>
#include <SD.h>

#define NUM_BYTES_TO_SEND   (52)

#define XBEE_PORT           (Serial3)
#define XBEE_BAUD_RATE      (57600)
#define XBEE_CONFIG         (SERIAL_8N1)

#define STRAT_PORT          (Serial1)
#define STRAT_BAUD_RATE     (9600)
#define STRAT_CONFIG        (SERIAL_8N1)
#define STRAT_READ_LEN      (7)

#define GPS_PORT            (Serial2)
#define GPS_BAUD_RATE       (9600)
#define GPS_CONFIG          (SERIAL_8N1)

//Acceleromter conversion from ADC values to g's
#define SF_ADXLX            (2.0/5.0)
#define SF_ADXLY            (2.0/7.0)
#define SF_ADXLZ            (2.0/7.0)
#define ZERO_ADXLX          (509.5)
#define ZERO_ADXLY          (511.5)
#define ZERO_ADXLZ          (510.5)


/** PIN DEFINES */
#define CS_TCA1             (5)
#define CS_TCA2             (6)
#define CS_TCA3             (6)
#define XBEE_CTS_PIN        (4)
#define RX_STRAT            (7) // RX_3
#define RX_GPS              (9) // RX_2
#define TX_GPS              (10)// TX_2
#define CS_SDCARD           (15)
#define Z_ADXL              (22)
#define Y_ADXL              (21)
#define X_ADXL              (20)

/** POLL FLAG DEFINES **/
#define TEMP1_BITMASK       (0x8000)
#define TEMP2_BITMASK       (0x4000)
#define TEMP3_BITMASK       (0x2000)
#define PRESS_TEMP_BITMASK  (0x1000)
#define ALT_PRESS_BITMASK   (0x0800)
#define ALT_STRAT_BITMASK   (0x0400)
#define ALT_GPS_BITMASK     (0x0200)
#define LAT_BITMASK         (0x0100)
#define LON_BITMASK         (0x0080)
#define ACCEL_X_BITMASK     (0x0040)
#define ACCEL_Y_BITMASK     (0x0020)
#define ACCEL_Z_BITMASK     (0x0010)
#define ACCEL_10DOF_BITMASK (0x0008)
#define MAG_10DOF_BITMASK   (0x0004)
#define GYRO_10DOF_BITMASK  (0x0002)
#define DATA_SENT_BITMASK   (0x0001)

//TODO GOAL: 20Hz for both. Values will need tweaking for sure
#define MILIS_BTWN_SEND     (25) // Should be limited by CTS
#define MILIS_BTWN_WRITE    (50)

/* Uncomment for serial output */
//#define DEBUG_MODE

//OLD INFO below. Double check with diagram before trusting

// XBEE TX_1 = Pin 1        Purple 7 on logic analyzer
// XBEE RX_1 = Pin 0        Blue 6  on logic analyzer

//SPI Clk Pin 13
//SPI DataOut Pin 11
//SPI DataIn Pin 12


//SPI Clock speed max ~1-2 MHz (4 for yolo)

struct tenDOF_data_s
{
    float *pressure;
    sensors_vec_t *accel;
    sensors_vec_t *mag;
    sensors_vec_t *gyro;
    sensors_vec_t orientation;
};

typedef tenDOF_data_s tenDOF_data_t;

struct send_data_s {

    float temp1;
    float temp2;
    float temp3;
    float temp_10dof;
    float alt_press;
    int32_t alt_strat;
    float alt_gps;
    float lat;
    float lon;
    float accel_x;
    float accel_y;
    float accel_z;
    short poll_flags;
    short padding;
};

/*  TEMP1,
TEMP2,
TEMP3,
TEMP_10DOF,
ALT_PRESS,
ALT_STRAT,
ALT_GPS,
LAT,
LON,
ACCEL_X,
ACCEL_Y,
ACCEL_Z,
PRESSURE,
ACCEL_10DOF_X,
ACCEL_10DOF_Y,
ACCEL_10DOF_Z,
MAG_X,
MAG_Y,
MAG_Z,
ROT_X,
ROT_Y,
ROT_Z,
ROLL,
PITCH,
HEADING,*/

typedef union {
    send_data_s component;
    byte        full[NUM_BYTES_TO_SEND];
}send_data_t;

enum karmaSensorEnum
{
    TEMP1 = 0,
    TEMP2,
    //TEMP3,
    TEMP_PRESS_10DOF,
    STRATOLOGGER,
    GPS,
    ANALOG_ACCEL,
    ACCEL_10DOF,
    MAG_10DOF,
    GYRO_10DOF,
    RPY_CALC_10DOF, /* Not really a sensor */
    NUM_KARMAN_SENSORS,
};

typedef struct
{
    tenDOF_data_t write_only;
    send_data_t write_send;
}write_data_t;

//Custom globals for Karman-specific stuff
write_data_t gWriteData;
send_data_t *gSendData = &(gWriteData.write_send);
String write_string;
elapsedMillis sinceSend;
elapsedMillis sinceWrite;
unsigned long currTime;
String dataFileName;
unsigned long ml_index;
float gSeaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA; /*initalize to value defined in Adafruit_Sensor.h*/


SPISettings SPI_TCA1(1000000, MSBFIRST, SPI_MODE0);
SPISettings SPI_TCA2(1000000, MSBFIRST, SPI_MODE0);
//SPISettings SPI_TCA3(1000000, MSBFIRST, SPI_MODE0);


// Call hardware spi constructor for TCA's
Thermocouple_Max31855 thermocouple1(CS_TCA1, SPI_TCA1);
Thermocouple_Max31855 thermocouple2(CS_TCA2, SPI_TCA2);
//Thermocouple_Max31855 thermocouple3(CS_TCA3, SPI_TCA3);

//Call constructors for 10DOF chips
Adafruit_BMP085_Unified         bmp085(1);
Adafruit_L3GD20_Unified         l3gd20(2);
Adafruit_LSM303_Accel_Unified   lsm303_accel(3);
Adafruit_LSM303_Mag_Unified     lsm303_mag(4);
Adafruit_10DOF                  tenDOFObject;

//Adafruit unified sensor library global sensor events
sensors_event_t                pressureBMP085;
sensors_event_t                gyroL3GD20;
sensors_event_t                accelLSM303;
sensors_event_t                magLSM303;

//Constructor for GPS
Adafruit_GPS myGPS(&GPS_PORT);

void setup() {
    /* Power on delay. Should act as debounce*/
    delay(1000);

    gSendData->component.padding = 0;
    analogReference(EXTERNAL); //DO NOT REMOVE UNDER ANY CIRCUMSTANCES
#ifdef DEBUG_MODE
    Serial.begin(115200);
#endif
    //setup sensors and get ready for transmit loop
    //Configure XBEE serial port
    XBEE_PORT.begin(XBEE_BAUD_RATE, XBEE_CONFIG);
    //Configure StratoLogger serial port
    STRAT_PORT.begin(STRAT_BAUD_RATE, STRAT_CONFIG);

    //TODO start GPS_PORT
    GPS_PORT.begin(GPS_BAUD_RATE, GPS_CONFIG);
    myGPS.begin(GPS_BAUD_RATE);

    SPI.begin();

    // setup SD card recording.
    // Note begin() uses the SPI interface
    // to do setup communications with the sd card.
    if (!SD.begin(CS_SDCARD)) {
#ifdef DEBUG_MODE
        Serial.println("Failed to Initialize SD card");
    }
    else
    {
        Serial.println("SD card initialized!");
#endif
    }
    int dataNum = 0;
    dataFileName = "data" + String(dataNum);
    while (SD.exists(dataFileName.c_str()))
    {
        dataNum++;
        dataFileName = "data" + String(dataNum);
        if (dataNum > 1000)
        {
            break;
        }
    }


    File dummyClear = SD.open(dataFileName.c_str(), FILE_WRITE);
    if (dummyClear)
    {
        dummyClear.println("/****************************************/");
        dummyClear.println("Karman Summer Telemetry Newer Hotness Data");
        dummyClear.println("/****************************************/");
        dummyClear.close();
    }

#ifdef DEBUG_MODE
    Serial.println("Header written!");
    Serial.println(dataFileName);
#endif
    //Pressure --> Altitude
    //Also temperature
    bmp085.begin(BMP085_MODE_ULTRALOWPOWER);

#ifdef DEBUG_MODE
    Serial.println("Finish init bmp085");
#endif

    //Rotation rate in Rad/Sec U,V,W
    l3gd20.begin(GYRO_RANGE_250DPS);
    l3gd20.enableAutoRange(true); //This will auto scale our gryo if it's saturating

#ifdef DEBUG_MODE
    Serial.println("Finish init l3gd20");
#endif
                                  //Acceleration X,Y,Z
    lsm303_accel.begin();
    lsm303_accel.enableAutoRange(true); //This will auto scale our accelerometer if it's saturating

#ifdef DEBUG_MODE
    Serial.println("Finish init lsm303 accel");
#endif
                                        //Magnetic field strength X,Y,Z
    lsm303_mag.begin();
    lsm303_mag.enableAutoRange(true); //This will auto scale our magnetometer if it's saturating

                                      //TODO Set sealevel pressure
                                      /* pseudocode:
                                      wait a few seconds
                                      get pressure event
                                      float sealevelPressure = event.pressure;
                                      */

                                      //Assign global pointers into sensors events
    gWriteData.write_only.pressure = &(pressureBMP085.pressure);
    gWriteData.write_only.accel = &(accelLSM303.acceleration);
    gWriteData.write_only.mag = &(magLSM303.magnetic);
    gWriteData.write_only.gyro = &(gyroL3GD20.gyro);

#ifdef DEBUG_MODE
    Serial.println("Preparing to send messages to GPS");
#endif

    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    delay(5);

    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

    delay(5);

    myGPS.sendCommand(PGCMD_ANTENNA);
    
#ifdef DEBUG_MODE
    Serial.println("preparing to wait 1 sec after all init");
#endif

    delay(1000);

    /* set sea level pressure as pressure at launch site */
    bmp085.getEvent(&pressureBMP085);
    gSeaLevelPressure = pressureBMP085.pressure;
}

void loop()
{
    //Serial.print("HELLO WORLD\n");
    //get data and try to send and write after each grab
    for (ml_index = 0; ml_index < NUM_KARMAN_SENSORS; ml_index++)
    {
        //TODO use this return value
        (void)get_karman_data(ml_index);
        send_check();
        write_check();
        while (GPS_PORT.available() > 0)
        {
            char discard = myGPS.read();
#ifdef DEBUG_MODE_EXTRA
            Serial.print(discard);
#endif
        }

    }

    return;
}


void send_check()
{
    if (sinceSend > MILIS_BTWN_SEND && digitalRead(XBEE_CTS_PIN) == LOW)
    {
#ifdef DEBUG_MODE
       // Serial.println("Sending data");
#endif
        XBEE_PORT.write((gSendData->full), NUM_BYTES_TO_SEND);
        sinceSend = 0;
        gSendData->component.poll_flags &= 0; //Clear flags
        gSendData->component.poll_flags |= DATA_SENT_BITMASK;
        /*If any of the poll flags are set and the data sent bitmask is 1,
        *Then we know when we write that the data was written before it was sent.
        */
    }
    return;

}

bool get_karman_data(byte data_number)
{
    bool retVal = false;
    switch (data_number)
    {
    case TEMP1: /* SPI */
        thermocouple1.getTemperature(gSendData->component.temp1);
#ifdef DEBUG_MODE
        Serial.print("Temperature 1");
        Serial.println(gSendData->component.temp1);
#endif
        gSendData->component.poll_flags |= TEMP1_BITMASK;
        retVal = true;
        break;
    case TEMP2: /* SPI */
        thermocouple2.getTemperature(gSendData->component.temp2);
        gSendData->component.poll_flags |= TEMP2_BITMASK;
        retVal = true;
        break;
        //   case TEMP3: /* SPI */
        //       thermocouple3.getTemperature(gSendData->component.temp3);
        //       gSendData->component.poll_flags |= TEMP3_BITMASK;
        //       retVal = true;
        //       break;
    case TEMP_PRESS_10DOF: /* TWI */
                           /*Introduces minimum of 15ms delay. 5ms for each TWI call. Two for getEvent, one for getTemperature*/
        bmp085.getEvent(&pressureBMP085);
        bmp085.getTemperature(&(gSendData->component.temp_10dof));
        gSendData->component.poll_flags |= PRESS_TEMP_BITMASK;
        /* This is not very accurate at high altitude. See http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf page 16*/
        gSendData->component.alt_press = bmp085.pressureToAltitude(gSeaLevelPressure, pressureBMP085.pressure);
        gSendData->component.poll_flags |= ALT_PRESS_BITMASK;
        retVal = true;
        break;
    case STRATOLOGGER: /* Serial */
        gSendData->component.alt_strat = parseStratoLogger();
        gSendData->component.poll_flags |= ALT_STRAT_BITMASK;
        retVal = true;
        break;
    case GPS: /* Serial2 */
        if (myGPS.newNMEAreceived())
        {
#ifdef DEBUG_MODE
            Serial.println("newNMEArecieved");
#endif
            if (!myGPS.parse(myGPS.lastNMEA()))
            {
                break;
            }
        }
        if (myGPS.fix)
        {
            gSendData->component.alt_gps = myGPS.altitude;
            gSendData->component.lat = myGPS.latitudeDegrees;
            gSendData->component.lon = myGPS.longitudeDegrees;
            gSendData->component.poll_flags |= ALT_GPS_BITMASK;
            gSendData->component.poll_flags |= LAT_BITMASK;
            gSendData->component.poll_flags |= LON_BITMASK;
            retVal = true;
        }
        break;
    case ANALOG_ACCEL: /* Analog pins */
        gSendData->component.accel_x = SF_ADXLX * ((float)(analogRead(X_ADXL)) - ZERO_ADXLX);
        gSendData->component.poll_flags |= ACCEL_X_BITMASK;
        gSendData->component.accel_y = SF_ADXLY * ((float)(analogRead(Y_ADXL)) - ZERO_ADXLY);
        gSendData->component.poll_flags |= ACCEL_Y_BITMASK;
        gSendData->component.accel_z = SF_ADXLY * ((float)(analogRead(Z_ADXL)) - ZERO_ADXLZ);
        gSendData->component.poll_flags |= ACCEL_Z_BITMASK;
        retVal = true;
        break;
    case ACCEL_10DOF: /* TWI */
        lsm303_accel.getEvent(&accelLSM303);
        gSendData->component.poll_flags |= ACCEL_10DOF_BITMASK;
        retVal = true;
        break;
    case MAG_10DOF: /* TWI */
        lsm303_mag.getEvent(&magLSM303);
        gSendData->component.poll_flags |= MAG_10DOF_BITMASK;
        retVal = true;
        break;
    case GYRO_10DOF: /* TWI */
        l3gd20.getEvent(&gyroL3GD20);
        gSendData->component.poll_flags |= GYRO_10DOF_BITMASK;
        retVal = true;
        break;
    case RPY_CALC_10DOF: /* Pure calculation */
        tenDOFObject.fusionGetOrientation(&accelLSM303, &magLSM303, &(gWriteData.write_only.orientation));
        /* No poll flag for RPY calculation */
        retVal = true;
        break;
    default:
        /*Error handler...? naaah */
        break;
    }
    return retVal;
}

int32_t parseStratoLogger(void)
{
    int32_t altitude = 0;

    char rawStratBuff[STRAT_READ_LEN + 1];

    if (STRAT_PORT.available() > 2)
    {
        byte numBytesRead = STRAT_PORT.readBytesUntil('\n', rawStratBuff, STRAT_READ_LEN);
        if (rawStratBuff[numBytesRead - 1] == '\r')
        {
            rawStratBuff[numBytesRead - 1] = '\0';
            altitude = atoi(rawStratBuff);
        }
        else if (rawStratBuff[numBytesRead - 1] == '\n')
        {
            rawStratBuff[numBytesRead - 1] = '\0';
            altitude = atoi(rawStratBuff);
        }
    }

    if (altitude > 0)
    {
        /* Did we get good data?*/
        return altitude;
    }
    else
    {
        /* We got 100% garbage :( Just assume we didn't move*/
        return  gSendData->component.alt_strat;
    }
}




// Records all data currently in the global data structure to the SD card
// With a timestamp instead of poll flags
void write_check()
{
    if (sinceWrite > MILIS_BTWN_WRITE)
    {
        //TODO get write-only 10DOF data
        File data_file = SD.open(dataFileName.c_str(), FILE_WRITE);
        if (data_file)
        {
            currTime = millis();
            //Fill write_string with data from gWriteData
            getFormattedWriteString(&write_string);
#ifdef DEBUG_MODE
            Serial.println(String("SD card Data: \n" + write_string));
#endif
            data_file.println(write_string);
            data_file.close();
            write_string = "";
        }
        else
        {
#ifdef DEBUG_MODE
            Serial.println("Error opening data file");
#endif
        }
        sinceWrite = 0;
        /*Clear data sent bit for next write. The info about what was new since last send is already stored
        *If the data sent bitmask wasn't set, then we know we wrote twice in a row before sending.
        */
        gSendData->component.poll_flags &= (~DATA_SENT_BITMASK);
    }
}

void getFormattedWriteString(String *pWriteString)
{
    *pWriteString += String(currTime) + ',';
    *pWriteString += String(gWriteData.write_send.component.temp1) + ',';
    *pWriteString += String(gWriteData.write_send.component.temp2) + ',';
    *pWriteString += String(gWriteData.write_send.component.temp3) + ',';
    *pWriteString += String(gWriteData.write_send.component.temp_10dof) + ',';
    *pWriteString += String(gWriteData.write_send.component.alt_press) + ',';
    *pWriteString += String(gWriteData.write_send.component.alt_strat) + ',';
    *pWriteString += String(gWriteData.write_send.component.alt_gps) + ',';
    *pWriteString += String(gWriteData.write_send.component.lat) + ',';
    *pWriteString += String(gWriteData.write_send.component.lat) + ',';
    *pWriteString += String(gWriteData.write_send.component.accel_x) + ',';
    *pWriteString += String(gWriteData.write_send.component.accel_y) + ',';
    *pWriteString += String(gWriteData.write_send.component.accel_z) + ',';
    *pWriteString += String(gWriteData.write_send.component.poll_flags) + ',';
    *pWriteString += String(*(gWriteData.write_only.pressure)) + ',';
    *pWriteString += String(gWriteData.write_only.orientation.roll) + ',';
    *pWriteString += String(gWriteData.write_only.orientation.pitch) + ',';
    *pWriteString += String(gWriteData.write_only.orientation.heading) + ',';
    *pWriteString += String(gWriteData.write_only.accel->x) + ',';
    *pWriteString += String(gWriteData.write_only.accel->y) + ',';
    *pWriteString += String(gWriteData.write_only.accel->z) + ',';
    *pWriteString += String(gWriteData.write_only.mag->x) + ',';
    *pWriteString += String(gWriteData.write_only.mag->y) + ',';
    *pWriteString += String(gWriteData.write_only.mag->z) + ',';
    *pWriteString += String(gWriteData.write_only.gyro->x) + ',';
    *pWriteString += String(gWriteData.write_only.gyro->y) + ',';
    *pWriteString += String(gWriteData.write_only.gyro->z) + ',';
    return;
}
