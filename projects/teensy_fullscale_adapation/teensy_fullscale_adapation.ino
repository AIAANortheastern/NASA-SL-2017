
/* Wiring:
  UNO     LIS331     SD Breakout    XBee    GPS
  5.0V    ---        <5V            ---     ---
  3.3V    VCC        ---            ---     ---
  GND     GND        GND            ---     ---
  9   	  ---        <CS            ---     ---
  10      CS         ---            ---     ---
  11      SDA/SDI    <DI            ---     ---
  12      SA0/SDO    >DO            ---     ---
  13      SCL/SPC    <CLK           ---     ---
  0       ---        ---            RX      ---
  1       ---        ---            TX      ---
  31      ---        ---            ---     RX
  32      ---        ---            ---     TX
*/

#include <SPI.h>
#include <stdlib.h>
#include <stdio.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

#define SSD    9   // Serial Select     -> CS   SD Breakout
#define SS    10   // Serial Select 	  -> CS 	on LIS331
#define MOSI  11   // MasterOutSlaveIn 	-> SDI  on Both
#define MISO  12   // MasterInSlaveOut 	-> SDO  on Both
#define SCK   13   // Serial Clock 		  -> SPC 	on Both

// 'true' if you want to debug and listen to raw GPS sentances, else false
#define GPSECHO  true

#define SCALE 0.0007324; // approximate scale factor for full range (+/-24g)
// scale factor: +/-24g = 48G range. 2^16 bits. 48/65536 = 0.0007324

// global acceleration values
double xAcc, yAcc, zAcc;

// Global File object for SD writing
File logFile;

// Global SoftwareSerial object for Xbee and GPS communication
SoftwareSerial xbee(2,3); // RX, TX
SoftwareSerial gps(5, 6); // TX, RX

// GPS object
Adafruit_GPS GPS(&gps);
boolean usingInterrupt = false; // For GPS only
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup()
{
  // Configure Serial output (for debugging)
  Serial.begin(115200);

  // Configure SD
  SD_SETUP();

  // Configure SPI
  SPI_SETUP();

  // Configure accelerometer
  Accelerometer_SETUP();

  // Configure GPS
  GPS_SETUP();

  // Configure Xbee SoftwareSerial port
  xbee.begin(9600);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  #ifdef UDR0
    if (GPSECHO)
      if (c) UDR0 = c;  
         // writing direct to UDR0 is much much faster than Serial.print 
        // but only one character can be written at a time. 
  #endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();


void loop()
{
  readAccelVal(); // get acc values and put into global variables

  redGPSVal(); // Get GPS data, need to implement

  // outputSerial();

  outputSD();

  outputXbee();
}

// Output values to serial
void outputSerial(){
  Serial.print(xAcc, 1);
  Serial.print(",");
  Serial.print(yAcc, 1);
  Serial.print(",");
  Serial.println(zAcc, 1);
  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4); Serial.print(GPS.lon);
    Serial.print(GPS.latitudeDegrees, 4);
    Serial.print(",");
    Serial.print(GPS.longitudeDegrees, 4);
    Serial.println((int)GPS.satellites);
  }
  else
  {
    Serial.print(0);
    Serial.print(",");
    Serial.println(0);
  }
}

void outputSD(){
  logFile = SD.open("log.csv", FILE_WRITE);

  if (logFile) {
    logFile.print(millis(), 1);
    logFile.print(",");
    logFile.print(xAcc, 1);
    logFile.print(",");
    logFile.print(yAcc, 1);
    logFile.print(",");
    logFile.println(zAcc, 1);
    if (GPS.fix) {
    	logFile.print("Location: ");
    	logFile.print(GPS.latitude, 4); Serial.print(GPS.lat);
    	logFile.print(", ");
    	logFile.print(GPS.longitude, 4); Serial.print(GPS.lon);
    	logFile.print(GPS.latitudeDegrees, 4);
    	logFile.print(",");
    	logFile.print(GPS.longitudeDegrees, 4);
    	logFile.println((int)GPS.satellites);
  	}
  	else
  	{
    	logFile.print(0);
    	logFile.print(",");
    	logFile.println(0);
  	}
    logFile.close();
  }
}



// Output to Xbee
void outputXbee(){
  xbee.print(millis(), DEC);
  xbee.print(",");
  xbee.print(xAcc, DEC);
  xbee.print(",");
  xbee.print(yAcc, DEC);
  xbee.print(",");
  xbee.println(zAcc, DEC);
  if (GPS.fix) {
    xbee.print("Location: ");
    xbee.print(GPS.latitude, 4); Serial.print(GPS.lat);
    xbee.print(", ");
    xbee.print(GPS.longitude, 4); Serial.print(GPS.lon);
    xbee.print(GPS.latitudeDegrees, 4);
    xbee.print(",");
    xbee.print(GPS.longitudeDegrees, 4);
    xbee.println((int)GPS.satellites);
  }
  else
  {
    xbee.print(0);
    xbee.print(",");
    xbee.println(0);
  }
}

// Read the accelerometer data and put values into global variables
void readAccelVal()
{
  byte xAddressByteL = 0x28; // Low Byte of X value (the first data register)
  byte readBit = B10000000; // bit 0 (MSB) HIGH means read register
  byte incrementBit = B01000000; // bit 1 HIGH means keep incrementing registers
  // this allows us to keep reading the data registers by pushing an empty byte
  byte dataByte = xAddressByteL | readBit | incrementBit;
  byte b0 = 0x0; // an empty byte, to increment to subsequent registers

  digitalWrite(SS, LOW); // SS must be LOW to communicate
  //  delay(1);
  SPI.transfer(dataByte); // request a read, starting at X low byte
  byte xL = SPI.transfer(b0); // get the low byte of X data
  byte xH = SPI.transfer(b0); // get the high byte of X data
  byte yL = SPI.transfer(b0); // get the low byte of Y data
  byte yH = SPI.transfer(b0); // get the high byte of Y data
  byte zL = SPI.transfer(b0); // get the low byte of Z data
  byte zH = SPI.transfer(b0); // get the high byte of Z data
  //  delay(1);
  digitalWrite(SS, HIGH);

  // shift the high byte left 8 bits and merge the high and low
  int xVal = (xL | (xH << 8));
  int yVal = (yL | (yH << 8));
  int zVal = (zL | (zH << 8));

  // scale the values into G's
  xAcc = xVal * SCALE;
  yAcc = yVal * SCALE;
  zAcc = zVal * SCALE;

  if (xAcc > 24){
    xAcc = xAcc - 48;
  }
  if (yAcc > 24){
    yAcc = yAcc - 48;
  }
  if (zAcc > 24){
    zAcc = zAcc - 48;
  }
}

void SPI_SETUP()
{
  pinMode(SS, OUTPUT);

  // wake up the SPI bus
  SPI.begin();

  // This device reads MSB first:
  SPI.setBitOrder(MSBFIRST);

  /*
    SPI.setDataMode()
    Mode Clock Polarity (CPOL) Clock Phase (CPHA)
    SPI_MODE0 0 0
    SPI_MODE1 0 1
    SPI_MODE2 1 0
    SPI_MODE3 1 1
  */
  SPI.setDataMode(SPI_MODE0);

  /*
    SPI.setClockDivider()
    sets SPI clock to a fraction of the system clock
    Arduino UNO system clock = 16 MHz
    Mode SPI Clock
    SPI_CLOCK_DIV2 8 MHz
    SPI_CLOCK_DIV4 4 MHz
    SPI_CLOCK_DIV8 2 MHz
    SPI_CLOCK_DIV16 1 MHz
    SPI_CLOCK_DIV32 500 Hz
    SPI_CLOCK_DIV64 250 Hz
    SPI_CLOCK_DIV128 125 Hz
  */

  SPI.setClockDivider(SPI_CLOCK_DIV16); // SPI clock 1000Hz
}

void Accelerometer_SETUP()
{
  // Set up the accelerometer
  // write to Control register 1: address 20h
  byte addressByte = 0x20;
  /* Bits:
    PM2 PM1 PM0 DR1 DR0 Zen Yen Xen
    PM2PM1PM0: Power mode (001 = Normal Mode)
    DR1DR0: Data rate (00=50Hz, 01=100Hz, 10=400Hz, 11=1000Hz)
    Zen, Yen, Xen: Z enable, Y enable, X enable
  */
  byte ctrlRegByte = 0x37; // 00111111 : normal mode, 1000Hz, xyz enabled

  // Send the data for Control Register 1
  digitalWrite(SS, LOW);
  delay(1);
  SPI.transfer(addressByte);
  SPI.transfer(ctrlRegByte);
  delay(1);
  digitalWrite(SS, HIGH);

  delay(100);

  // write to Control Register 2: address 21h
  addressByte = 0x21;
  // This register configures high pass filter
  ctrlRegByte = 0x00; // High pass filter off

  // Send the data for Control Register 2
  digitalWrite(SS, LOW);
  delay(1);
  SPI.transfer(addressByte);
  SPI.transfer(ctrlRegByte);
  delay(1);
  digitalWrite(SS, HIGH);

  delay(100);

  // Control Register 3 configures Interrupts
  // Since I'm not using Interrupts, I'll leave it alone

  // write to Control Register 4: address 23h
  addressByte = 0x23;
  /* Bits:
    BDU BLE FS1 FS0 STsign 0 ST SIM
    BDU: Block data update (0=continuous update)
    BLE: Big/little endian data (0=accel data LSB at LOW address)
    FS1FS0: Full-scale selection (00 = +/-6G, 01 = +/-12G, 11 = +/-24G)
    STsign: selft-test sign (default 0=plus)
    ST: self-test enable (default 0=disabled)
    SIM: SPI mode selection(default 0=4 wire interface, 1=3 wire interface)
  */
  ctrlRegByte = 0x30; // 00110000 : 24G (full scale)

  // Send the data for Control Register 4
  digitalWrite(SS, LOW);
  delay(1);
  SPI.transfer(addressByte);
  SPI.transfer(ctrlRegByte);
  delay(1);
  digitalWrite(SS, HIGH);
}

void SD_SETUP() {
  // Configure SD Breakout on SSD pin
  pinMode(9, OUTPUT);

  // Switch SPI to SD card
  //digitalWrite(SSD, LOW);
  
  if (!SD.begin(9)) {
    Serial.println("SD Card Initialization Failed!");
    return;
  }

  // Sketchy as hell way to get random file name via live input
  // delay((int) (analogRead(1) * 3 * ((analogRead(2))/512)));

  logFile = SD.open("log.csv", FILE_WRITE);

  if (logFile) {
    logFile.println("millis,x,y,z");
    logFile.close();
  }

  digitalWrite(SSD, HIGH);
}

void GPS_SETUP() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  gps.println(PMTK_Q_RELEASE);
}