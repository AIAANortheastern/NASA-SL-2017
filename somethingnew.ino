/*
  Xbee1
  D. Thiebaut
  
  Makes Arduino send 1 character via XBee wireless to another XBee connected
  to a computer via a USB cable. 

  The circuit: 
  * RX is digital pin 2 (connect to TX of XBee)
  * TX is digital pin 3 (connect to RX of XBee)
 
  Based on a sketch created back in the mists of time by Tom Igoe
  itself based on Mikal Hart's example
 
*/

#include <SoftwareSerial.h>

SoftwareSerial xbee(2, 3); // RX, TX
int c = 1;
int  pingPong = 1;

void setup()  {
   Serial.begin(57600);
   Serial.println( "Arduino started sending bytes via XBee" );

   // set the data rate for the SoftwareSerial port
   xbee.begin( 9600 );
}

void loop()  {
  // send character via XBee to other XBee connected to Mac
  // via USB cable
  xbee.print( c );
  
  //--- display the character just sent on console ---
  Serial.println( c );
  
  //--- get the next letter in the alphabet, and reset to ---
  //--- 'A' once we have reached 'Z'. 
  c = c + 1;
  if ( c > 9 ) 
       c = 1;
  
  //--- switch LED on Arduino board every character sent---
  if ( pingPong == 0 )
    digitalWrite(13, LOW);
  else
    digitalWrite(13, HIGH);
  pingPong = 1 - pingPong;
  delay( 1000 );
}


