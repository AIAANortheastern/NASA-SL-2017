#from xbee import XBee
import serial
from struct import *
import os.path


PORT = 'COM5'
BAUD_RATE = 57600

# Open serial port
ser = serial.Serial(PORT, BAUD_RATE)

# Create API object
#xbee = XBee(ser)

filename = "RecvData0"
i = 0
while True == os.path.isfile(filename + ".csv") :
      filename = filename[:-1] + str(i)
      i = i + 1

filename += ".csv"

dataFile = open(filename, 'w');

# Continuously read and print packets
while True:
    try:
        #response = xbee.wait_read_frame()
        if(ser.inWaiting() >= 52):
            response = ser.read(52)
            temp1, temp2, temp3, temp10dof, alt_press, alt_strat, alt_gps, lat, lon, accx, accy, accz, flags, pad = unpack('fffffiffffffhh', response)
            print "Temp: ", temp1 , "accel: ", accx
            s = str(temp1) + "," + str(temp2) + "," + str(temp3) + "," + str(temp10dof) + "," + str(alt_press) + "," + str(alt_strat) + "," + str(alt_gps) + "," + str(lat) + "," + str(lon) + "," + str(accx) + "," + str(accy) + "," + str(accz) + "," + str(flags) + "\n"
            dataFile.write(s)
    except KeyboardInterrupt:
        break
        
ser.close()
dataFile.close();
