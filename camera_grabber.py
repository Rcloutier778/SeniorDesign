import sys
sys.path.insert(0, u'/…/…/…/python2.7/site-packages')
import serial

COMMPORT="COM111"
ser = serial.Serial(COMMPORT, 2000000, timeout=2, xonxoff=False, rtscts=False, dsrdtr=False) #Tried with and without the last 3 parameters, and also at 1Mbps, same happens.
ser.flushInput()
ser.flushOutput()
while(true):
    bytesToRead = ser.inWaiting()
    ser.read(bytesToRead)
