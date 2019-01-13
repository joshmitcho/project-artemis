import serial
import time
import struct
import random

count = -32768
successes = 0
failures = 0
ser = serial.Serial('COM3', 9600, timeout = 3)
print(ser.name)
time.sleep(2)
while count < -32512:
    y = count
    ser.write(struct.pack('>bhhhhb',36,y,y,y,y,47))
    s = ser.read(1)
    if (ord(s) == 1):
        successes += 1
    else:
        #print count
        #print ord(s)
        failures += 1
    count+=1

print "number of successes"
print successes
print "number of failures"
print failures
ser.close()
