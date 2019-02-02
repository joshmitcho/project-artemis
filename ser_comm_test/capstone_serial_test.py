import serial
import time
import struct
import random

count = 120
successes = 0
failures = 0
ser = serial.Serial('COM3', 9600, timeout = 3)
print(ser.name)
time.sleep(10)
#while count < 10:
y = count
ser.write(struct.pack('>bhhhb',36,10,-20,-30,47))
s = ser.read(1)
#print ord(s)
#if (ord(s) == 1):
#    successes += 1
#else:
#print count
#print ord(s)
#    failures += 1
count+=1
time.sleep(15)
ser.write(struct.pack('>bhhhb',36,10,20,30,47))

#print "number of successes"
#print successes
#print "number of failures"
#print failures
ser.close()
