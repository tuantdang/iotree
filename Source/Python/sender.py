import serial
import time
from datetime import datetime

s = serial.Serial(port='COM6', baudrate=9600, timeout=None)
arr = [];
for i in range(100):
    arr.append(i%256);
    
#print(arr);
print(s.write(arr));
#while 1:
time.sleep(5)
s.close();
    
