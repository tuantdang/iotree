import matplotlib.pyplot as plt
import numpy as np
import serial
from os import system

ser = serial.Serial(port='COM4',baudrate=9600,timeout=None)

fig=plt.figure()
i=0
x=list()
y=list()
while i <60:
    data_raw = ser.readline()        
    #print(data_raw[1])
    res = data_raw[0] + data_raw[1]*256
    print(res)
            
    x.append(i);
    y.append(res);
    plt.plot(x,y);    
    plt.pause(0.1) #Note this correction
    i+=1;        
plt.show()        
