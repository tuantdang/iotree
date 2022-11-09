import serial
from datetime import datetime

s = serial.Serial(port='COM19', baudrate=9600, timeout=None)
count = 0
while count < 64:    
    try:
        res = s.read(1); 
        while 1:
            res = s.read(1); 
            if res[0] == 0xA:
                break;
        str = "";
        while 1:
            res = s.read(1); 
            if res[0] == 0xD:
                break
            else:
                #print(" {:02X}".format(res[0]))
                str += " {:02X}".format(res[0]);
                count = count + 1;

        #print("Counter = {:03d}. Value: {:03d}".format(count, res[0]))
        print(str);    
        if count >= 64:
            break;
    except Exception as ex:
        print(ex)            
s.close()

