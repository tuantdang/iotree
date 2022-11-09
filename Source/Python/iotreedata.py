import serial
from datetime import datetime
s = serial.Serial(port='COM8', baudrate=9600, timeout=None)

logfileName = "test.csv"
fid = open(logfileName, "w")
count = 0
print("Start logging...");
#while count < N:    
line = 0
N = 64
while 1:
    try:        
        text = str(datetime.now())+ ","
        i = 0;
        while i < N:
            res = s.read(1);                           
            #print(i, " : ", res[0])
            text += " {:02X}".format(res[0])            
            i += 1
        fid = open(logfileName, "a")
        fid.write(text  + "\n")
        fid.close()
        line += 1        
        print(line, ": ", text)
        count = count + 1
    except Exception as ex:
        print(ex)        
        break    
s.close()

