import serial 
import time 
import pandas
from datetime import datetime

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyS0', baudrate = 115200, timeout=1)
    ser.reset_input_buffer()
    fp = open(f'test_{datetime.now()}.csv', 'a')
    goBoy = False
    
    while True:
    
        try:
          isAvalible = ser.in_waiting > 0
        except:
            pass
        
        if isAvalible:
              line = ser.readline().decode('utf-8').rstrip()
              if goBoy:
              
                
                fp.write(line)
                print(line)
                 
              if 'boy' in line:
                goBoy = True
            

