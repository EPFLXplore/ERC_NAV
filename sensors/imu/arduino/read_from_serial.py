import serial
import time


ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
time.sleep(2)


with open("values_from_imu.txt", "w") as file:
    line_count = 0
    max_lines = 2000 
    
    while line_count < max_lines:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            print(line)
            file.write(line + '\n')
            file.flush()
            line_count += 1
        
ser.close()