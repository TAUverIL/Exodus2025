import serial
import time

ser = serial.Serial('/dev/serial0', 9600, timeout = 1)

time.sleep(2)

message = "telecom are the rulers"

if ser.is_open:
    ser.write(message.encode('utf-8'))
    time.sleep(0.1)
    ser.close()
    
ser.close()