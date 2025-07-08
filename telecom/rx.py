import serial
import time

lora = serial.Serial(
    port = '/dev/ttyTHS1',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
)

while True:
    data_read = lora.readline()
    if data_read:
        print("rx:", data_read.decode('utf-8', errors = 'ignore').strip())
    else:
        print("searching")
    time.sleep(0.2)
    
    