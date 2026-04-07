import serial
import time

ser = serial.Serial("/dev/ttyAMA3", 115200, timeout=1)

while True:
    ser.write(b"TEST123\r\n")
    #time.sleep(1)
    data = ser.read(32)
    print("RX:", data)