import serial
import json
import time

with serial.Serial('/dev/ttyS0', 115200) as ser:
    ser.dtr = False
    time.sleep(1)
    ser.flushInput()
    ser.dtr = True
    for i in range(1000):
        ser_out = json.dumps({'light': i % 2 == 0})
        print('Sending:', ser_out)
        ser.write((ser_out+'\n').encode())
        print('Receiving:', ser.readline().decode().strip())
        time.sleep(1)

#TODO: po chvili spadne s IO errorom, 115200 aj 57600
