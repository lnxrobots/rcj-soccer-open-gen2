# import serial
# import json
# import time

# v = ''
# with serial.Serial('COM7', 115200) as ser:
#     time.sleep(5)
#     for i in range(100):
#         ser_out = json.dumps({'light': i % 2 == 0})
#         print('Sending:', ser_out)
#         ser.write((ser_out+'\n').encode())
#         print('Receiving:', ser.readline().decode().strip())
#         time.sleep(1)

# #TODO: na zaciatku divne znaky

import serial
import json
import time

with serial.Serial('COM7', 115200) as ser:
    ser.dtr = False
    time.sleep(2)
    ser.flushInput()
    ser.dtr = True
    ser.reset_input_buffer()
    for i in range(20):
        ser_out = json.dumps({'light': i % 2 == 0})
        print('Sending:', ser_out)
        ser.write((ser_out+'\n').encode())
        print('Receiving:', ser.readline().decode().strip())
        time.sleep(1)

#TODO: po chvili spadne s IO errorom, 115200 aj 57600
