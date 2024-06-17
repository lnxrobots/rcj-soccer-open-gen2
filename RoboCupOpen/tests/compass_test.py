# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
#from adafruit_extended_bus import ExtendedI2C as I2C
import serial
import adafruit_bno055

LOOPS = 1000

# i2c = I2C(3)
uart = serial.Serial("/dev/ttyAMA2")
sensor = adafruit_bno055.BNO055_UART(uart)
sensor.mode = adafruit_bno055.IMUPLUS_MODE

# with open('sensor.log', 'w') as file:
start = time.time()
# for i in range(LOOPS):
while True:
    print(sensor.euler)#, file=file)
    #a = sensor.euler[0]
    # time.sleep(0.01)
print((time.time()-start)/LOOPS*1000, 'ms')
