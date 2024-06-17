from soccer_robot.module.module import Module
from soccer_robot.logger.logger_module import LoggerModule
import soccer_robot.constants as con

import multiprocessing as mp
import serial
import adafruit_bno055
import time


class CompassModule(Module):
    """Module for communication with Adafruit BNO055 compass module"""

    module_name = "CompassModule"

    @classmethod
    def init(cls) -> None:
        super().init()
        cls._heading = mp.Value("d", -1.0)
        cls._acceleration = mp.Array("d", [0, 0, 0])

    @classmethod
    def get_heading(cls):
        with cls._heading.get_lock():
            val = cls._heading.value
        return val

    @classmethod
    def set_heading(cls, heading: float):
        with cls._heading.get_lock():
            cls._heading.value = heading

    @classmethod
    def set_acc(cls, acc: float):
        with cls._acceleration.get_lock():
            cls._acceleration[0:3] = acc

    @classmethod
    def get_acc(cls):
        with cls._acceleration.get_lock():
            val = cls._acceleration[0:3]
        return val

    @classmethod
    def on_run(cls, _stop_flag):
        uart = serial.Serial(con.COMPASS_SERIAL_PORT)

        inited = False
        init_count = 0
        while not inited:
            try:
                sensor = adafruit_bno055.BNO055_UART(uart)
            except Exception as e:
                LoggerModule.log_warning("Compass init error: " + str(e))
                init_count += 1
                if init_count > con.COMPASS_INIT_ATTEMPTS:
                    raise e
                time.sleep(con.COMPASS_INIT_DELAY)
            else:
                inited = True

        sensor.mode = adafruit_bno055.IMUPLUS_MODE

        while not _stop_flag.value:
            try:
                heading = sensor.euler[0]
                if heading is not None:
                    cls.set_heading(heading)
                if con.COMPASS_READ_ACC:
                    acc = sensor.acceleration
                    gravity = sensor.gravity
                    if all([a is not None for a in acc+gravity]):
                        cls.set_acc(list(map(lambda a: a[0]-a[1], zip(acc, gravity))))
            except RuntimeError as e:
                if str(e) != "UART read error: 7":
                    raise e

        LoggerModule.log_info(f"Compass init attempts: {init_count+1}")
