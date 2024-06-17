from soccer_robot.module.module import Module
from soccer_robot.logger.logger_module import LoggerModule
from soccer_robot.interface.ui_module.ui_module import UIModule
from soccer_robot.interface.lidar.lidar_module import LidarModule
from soccer_robot.mathf import mathf
from soccer_robot.mathf.vector2 import Movement
from soccer_robot.utils.timer import Timer
import soccer_robot.constants as con

from ctypes import c_bool
import multiprocessing as mp
import serial
import json
import enum
import time
import math

PACKET_SIZE = 152

class UndercarriageModule(Module):
    module_name = "UndercarriageModule"

    @classmethod
    def init(cls) -> None:
        super().init()

        cls._heading = mp.Value("d", -1.0)
        cls._buttons = mp.Array(c_bool, [0] * 3)

        cls._button_one_status = mp.Value("b", True)
        cls._button_one_last = False

        cls._to_terminate = mp.Value("b", False)
        cls._to_terminate_timer = Timer()

        cls._motor_values = mp.Array("i", [0] * 4)
        cls._color_sensor_values = mp.Array("i", [0] * 16)
        cls._color_sensor_values_change = mp.Value("i", 1)

        cls._motor_message_interval = mp.Value("d", 0.006)

        cls._dribbler_speed = mp.Value("i", 0)
        cls._kicker_state = mp.Value("b", False)
        cls._to_kick = mp.Value("b", False)
        cls._kick_timer = Timer()

    @classmethod
    def on_start(cls) -> None:
        cls.serial_connection = serial.Serial(con.UNDERCARRIAGE_SERIAL_PORT, con.UNDERCARRIAGE_BOUD_RATE, timeout = 1)
        cls.serial_connection.flush()

        cls.last_message_time = time.time()
        cls._data_packet: bytearray = bytearray()

    @classmethod
    def on_update(cls) -> None:
        while cls.serial_connection.in_waiting > 0:
            try:
                cls._read_data()
            except Exception as e:
                LoggerModule.log_error(f"Undercarriage message cannot be decoded ({e})")
                return

        if time.time() - cls.last_message_time > 0.01:
            # Send messages
            with cls._dribbler_speed.get_lock() and cls._kicker_state.get_lock():
                dribbler = cls._dribbler_speed.value
                kicker = int(cls._kicker_state.value)
                beeper = 0

            cls.last_message_time = time.time()
            leds = int(''.join(["1" if b else "0" for b in UIModule.get_leds()]))
            message_json = json.dumps({
                #"a": ",".join(map(str, cls.get_motor_values()+[leds]+UIModule.get_display_info()+UIModule.get_ip_addr())),
                "e": ",".join(map(str, cls.get_motor_values()+[leds]+UIModule.get_display_info()+UIModule.get_ip_addr()+[dribbler]+[kicker]+[beeper])),
            }, separators=(',', ':'))
            #message_json += (71-len(message_json))*" "+"\n"
            message_json += (79-len(message_json))*" "+"\n"
            # LoggerModule.log_info("Sending message: {}".format(repr(message_json)))
            cls.serial_connection.write(message_json.encode())



    @classmethod
    def on_stop(cls) -> None:
        cls.serial_connection.close()

    @classmethod
    def _parse_packet(cls)-> None:
        len_data = len(cls._data_packet)

        if(len_data != PACKET_SIZE):
            return

        if(cls._data_packet[0] != 0x7b or cls._data_packet[1] != 0x23 or cls._data_packet[2] != 0x66):
            return
        if(cls._data_packet[len_data-2] != 0x7d or cls._data_packet[len_data-1] != 0xa):
            return
        #FIXME check crc


        color_sensor_values = []
        for i in range(7):
            color_sensor_values.append(int.from_bytes(cls._data_packet[4+i*2 : 6+i*2], 'little'))

        with cls._color_sensor_values.get_lock() and cls._color_sensor_values_change.get_lock():
            if(cls._color_sensor_values_change.value == 1):
                for i in range(len(color_sensor_values)):
                    cls._color_sensor_values[i] = color_sensor_values[i]
                cls._color_sensor_values_change.value = 0
            else:
                for i in range(len(color_sensor_values)):
                    cls._color_sensor_values[i] = max(color_sensor_values[i], cls._color_sensor_values[i])

        heading_value = int.from_bytes(cls._data_packet[18 : 20], 'little') / 100

        with cls._heading.get_lock():
            cls._heading.value = heading_value

        buttons = int(cls._data_packet[20])

        button1 = buttons % 10
        button2 = (buttons // 10) % 10

        if button1 and not cls._button_one_last:
            with cls._button_one_status.get_lock():
                cls._button_one_status.value = not cls._button_one_status.value
        cls._button_one_last = button1

        if button2:
            if cls._to_terminate_timer.get() > 0.5:
                with cls._to_terminate.get_lock():
                    cls._to_terminate.value = True
                cls.stop()
        else:
            cls._to_terminate_timer.reset()

        gate = int.from_bytes(cls._data_packet[21 : 23], 'little')

        battery_voltage = []
        for i in range(2):
            battery_voltage.append(int.from_bytes(cls._data_packet[23+i*2 : 25+i*2], 'little') / 100)


        lidar_speed = int.from_bytes(cls._data_packet[27 : 29], 'little')

        lidar_timestamp = []
        lidar_start_angle = []
        lidar_end_angle = []
        lidar_distance = []

        for i in range(4):
            """
            lidar_timestamp.append(int.from_bytes(cls._data_packet[29+i*30 : 31+i*30], 'little'))
            lidar_start_angle.append(int.from_bytes(cls._data_packet[31+i*30 : 33+i*30], 'little') / 100)
            lidar_end_angle.append(int.from_bytes(cls._data_packet[33+i*30 : 35+i*30], 'little') / 100)
            for j in range(12):
                lidar_distance.append(int.from_bytes(cls._data_packet[35+i*30+j*2 : 37+i*30+j*2], 'little'))
            """

            start_angle = int.from_bytes(cls._data_packet[31+i*30 : 33+i*30], 'little') / 100
            end_angle = int.from_bytes(cls._data_packet[33+i*30 : 35+i*30], 'little') / 100

            distances = list()

            for j in range(12):
                distances.append(int.from_bytes(cls._data_packet[35+i*30+j*2 : 37+i*30+j*2], 'little'))

            LidarModule.add_lidar_scan(start_angle, end_angle, distances)


        #LoggerModule.log_info("lidar_timestamp: {}".format(lidar_timestamp))
        #LoggerModule.log_info("lidar_start_angle: {}".format(lidar_start_angle))
        #LoggerModule.log_info("lidar_end_angle: {}".format(lidar_end_angle))
        #LoggerModule.log_info("lidar_distance: {}".format(lidar_distance))

    @classmethod
    def _read_data(cls) -> None:
        len_data = len(cls._data_packet)

        size_to_read = PACKET_SIZE - len_data
        in_waiting = cls.serial_connection.in_waiting

        if(size_to_read > in_waiting):
            size_to_read = in_waiting

        #LoggerModule.log_info("Size to read: {}".format(size_to_read))
        raw_data = cls.serial_connection.read(size_to_read)
        #LoggerModule.log_info("Raw data: {}".format(raw_data.hex()))

        for byte in raw_data:
            if(len_data <= 3):
                if(len_data == 0 and byte != 0x7b): #b'{'
                    #LoggerModule.log_info("Byte: {}".format(hex(byte)))
                    continue
                if(len_data == 1 and byte != 0x23): #b'#'
                    len_data = 0
                    cls._data_packet.clear()
                    continue
                if(len_data == 2 and byte != 0x66): #b'f'
                    len_data = 0
                    cls._data_packet.clear()
                    continue
                if(len_data == 3 and byte != 0x91): #145
                    len_data = 0
                    cls._data_packet.clear()
                    continue

            cls._data_packet.append(byte)
            len_data += 1

        #LoggerModule.log_info("Len data: {}".format(len_data))
        if(len_data >= PACKET_SIZE):
            cls._parse_packet()
            cls._data_packet.clear()



    @classmethod
    def get_button_one_status(cls) -> bool:
        with cls._button_one_status.get_lock():
            return cls._button_one_status.value

    @classmethod
    def set_robot_movement(cls, movement: Movement) -> None:
        """Set the robot movement in angle of motion, speed in mm/s and spin in deg/s"""
        speed_mot = movement.get_magnitude()/math.pi/con.ROBOT_WHEEL_D*60*con.MOTOR_SPEED_TO_RPM
        spin_mot = mathf.angular_speed_to_rpm(movement.spin)*con.MOTOR_SPEED_TO_RPM
        cls.set_motor_values(*mathf.direction_to_motors_abs(math.degrees(movement.get_angle()), spin_mot, speed_mot))

    # TODO: Check whether motor values are in range
    @classmethod
    def set_motor_values(cls, first, second, third, fourth) -> None:
        motors = [first, second, third, fourth]
        for i, m in enumerate(motors):
            motors[i] = int(m*con.MOTOR_MULTIPLIERS[i])

            if abs(m) > con.MOTOR_SPEED_RANGE:
                motors[i] = con.MOTOR_SPEED_RANGE if m > 0 else -con.MOTOR_SPEED_RANGE
                LoggerModule.log_warning(f"Motor value n.{i} out of range: {m}")

        with cls._motor_values.get_lock():
            cls._motor_values[0:4] = motors

    @classmethod
    def set_dribbler_speed(cls, speed) -> None:
        with cls._dribbler_speed.get_lock():
            cls._dribbler_speed.value = speed

    @classmethod
    def set_kicker_state(cls, state) -> None:
        with cls._kicker_state.get_lock():
            cls._kicker_state.value = state

    @classmethod
    def get_to_terminate(cls):
        with cls._to_terminate.get_lock():
            return cls._to_terminate.value

    @classmethod
    def get_heading(cls):
        with cls._heading.get_lock():
            return cls._heading.value

    @classmethod
    def get_motor_values(cls) -> tuple:
        with cls._motor_values.get_lock():
            return cls._motor_values[0:4]

    @classmethod
    def get_color_sensor_values(cls) -> tuple:
        with cls._color_sensor_values.get_lock() and cls._color_sensor_values_change.get_lock():
            cls._color_sensor_values_change.value = 1
            return cls._color_sensor_values[0:16]
