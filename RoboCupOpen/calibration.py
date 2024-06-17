from soccer_robot.soccer_robot import SoccerRobot

from soccer_robot.logger.logger_module import LoggerModule
from soccer_robot.interface.camera_module.camera_module import CameraModule
from soccer_robot.interface.undercarriage_module import UndercarriageModule
from soccer_robot.interface.compass_module import CompassModule
from soccer_robot.interface.ui_module.ui_module import UIModule

from soccer_robot.mathf import mathf
from soccer_robot.mathf.vector2 import Vector2
import soccer_robot.constants as con
from soccer_robot.utils.timer import Timer

import os
import sys
import time

import math

class Robot(SoccerRobot):
    def on_start(self) -> None:
        self.do_calibration = 0
        self.calibration_state = 0
        self.calibration_state_old = -1
        self.calibration_timer = Timer()

        # 0 - movement_stop, 1 - movement_straight, 2 - movement_spin, 3 - movement_square, 4 - movement_star, 5 - movement_straight_line, 6 - movement_straight_angle, 7 - movement_angle_rotation, 8- movement_straight_stop, 9 - movement_custom
        self.MOVEMENT_SELECT = 1
        self.MOVEMENT_START_ANGLE = 0 # used in 4 - movement_star, 6 - movement_straight_angle, 7 - movement_angle_rotation
        self.movement_speed = 70
        self.MOVEMENT_SELECT = 6
        self.MOVEMENT_START_ANGLE = 45 # used in 4 - movement_star, 6 - movement_straight_angle, 7 - movement_angle_rotation
        self.movement_speed = 150
        self.movement_rotation = 0 # used in 7 - movement_angle_rotation
        self.movement_timer = Timer()
        self.movement_state = 0
        self.movement_counter = 0
        self.LINE_SENSORS_THRESHOLD = 500 #self.get_line_sensors_threshold() #used in 5 - movement_straight_line

        self.log_info_id_ptr = 0
        self.LOG_READ_FROM = 0
        self.LOG_READ_COUNT = 4000
        self.LOG_READ_TIME = 90
        self.log_info_id = [] #[0,2,5,10,14]    #[0, self.LOG_READ_TIME]

    def movement_stop(self):
        mt = 3000
        mv = self.movement_speed
        if (self.movement_timer.get() < mt):
            motor_values = (0, 0, 0, 0)
            UndercarriageModule.set_motor_values(*motor_values)
        else:
            LoggerModule.log_debug("movement_stop(mt= " + str(mt) + ", mv= " + str(mv) + ") finished")
            return 1
        return 0

    def movement_straight(self):
        mt = 100
        mv = self.movement_speed
        if (self.movement_timer.get() < mt):
            motor_values = (mv, mv, -mv, -mv)
            UndercarriageModule.set_motor_values(*motor_values)
        elif (self.movement_timer.get() < mt*3):
            motor_values = (-mv, -mv, mv, mv)
            UndercarriageModule.set_motor_values(*motor_values)
        else:
            motor_values = (0, 0, 0, 0)
            UndercarriageModule.set_motor_values(*motor_values)
            LoggerModule.log_debug("movement_straight(mt= " + str(mt) + ", mv= " + str(mv) + ") finished")
            return 1
        return 0

    def movement_spin(self):
        mt = 3000
        mv = self.movement_speed
        if (self.movement_timer.get() < mt):
            motor_values = (mv-20, mv, mv, mv)
            UndercarriageModule.set_motor_values(*motor_values)
        else:
            motor_values = (0, 0, 0, 0)
            UndercarriageModule.set_motor_values(*motor_values)
            LoggerModule.log_debug("movement_spin(mt= " + str(mt) + ", mv= " + str(mv) + ") finished")
            return 1
        return 0

    def movement_square(self): # square to the left
        mt = 700
        mv = self.movement_speed
        if (self.movement_timer.get() < mt):
            motor_values = (mv, mv, -mv, -mv)
            UndercarriageModule.set_motor_values(*motor_values)
        elif (self.movement_timer.get() < mt*2):
            motor_values = (mv, -mv, -mv, mv) #left
            UndercarriageModule.set_motor_values(*motor_values)
        elif (self.movement_timer.get() < mt*3):
            motor_values = (-mv, -mv, mv, mv)
            UndercarriageModule.set_motor_values(*motor_values)
        elif (self.movement_timer.get() < mt*4):
            motor_values = (-mv, mv, mv, -mv) #right
            UndercarriageModule.set_motor_values(*motor_values)
        else:
            motor_values = (0, 0, 0, 0)
            UndercarriageModule.set_motor_values(*motor_values)
            self.movement_timer.reset()
            self.movement_counter +=1
            if(self.movement_counter == 1):
                self.movement_counter = 0
                LoggerModule.log_debug("movement_square(mt= " + str(mt) + ", mv= " + str(mv) + ") finished")
                return 1
        return 0

    def movement_star(self):
        mt = 500
        mt2 = 0
        mt3 = 0
        mv = self.movement_speed
        SECTION = 90
        angle = self.MOVEMENT_START_ANGLE + self.movement_counter
        if (angle > 180):
            back_angle = angle-180
        else:
            back_angle = angle+180
        if(self.movement_timer.get() < mt):
            motor_values = mathf.direction_to_motors(angle, 0, mv, False)
            for i in range(4): # len(motor_values)
                motor_values[i] = int(motor_values[i])
            if self.movement_state != 1:
                LoggerModule.log_debug(str(motor_values))
            UndercarriageModule.set_motor_values(*motor_values)
            self.movement_state = 1

        elif(self.movement_timer.get() < mt + mt2):
            motor_values = [0,0,0,0]
            if self.movement_state != 2:
                LoggerModule.log_debug(str(motor_values))
            UndercarriageModule.set_motor_values(*motor_values)
            self.movement_state = 2

        elif(self.movement_timer.get() < mt*2 + mt2):
            motor_values = mathf.direction_to_motors(back_angle, 0, mv, False)
            for i in range(4): # len(motor_values)
                motor_values[i] = int(motor_values[i])
            if self.movement_state != 3:
                LoggerModule.log_debug(str(motor_values))
            UndercarriageModule.set_motor_values(*motor_values)
            self.movement_state = 3

        elif(self.movement_timer.get() < mt*2 + mt2 + mt3):
            motor_values = [0,0,0,0]
            if self.movement_state != 4:
                LoggerModule.log_debug(str(motor_values))
            UndercarriageModule.set_motor_values(*motor_values)
            self.movement_state = 4

        else:
            self.movement_counter += SECTION
            self.movement_state = 0
            if(self.movement_counter >= 360):
                motor_values = (0, 0, 0, 0)
                UndercarriageModule.set_motor_values(*motor_values)
                return 1
            self.movement_timer.reset()
        return 0

    def movement_straight_line(self):
        mv = self.movement_speed
        if(self.movement_state == 0):
            motor_values = (mv, mv, -mv, -mv)
            UndercarriageModule.set_motor_values(*motor_values)
            sensor_values = UndercarriageModule.get_color_sensor_values()
            active_i = []
            for i, v in enumerate(sensor_values):
                if v > self.LINE_SENSORS_THRESHOLD:
                    active_i.append(i)
            line = mathf.get_line_pos(active_i)
            if (len(active_i) > 0):
                self.movement_state = 1
                self.movement_counter = self.movement_timer.get()
                self.movement_timer.reset()
        elif(self.movement_state == 1):
            motor_values = (-mv, -mv, mv, mv)
            UndercarriageModule.set_motor_values(*motor_values)
            if(self.movement_timer.get() >= self.movement_counter):
                motor_values = (0, 0, 0, 0)
                UndercarriageModule.set_motor_values(*motor_values)
                return 1
        return 0

    def movement_straight_angle(self):
        mt = 500
        mt2 = 0
        mv = self.movement_speed
        angle = self.MOVEMENT_START_ANGLE
        if (angle > 180):
            back_angle = angle-180
        else:
            back_angle = angle+180
        if(self.movement_timer.get() < mt):
            motor_values = mathf.direction_to_motors(angle, 0, mv, False)
            for i in range(4): # len(motor_values)
                motor_values[i] = int(motor_values[i])
            if self.movement_state != 1:
                LoggerModule.log_debug(str(motor_values))
            UndercarriageModule.set_motor_values(*motor_values)
            self.movement_state = 1

        elif(self.movement_timer.get() < mt + mt2):
            motor_values = [0,0,0,0]
            if self.movement_state != 2:
                LoggerModule.log_debug(str(motor_values))
            UndercarriageModule.set_motor_values(*motor_values)
            self.movement_state = 2

        elif(self.movement_timer.get() < mt*2 + mt2):
            motor_values = mathf.direction_to_motors(back_angle, 0, mv, False)
            for i in range(4): # len(motor_values)
                motor_values[i] = int(motor_values[i])
            if self.movement_state != 3:
                LoggerModule.log_debug(str(motor_values))
            UndercarriageModule.set_motor_values(*motor_values)
            self.movement_state = 3
        else:
            self.movement_state = 0
            motor_values = (0, 0, 0, 0)
            UndercarriageModule.set_motor_values(*motor_values)
            self.movement_timer.reset()
            return 1
        return 0

    def movement_angle_rotation(self):
        mt = 1000
        mv = self.movement_speed
        spin = self.movement_rotation
        angle = self.MOVEMENT_START_ANGLE
        if (self.movement_timer.get() < mt):
            motor_values = mathf.direction_to_motors(angle, spin, mv, False)
            for i in range(4): # len(motor_values)
                motor_values[i] = int(motor_values[i])
            UndercarriageModule.set_motor_values(*motor_values)
        else:
            motor_values = (0, 0, 0, 0)
            UndercarriageModule.set_motor_values(*motor_values)
            LoggerModule.log_debug("movement_straight(mt= " + str(mt) + ", mv= " + str(mv) + ") finished")
            return 1

        return 0

    def movement_straight_stop(self):
        mt1 = 700
        mt1c = 5
        mt2 = 200
        mv = self.movement_speed
        if (self.movement_timer.get() < mt1):
            motor_values = (mv, mv, -mv, -mv)
            UndercarriageModule.set_motor_values(*motor_values)
        elif (self.movement_timer.get() < mt1 + mt1c):
            motor_values = (50, 50, 50, 50)
            UndercarriageModule.set_motor_values(*motor_values)
        elif (self.movement_timer.get() < mt1 + mt2 + mt1c):
            motor_values = (0, 0, 0, 0)
            UndercarriageModule.set_motor_values(*motor_values)
        else:
            motor_values = (0, 0, 0, 0)
            UndercarriageModule.set_motor_values(*motor_values)
            if(self.movement_counter == 10): # make 5 into global value
                LoggerModule.log_debug("movement_straight(mt= " + str(mt1+mt2*5) + ", mv= " + str(mv) + ") finished")
                return 1
            else:
                self.movement_counter += 1
                self.movement_timer.reset()

    def movement_custom(self):
        mt = 5000
        mv2 = 101
        mv1 = 57
        if (self.movement_timer.get() < mt):
            motor_values = (-mv1, mv2+7, mv2, -mv1+3) #-55 100 100 -55
            #motor_values = (-50, -50, 50, 50) #-55 100 100 -55
            UndercarriageModule.set_motor_values(*motor_values)
        else:
            motor_values = (0, 0, 0, 0)
            UndercarriageModule.set_motor_values(*motor_values)
            LoggerModule.log_debug("movement_straight(mt= " + str(mt) + ") finished")
            return 1
        return 0

    def movement_switch(self):
        if(self.MOVEMENT_SELECT == 0):
            if (self.movement_stop() == 1):
                return 1
            return 0
        elif(self.MOVEMENT_SELECT == 1):
            if (self.movement_straight() == 1):
                return 1
            return 0
        elif(self.MOVEMENT_SELECT == 2):
            if (self.movement_spin() == 1):
                return 1
            return 0
        elif(self.MOVEMENT_SELECT == 3):
            if (self.movement_square() == 1):
                return 1
            return 0
        elif(self.MOVEMENT_SELECT == 4):
            if (self.movement_star() == 1):
                return 1
            return 0
        elif(self.MOVEMENT_SELECT == 5):
            if (self.movement_straight_line() == 1):
                return 1
            return 0
        elif(self.MOVEMENT_SELECT == 6):
            if (self.movement_straight_angle() == 1):
                return 1
            return 0
        elif(self.MOVEMENT_SELECT == 7):
            if (self.movement_angle_rotation() == 1):
                return 1
            return 0
        elif(self.MOVEMENT_SELECT == 8):
            if (self.movement_straight_stop() == 1):
                return 1
            return 0
        elif(self.MOVEMENT_SELECT == 9):
            if (self.movement_custom() == 1):
                return 1
            return 0
        else:
            LoggerModule.log_error("nonexistent movement selected")
            return 1

    def on_update(self) -> None:
        if (self.calibration_state != self.calibration_state_old):
            LoggerModule.log_debug("on_update: calibration_state = " + str(self.calibration_state))
            self.calibration_state_old = self.calibration_state

        if (self.calibration_state == 0):
            motor_values = (0, 0, 0, 0)
            UndercarriageModule.set_motor_values(*motor_values)

            UndercarriageModule.request_stat_reset()
            self.calibration_state = 1
            self.calibration_timer.reset()

        elif(self.calibration_state == 1):
            state = UndercarriageModule.get_stat_reset_state()
            if (state == 0):
                LoggerModule.log_error("Calibration error(1)!")
                self.calibration_state = 9
                return

            if (state == 3):
                self.calibration_state = 2
                self.calibration_timer.reset()
                self.movement_timer.reset()
                return

            if (self.calibration_timer.get() > 1000):
                LoggerModule.log_error("Calibration error(2)!")
                self.calibration_state = 0
                return

        elif(self.calibration_state == 2):

            if (self.movement_switch() == 1):
                self.calibration_state = 3

        elif(self.calibration_state == 3):
            UndercarriageModule.request_stat_read()
            self.calibration_state = 4
            self.calibration_timer.reset()

        elif(self.calibration_state == 4):
            state = UndercarriageModule.get_stat_read_state()
            if (state == 0):
                LoggerModule.log_error("Calibration error(3)!")
                self.calibration_state = 9
                return

            if (state == 3):
                self.calibration_state = 5
                self.calibration_timer.reset()
                return

            if (self.calibration_timer.get() > 1000):
                LoggerModule.log_error("Calibration error(4)!")
                self.calibration_state = 3
                return

        elif(self.calibration_state == 5):
            if (self.log_info_id_ptr == 0):
                samples = UndercarriageModule.get_sensor_stats_samples()
                stats = UndercarriageModule.get_sensor_stats_values()
                histogram = UndercarriageModule.get_sensor_histogram_values()
                LoggerModule.log_debug("Sample count: " + str(samples))
                LoggerModule.log_debug("Sensors mins: " + str(stats[0:16]))
                LoggerModule.log_debug("Sensors maxs: " + str(stats[16:32]))
                LoggerModule.log_debug("Sensors avgs: " + str(stats[32:48]))

                intervals = []
                for i in range(0, 1024, 128):
                    intervals.append(i)
                LoggerModule.log_debug("Histogram interval mins:" + str(intervals))
                intervals.clear()
                for i in range(128, 1024+1, 128):
                    intervals.append(i-1)
                LoggerModule.log_debug("Histogram interval maxs:" + str(intervals))

                for i in range(16):
                    LoggerModule.log_debug("Sensors hist[" + str(i) + "]: " + str(histogram[8*i:8*(i+1)]))
            if(len(self.log_info_id) > 0):
                UndercarriageModule.request_log_read(self.log_info_id[self.log_info_id_ptr], self.LOG_READ_FROM, self.LOG_READ_COUNT)
                self.calibration_state = 6
            else:
                self.calibration_state = 9
            self.calibration_timer.reset()

        elif(self.calibration_state == 6):
            state = UndercarriageModule.get_log_read_state()
            if (state == 0):
                LoggerModule.log_error("Calibration error(5)!")
                self.calibration_state = 9
                return

            if (state == 4):
                self.calibration_state = 7
                self.calibration_timer.reset()
                return

            if (self.calibration_timer.get() > self.LOG_READ_COUNT * 10):
                LoggerModule.log_error("Calibration error(6)!")
                self.calibration_state = 5
                return

        elif(self.calibration_state == 7):
            log_data = UndercarriageModule.get_sensor_log_values()
            if (self.log_info_id[self.log_info_id_ptr] == self.LOG_READ_TIME):
                LoggerModule.log_debug("Log time: " + str(log_data))
            else:
                LoggerModule.log_debug("Log sensor[" + str(self.log_info_id[self.log_info_id_ptr]) + "]: " + str(log_data))
            if(self.log_info_id_ptr == len(self.log_info_id)-1):
                self.calibration_state = 9
            else:
                self.log_info_id_ptr += 1
                self.calibration_state = 5

        elif(self.calibration_state == 9):
           self.stop()

    def on_stop(self):
        UndercarriageModule.set_motor_values(0, 0, 0, 0)

robot = Robot()

if __name__ == "__main__":
    robot.run()
    sys.exit(robot.get_exit_code())
