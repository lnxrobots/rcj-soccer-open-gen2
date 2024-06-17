from soccer_robot.soccer_robot import SoccerRobot

from soccer_robot.logger.logger_module import LoggerModule
from soccer_robot.interface.camera_module.front_camera_module import FrontCameraModule
from soccer_robot.interface.camera_module.mirror_camera_module import MirrorCameraModule
from soccer_robot.interface.undercarriage_module import UndercarriageModule
from soccer_robot.interface.ui_module.ui_module import UIModule
from soccer_robot.interface.bluetooth_module import BluetoothModule

# TEMPORARY SOLUTION, should be handled by soccer_robot class in the future
from soccer_robot.visualization.visualizer_module import VisualizationModule

from soccer_robot.mathf import mathf
from soccer_robot.mathf.vector2 import Vector2
import soccer_robot.constants as con
from soccer_robot.utils.timer import Timer

import math
import os
import sys
import time

# import RPi.GPIO as GPIO

KICKOFF_PUSH_TIME = 0

# import RPi.GPIO as GPIO

class Robot(SoccerRobot):
    def on_start(self) -> None:
        LoggerModule.log_info("start")
        UndercarriageModule.set_motor_values(0, 0, 0, 0)

        self.star_time = time.time()
        self.has_ball_in_timer = Timer()
        self.shoot_timer = Timer()

        self.shoot = False

        self.camera_module = FrontCameraModule
        self.last_checked_front_timer = Timer()
        self.front_timer = Timer()
        self.checking_front_camera = False

        self.motor_values = (0, 0, 0, 0)
        self.dribbler_speed = 400

        self.ball_angle = 0
        self.ball_dist = 0
        self.goal_center_angle = 0
        self.goal_dist = 0

    def on_update(self) -> None:
        self.lock_motors = UndercarriageModule.get_button_one_status()



        # Get data from other modules
        self.goal_position = FrontCameraModule.get_goal_position()
        self.see_goal = self.goal_position[:2] != [-1, -1]

        max_angle = FrontCameraModule.camera.center_fov / 2

        self.heading = UndercarriageModule.get_heading()

        if self.camera_module == FrontCameraModule and not self.see_ball and not self.checking_front_camera:
            self.camera_module = MirrorCameraModule

        elif self.camera_module == MirrorCameraModule:
            if self.see_ball:
                in_front = self.ball_angle < max_angle or self.ball_angle > (360-max_angle)
                self.camera_module = FrontCameraModule if in_front else MirrorCameraModule
            elif self.last_checked_front_timer.get() > con.FRONT_CHECK_TIME:
                self.camera_module = FrontCameraModule
                self.front_timer.reset()
                self.last_checked_front_timer.reset()
                self.checking_front_camera = True

        if self.camera_module == FrontCameraModule and self.front_timer.get() > con.FRONT_STAY_TIME:
            self.last_checked_front_timer.reset()
            self.front_timer.reset()
            self.checking_front_camera = False

        FrontCameraModule.set_enable_ball(self.camera_module == FrontCameraModule)
        MirrorCameraModule.set_enable_ball(self.camera_module == MirrorCameraModule)

        self.goal_center_angle, self.goal_dist = FrontCameraModule.get_angle_dist(self.goal_position, con.GOAL_REAL_SIZE, 1)

        self.ball_position = self.camera_module.get_ball_position()
        self.see_ball = self.ball_position[:2] != [-1, -1]
        self.ball_angle, self.ball_dist = self.camera_module.get_angle_dist(self.ball_position)

        if self.camera_module == FrontCameraModule and self.see_ball:
            if not self.has_ball_in(self.ball_position[0], self.ball_position[1]):
                spin = mathf.direction_to_spin(self.ball_angle) * con.SPIN_BALL_SPEED * 0.7
                self.motor_values = mathf.direction_to_motors(0, spin, 80)
                self.has_ball_in_timer.reset()
            else:
                if self.has_ball_in_timer.get() < 2000:
                    spin = mathf.direction_to_spin(-self.heading) * con.SPIN_BALL_SPEED * 0.7
                    self.motor_values = mathf.direction_to_motors(180, spin, 80)
                    self.shoot = False
                else:
                    spin = mathf.direction_to_spin(self.goal_center_angle) * con.SPIN_BALL_SPEED * 0.7
                    self.motor_values = mathf.direction_to_motors(0, spin, 150)

                    if self.goal_center_angle < 5 or self.goal_center_angle > 355 and not self.shoot and self.has_ball_in_timer.get() > 2500:
                        self.dribbler_speed = 0
                        self.shoot_timer.reset()
                        self.shoot = True

                    if self.shoot and self.shoot_timer.get() > 100:
                        UndercarriageModule.set_kicker_state(True)
                        UndercarriageModule.set_motor_values(0, 0, 0, 0)
                        time.sleep(1)
                        UndercarriageModule.set_kicker_state(False)
                        self.dribbler_speed = 400

        else:
            if self.camera_module == FrontCameraModule and not self.see_ball:
                spin = mathf.direction_to_spin(-self.heading) * con.SPIN_BALL_SPEED * 0.7
                self.motor_values = mathf.direction_to_motors(180, spin, 80)
            else:
                self.motor_values = (0, 0, 0, 0)

        if not self.lock_motors:
            UndercarriageModule.set_motor_values(*self.motor_values)
            UndercarriageModule.set_dribbler_speed(self.dribbler_speed)


        else:
            self.star_time = time.time()

            UndercarriageModule.set_motor_values(0, 0, 0, 0)
            UndercarriageModule.set_dribbler_speed(0)
            UndercarriageModule.set_kicker_state(False)

    def on_stop(self):
        UndercarriageModule.set_motor_values(0, 0, 0, 0)
        UndercarriageModule.set_dribbler_speed(0)
        UndercarriageModule.set_kicker_state(False)
        time.sleep(0.1)

    def has_ball_in(self, ball_x, ball_y):
        sx, sy, sw, sh = FrontCameraModule.camera.slot_bounding_box
        return abs(ball_x - sx) < sw / 2 and abs(ball_y - sy) < sh / 2

robot = Robot()

if __name__ == "__main__":
    robot.run()
    sys.exit(robot.get_exit_code())
