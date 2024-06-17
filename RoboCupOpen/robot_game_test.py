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
from soccer_robot.mathf.vector2 import Vector2, Movement
import soccer_robot.constants as con
from soccer_robot.utils.timer import Timer
from soccer_robot.utils.tracker import Tracker

import math
import os
import sys
import time

class Robot(SoccerRobot):
    def on_start(self) -> None:
        LoggerModule.log_info("start")
        UndercarriageModule.set_motor_values(0, 0, 0, 0)

        self.start_time = time.time()
        self.has_ball_in_timer = Timer()
        self.turn_angle = 0
        self.get_to_ball_timer = Timer()
        self.ball_time_started = False

        self.shoot = False
        self.shoot_timer = Timer()

        self.camera_module = FrontCameraModule
        self.last_checked_front_timer = Timer()
        self.front_timer = Timer()
        self.checking_front_camera = False

        self.movement = Movement()
        self.dribbler_speed = 0#400

        self.ball_angle = 0
        self.ball_dist = 0
        self.goal_center_angle = 0
        self.goal_dist = 0

        self.braking_v = 1

        self.tracker = Tracker()

    def on_update(self) -> None:
        self.lock_motors = UndercarriageModule.get_button_one_status()

        # Get data from other modules
        self.goal_position = FrontCameraModule.get_goal_position()
        self.see_goal = self.goal_position[:2] != [-1, -1]

        max_angle = FrontCameraModule.camera.center_fov / 2

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
        ball_angle, ball_dist = self.camera_module.get_angle_dist(self.ball_position)

        self.tracker.update(self.movement, self.see_ball, ball_angle, ball_dist)
        self.heading = self.tracker.heading
        # self.ball_angle = mathf.normalize_angle(math.degrees(self.tracker.get_relative_ball_position().get_angle())-self.heading)
        # self.ball_dist = self.tracker.get_relative_ball_position().get_magnitude()
        self.ball_angle = ball_angle
        self.ball_dist = ball_dist
        # LoggerModule.log_info(f"\nball_see {self.see_ball}, {self.tracker.ball_seen()}\nheading {UndercarriageModule.get_heading():.2f}, {self.tracker.heading:.2f}\nball_angle {ball_angle:.2f}, {self.ball_angle:.2f}\nball_dist {ball_dist:.2f}, {self.ball_dist:.2f}")

        if self.see_ball:#self.tracker.ball_seen():
            if not self.has_ball_in(self.ball_position[0], self.ball_position[1]):
                if not self.ball_time_started:
                    self.get_to_ball_timer.reset()
                    self.ball_time_started = True
                self.dribbler_speed = 400
                spin_angle = self.ball_angle
                if spin_angle > 180:
                    spin_angle -= 360

                dist = self.ball_dist-con.BALL_CHASING_STOP_DIST

                v, travel_time = mathf.distance_to_speed_time(dist, con.BALL_CHASING_TIMES, con.BALL_CHASING_SPEEDS)
                spin, spin_time = mathf.distance_to_speed_time(abs(spin_angle), con.BALL_CHASING_TIMES, con.BALL_CHASING_SPINS)
                if travel_time == 0:
                    travel_time = 0.01
                spin *= spin_time/travel_time*mathf.sign(spin_angle)
                if dist < con.DRIBBLER_CATCH_TOLERANCE[0] and abs(spin_angle) < con.DRIBBLER_CATCH_TOLERANCE[1]/2:
                    spin = 0

                if abs(spin_angle) > con.BALL_CHASING_WATCH_TRESHOLD:
                    self.turn_angle = mathf.normalize_angle(self.heading + self.ball_angle)
                    spin = mathf.sign(spin_angle)*mathf.distance_to_speed_time(abs(spin_angle), con.WATCHING_TIMES, con.WATCHING_SPINS)[0]
                    v = 0
                self.movement = Movement(v, self.ball_angle, spin)
                self.has_ball_in_timer.reset()
            else:
                spin_angle = 360-self.heading
                if spin_angle > 180:
                    spin_angle -= 360
                spin = 150*mathf.sign(spin_angle)#*mathf.distance_to_speed_time(abs(spin_angle), con.WATCHING_TIMES, con.WATCHING_SPINS)[0]
                if abs(spin_angle) < 10:
                    self.dribbler_speed = 0
                    self.shoot = True

                self.movement = Movement(0, 0, spin)
                if self.ball_time_started:
                    self.ball_time_started = False
                    LoggerModule.log_info(f"Ball got in {self.get_to_ball_timer.get():.2f} ms")
        else:
            self.dribbler_speed = 0
            self.brake()
            a = mathf.normalize_angle(self.turn_angle-self.heading)
            if a > 180:
                a -= 360
            if abs(a) > con.BALL_CHASING_WATCH_TRESHOLD:
                turn_speed = mathf.sign(a)*mathf.distance_to_speed_time(abs(a), con.WATCHING_TIMES, con.WATCHING_SPINS)[0]
                self.movement = Movement(spin=turn_speed)

        # UndercarriageModule.set_kicker_state(False)
        if self.shoot:# and self.shoot_timer.get() > 100:
            # LoggerModule.log_info("KICK")
            UndercarriageModule.set_kicker_state(True)
            self.shoot_timer.reset()
            # UndercarriageModule.set_motor_values(0, 0, 0, 0)
            # time.sleep(1)
            self.shoot = False
            # self.dribbler_speed = 400
        if self.shoot_timer.get() > 100:
            UndercarriageModule.set_kicker_state(False)
        else:
            self.brake()

        if not self.lock_motors:
            UndercarriageModule.set_dribbler_speed(self.dribbler_speed)

        else:
            self.ball_time_started = False
            self.start_time = time.time()

            self.movement = Movement()
            UndercarriageModule.set_dribbler_speed(0)
            UndercarriageModule.set_kicker_state(False)

        UndercarriageModule.set_robot_movement(self.movement)

    def on_stop(self):
        UndercarriageModule.set_motor_values(0, 0, 0, 0)
        UndercarriageModule.set_dribbler_speed(0)
        UndercarriageModule.set_kicker_state(False)
        time.sleep(0.1)

    def has_ball_in(self, ball_x, ball_y):
        sx, sy, sw, sh = FrontCameraModule.camera.slot_bounding_box
        return abs(ball_x - sx) < sw / 2 and abs(ball_y - sy) < sh / 2

    def brake(self):
        self.movement = Movement(spin=mathf.rpm_to_angular_speed(self.braking_v))
        self.braking_v = -mathf.sign(self.braking_v)

robot = Robot()

if __name__ == "__main__":
    robot.run()
    sys.exit(robot.get_exit_code())
