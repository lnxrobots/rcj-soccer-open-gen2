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
        self.ball_position = [0, 0, 0]
        self.last_ball_position = [0, 0, 0]

        self.shoot = False

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

        ball_position_front = FrontCameraModule.get_ball_position()
        ball_position_mirror = MirrorCameraModule.get_ball_position()
        see_ball_front = ball_position_front[0:2] != [-1, -1]
        see_ball_mirror = ball_position_mirror[0:2] != [-1, -1]
        if(see_ball_front):
            self.ball_position = ball_position_front
            self.see_ball = True
            ball_angle, ball_dist = FrontCameraModule.get_angle_dist(self.ball_position)
        elif(see_ball_mirror):
            self.ball_position = ball_position_mirror
            self.see_ball = True
            ball_angle, ball_dist = MirrorCameraModule.get_angle_dist(self.ball_position)
        else:
            self.see_ball = False
            ball_angle, ball_dist = 0, 0

        self.goal_center_angle, self.goal_dist = FrontCameraModule.get_angle_dist(self.goal_position, con.GOAL_REAL_SIZE, 1)

        self.tracker.update(self.movement, self.ball_position != self.last_ball_position, self.see_ball, ball_angle, ball_dist)
        self.heading = self.tracker.heading
        self.ball_angle = mathf.normalize_angle(math.degrees(self.tracker.get_relative_ball_position().get_angle())-self.heading)
        self.ball_dist = self.tracker.get_relative_ball_position().get_magnitude()

        if self.tracker.ball_seen():
            if not self.has_ball_in(self.ball_position[0], self.ball_position[1]):
                if not self.ball_time_started:
                    self.get_to_ball_timer.reset()
                    self.ball_time_started = True
                self.dribbler_speed = con.DRIBBLER_SPEED
                spin_angle = self.ball_angle
                if spin_angle > 180:
                    spin_angle -= 360

                dist = self.ball_dist-con.BALL_CHASING_STOP_DIST

                # Use this to calculate the speed based on the time it takes to get to the ball
                # pos, travel_time = self.tracker.get_ball_catch_pos_time(con.BALL_CHASING_TIMES, con.BALL_CHASING_SPEEDS)
                # new_dist = (self.tracker.position-pos).get_magnitude()
                # v = mathf.distance_to_speed_time(new_dist, con.BALL_CHASING_TIMES, con.BALL_CHASING_SPEEDS)[0]
                v, travel_time = mathf.distance_to_speed_time(dist, con.BALL_CHASING_TIMES, con.BALL_CHASING_SPEEDS)
                spin, spin_time = mathf.distance_to_speed_time(abs(spin_angle), con.BALL_CHASING_TIMES, con.BALL_CHASING_SPINS)
                angle = self.ball_angle
                if travel_time == 0:
                    travel_time = 0.01
                spin *= spin_time/travel_time*mathf.sign(spin_angle)
                if dist < con.DRIBBLER_CATCH_TOLERANCE[0] and abs(spin_angle) < con.DRIBBLER_CATCH_TOLERANCE[1]/2:
                    spin = 0
                    angle = 0
                    v = con.DRIBBLER_CATCH_SPEED

                if abs(spin_angle) > con.BALL_CHASING_WATCH_TRESHOLD:
                    self.turn_angle = mathf.normalize_angle(self.heading + self.ball_angle)
                    spin = mathf.sign(spin_angle)*mathf.distance_to_speed_time(abs(spin_angle), con.WATCHING_TIMES, con.WATCHING_SPINS)[0]
                    v = 0
                self.movement = Movement(v, angle, spin)
                self.has_ball_in_timer.reset()
            else:
                self.brake()
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

        self.last_ball_position = self.ball_position

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
