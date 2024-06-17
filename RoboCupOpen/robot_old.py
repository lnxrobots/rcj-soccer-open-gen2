from soccer_robot.soccer_robot import SoccerRobot

from soccer_robot.logger.logger_module import LoggerModule
from soccer_robot.interface.camera_module.front_camera_module import FrontCameraModule
from soccer_robot.interface.camera_module.mirror_camera_module import MirrorCameraModule
from soccer_robot.interface.undercarriage_module import UndercarriageModule
from soccer_robot.interface.lidar.lidar_module import LidarModule
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

        self.kicking = False
        self.kicking_timer = Timer()

        self.last_ball_delta_x = 0

        self.motorValues = (0, 0, 0, 0)
        self.last_ball_timer = Timer()


        self.line_timer = Timer()
        self.seen_line = False
        self.line_while_going_for_ball = 0

        self.last_ball_relative_x = 0

        self.active_i = []
        self.line = []

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

        fieldWidth = 1700 #1400 
        fieldHeight = 2030 #2030

        position = LidarModule.get_position()
        goalLine = 1500

                #LoggerModule.log_debug(position)

        self.motorValues = (0, 0, 0, 0)

        horizontal = 0
        vertical = 0

        maxSpeed = 90

        maxDistance = 0

        maxDistance = max(maxDistance, abs(position.x - goalLine / 2))
        maxDistance = max(maxDistance, abs(position.y - 1650))

        relative_ball_pos = Vector2(math.sin(math.radians(self.ball_angle + self.heading)) * self.ball_dist, math.cos(math.radians(self.ball_angle + self.heading)) * self.ball_dist)
        absolute_ball_pos = Vector2(position.x + relative_ball_pos.x, position.y - relative_ball_pos.y)

        if self.see_ball:
            self.last_ball_relative_x = relative_ball_pos.x
            self.last_ball_timer.reset()


        goal_ball_vector = absolute_ball_pos - Vector2(fieldWidth / 2, fieldHeight)


        if absolute_ball_pos.y < goalLine or not self.see_ball:
            lineA = goal_ball_vector.y
            lineB = -goal_ball_vector.x
            lineC = -lineA * absolute_ball_pos.x - lineB * absolute_ball_pos.y


            xIntersect = (-lineB * position.y - lineC) / lineA 

            positionDelta = xIntersect - position.x

            targetYPos = goalLine

            if absolute_ball_pos.y > 2050 / 2:
                targetYPos = min(absolute_ball_pos.y + 300, goalLine)

            if abs(position.y - targetYPos) < 100:
                vertical = 0

            elif position.y > targetYPos:
                vertical = 1

            elif position.y < targetYPos:
                vertical = -1

            if not self.see_ball:
                if abs(position.x - goalLine / 2) < 100:
                    horizontal = 0

                elif position.x > goalLine / 2:
                    horizontal = -1

                elif position.x < goalLine / 2:
                    horizontal = 1

                if maxDistance < 500:
                    maxSpeed = 70

            else:
                if relative_ball_pos.y > 200 and position.x > 300 and position.x < fieldWidth - 300:
                    if abs(positionDelta) < 50:
                        horizontal = 0
                    elif positionDelta > 0:
                        horizontal = 1
                    elif positionDelta < 0:
                        horizontal = -1
                    if abs(positionDelta) < 150:
                        maxSpeed = 60
                else:
                    if abs(relative_ball_pos.x) < 50:
                        horizontal = 0
                    elif relative_ball_pos.x > 0:
                        horizontal = 1
                    elif relative_ball_pos.x < 0:
                        horizontal = -1
                    if abs(relative_ball_pos.x) < 20:
                        maxSpeed = 60

            moveVector = Vector2(horizontal, vertical)

            if moveVector != Vector2.zero:
                angle = math.degrees(moveVector.get_angle())
                spin = -mathf.direction_to_spin(self.heading) * con.SPIN_GOAL_SPEED
                
                if self.see_ball:
                    spin = mathf.direction_to_spin(self.ball_angle) * con.SPIN_GOAL_SPEED

                self.motorValues = mathf.direction_to_motors(round((angle -  self.heading) / 45) * 45, spin, maxSpeed)

            else:
                if self.see_ball:
                    if not (self.ball_angle < 20 or self.ball_angle > 340):
                        if self.ball_angle > 180:
                            direction = -1
                        else:
                            direction = 1

                        speed = -50 * direction
                        self.motorValues = (speed, speed, speed, speed)
                    else:
                        self.motorValues = (0, 0, 0, 0)
                else:
                    if not (self.heading < 20 or self.heading > 340):
                        if self.heading > 180:
                            direction = -1
                        else:
                            direction = 1

                        speed = 50 * direction
                        self.motorValues = (speed, speed, speed, speed)
                    else:
                        self.motorValues = (0, 0, 0, 0)

            if (self.ball_angle < 20 or self.ball_angle > 340) and self.ball_dist < 400 and self.see_ball and position.y > fieldHeight / 2 and position.x > 500 and position.x < fieldWidth - 500 and (self.heading > 270 or self.heading < 90):
                spin = mathf.direction_to_spin(self.ball_angle) * con.SPIN_GOAL_SPEED
                self.motorValues = mathf.direction_to_motors(0, spin, 85)
                
        elif self.see_ball:
            if abs(absolute_ball_pos.x - fieldWidth / 2) < 300:
                self.motorValues = (0, 0, 0, 0)

            elif position.x > fieldWidth / 2 - 300:
                target_position: Vector2 = Vector2(absolute_ball_pos.x, goalLine)
                move_angle = (target_position - position).get_angle()
                
                spin = mathf.direction_to_spin(self.ball_angle) * con.SPIN_GOAL_SPEED
                self.motorValues = mathf.direction_to_motors(round((move_angle -  self.heading) / 45), spin, 60)

            elif absolute_ball_pos.y - position.y > 150:
                target_position: Vector2 = Vector2(fieldWidth / 2 - 350, absolute_ball_pos.y)
                move_angle = (target_position - position).get_angle()
                
                spin = mathf.direction_to_spin(self.ball_angle) * con.SPIN_GOAL_SPEED
                self.motorValues = mathf.direction_to_motors(round((move_angle -  self.heading) / 45), spin, 60)

            else:
                self.motorValues = (0, 0, 0, 0)


        if not self.see_ball and self.last_ball_timer.get() < 200:
            direction = mathf.sign(self.last_ball_relative_x)
            speed = -50 * direction
            self.motorValues = (speed, speed, speed, speed)


        #if position.x < 500 or position.x > fieldWidth - 500:
            #self.check_lines()

        if self.lock_motors:
            UndercarriageModule.set_motor_values(0, 0, 0, 0)
        else:
            UndercarriageModule.set_motor_values(*self.motorValues)

        self.last_ball_delta_x = relative_ball_pos.x


        self.set_status("{}, {}".format(absolute_ball_pos.y < goalLine, round(position.x, 1)))


    def check_lines(self):
        if self.seen_line:
            self.set_status("avoiding line")
            self.motorValues = mathf.direction_to_motors(self.anti_line_angle, 0, 80)

            if (self.line_timer.get() > self.line_avoid_timer):
                self.seen_line = False

        elif (len(self.active_i) > 0):
            self.set_status("avoiding line")
            self.line_while_going_for_ball += 1
            self.seen_line = True

            current_speed = max(map(abs, self.motorValues))

            if (current_speed >= con.LINE_AVOID_FAST_THRES):
                self.line_avoid_timer = con.LINE_AVOID_FAST
            else:
                self.line_avoid_timer = con.LINE_AVOID_SLOW

            self.line_timer.reset()

            antiline_vector = Vector2(0, 0)
            if self.line is not None:
                antiline_vector.set_angle_magn(
                    math.radians(self.line[0]+180),
                    con.LINE_AVOID_ANTIVECTOR*con.SENSOR_MAX_DIST-self.line[1]
                )

            result_angle = math.degrees(antiline_vector.get_angle())
            result_angle_rounded = round(result_angle / 45) * 45
            self.anti_line_angle = result_angle_rounded

            self.motorValues = mathf.direction_to_motors(result_angle_rounded, 0, 80)

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