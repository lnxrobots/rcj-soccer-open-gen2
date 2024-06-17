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

        # GPIO.setup(16, GPIO.IN)

        LoggerModule.log_debug("Main Process pid: " + str(os.getpid()))

        self.motor_values = (0, 0, 0, 0)
        UndercarriageModule.set_motor_values(*self.motor_values)

        self.ball_position = [0, 0, 0, 0]
        self.goal_position = [0, 0, 0, 0]

        self.see_ball = False
        self.see_goal = False

        self.camera_module = FrontCameraModule
        self.last_checked_front_timer = Timer()
        self.front_timer = Timer()

        self.ball_angle = 0
        self.ball_dist = 0
        self.goal_center_angle = 0
        self.goal_dist = 0

        self.heading = 0

        self.active_i = []
        self.line = []

        self.lock_motors = True
        self.speed = 0
        self.shooting_speed = 0
        self.rotating_speed = 0

        self.backwards_speed = 0
        self.line_avoid_speed = 0

        self.seen_line = False
        self.anti_line_angle = 0
        self.line_timer = Timer()
        self.line_avoid_timer = con.LINE_AVOID_FAST

        self.last_ball_angle = 0
        self.looking_for_ball_phase = 0
        self.last_seen_ball_timer = Timer()

        self.goalie = False
        self.goalie_change_timer = Timer()

        self.should_defend = False
        self.defend_start_timer = Timer()

        self.start_button_timer = Timer()

        self.line_while_going_for_ball = 0

        self.rotating = False
        self.rotating_timer = Timer()

        self.goalie_status = 1

        self.checking_front_camera = False

    def on_update(self) -> None:
        self.motor_values = (0, 0, 0, 0)

        # Set speeds from calibration
        self.speed = self.get_robot_speed()
        self.shooting_speed = self.get_robot_shooting_speed()
        self.rotating_speed = self.get_robot_rotating_speed()

        self.backwards_speed = self.speed
        self.line_avoid_speed = self.speed


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
        # MirrorCameraModule.set_enable_goal(True)


        
        self.ball_position = self.camera_module.get_ball_position()
        self.see_ball = self.ball_position[:2] != [-1, -1]
        self.ball_angle, self.ball_dist = self.camera_module.get_angle_dist(self.ball_position)

        # LoggerModule.log_debug(f"ball_angle: {self.ball_angle}, ball_dist: {self.ball_dist}")

        #LoggerModule.log_info(f"angle: {self.ball_angle:2f}, dist {self.ball_dist:2f}, Front enable_ball: {FrontCameraModule.get_enable_ball()}, Mirror enable_ball: {MirrorCameraModule.get_enable_ball()}")

        #LoggerModule.log_debug(f"ball_pos: {self.ball_position}")

        self.heading = UndercarriageModule.get_heading()
        sensor_values = UndercarriageModule.get_color_sensor_values()
        self.lock_motors = UndercarriageModule.get_button_one_status()

        #Log lock_motors

        self.active_i = []
        for i, v in enumerate(sensor_values):
            if v > self.get_line_sensors_threshold():
                self.active_i.append(i)
        self.line = mathf.get_line_pos(self.active_i)


        goal_right_corner_angle = FrontCameraModule.get_angle_dist(
            [self.goal_position[0] + self.goal_position[2] / 2] + self.goal_position[1:],
            con.GOAL_REAL_SIZE, 1
        )[0]
        self.goal_center_angle, self.goal_dist = FrontCameraModule.get_angle_dist(self.goal_position, con.GOAL_REAL_SIZE, 1)
        goal_left_corner_angle = FrontCameraModule.get_angle_dist(
            [self.goal_position[0] - self.goal_position[2] / 2] + self.goal_position[1:],
            con.GOAL_REAL_SIZE, 1
        )[0]
        goal_width_angle = mathf.normalize_angle(goal_right_corner_angle - goal_left_corner_angle)

        if (self.see_ball):
            self.last_seen_ball_timer.reset()
        

        # Decide a goalie
        """
        if BluetoothModule.is_connected() and not BluetoothModule.is_locked():
            if con.ROBOT_INDEX == con.GOALIE_MASTER_INDEX:
                is_time = self.goalie_change_timer.get() >= con.GOALIE_CHANGE_TIME
                if is_time:
                    self.goalie_change_timer.reset()
                    both_see = self.see_ball and BluetoothModule.see_ball()
                    other_is_near = both_see and BluetoothModule.get_ball_angle_dist()[1] <= self.ball_dist
                    self.goalie = BluetoothModule.see_ball() and (not self.see_ball or other_is_near)
            else:
                self.goalie = not BluetoothModule.is_goalie()
        else:
            self.goalie = False
        """

        self.goalie = False

        in_front = self.ball_angle < max_angle or self.ball_angle > (360-max_angle)

        if self.camera_module == FrontCameraModule and not self.checking_front_camera:
            # PLAY ON GOAL
            # If the robot sees the ball and the goal, ...
            if (self.see_ball and self.see_goal):
                self.last_ball_angle = self.ball_angle

                # If ball angle is between goal corner angles = robot is in shooting position, ...
                if (mathf.is_angle_between(self.ball_angle, (goal_left_corner_angle) % 360, (goal_right_corner_angle) % 360) and (self.ball_angle < 15 or self.ball_angle > 360 - 15)):
                    self.set_status("goal shoot")
                    self.goal_shoot()
                # Else robot needs to go to the suitable shooting position
                else:
                    self.set_status("goal shooting pos")
                    self.go_to_goal_shooting_pos()

            # PLAY ON NORTH
            elif (self.see_ball):
                self.last_ball_angle = self.ball_angle

                if (self.heading < 360 - con.GET_AROUND_ANGLE and self.heading > con.GET_AROUND_ANGLE) or (self.ball_angle < 360 - con.GET_AROUND_ANGLE and self.ball_angle > con.GET_AROUND_ANGLE):
                    self.set_status("compass shooting pos")
                    self.go_to_compass_shooting_pos()

                else:
                    self.set_status("compass shoot")
                    self.compass_shoot()

            # LOOK FOR BALL
            else:
                self.set_status("looking for ball")
                if self.goalie:
                    self.back_off()
                else:
                    self.look_for_ball()

            # If robot detects line for few times when he is trying to get to ball, do direct shot
            if (self.should_defend and con.ROBOT_DEFEND):
                if (not mathf.is_angle_between(self.heading, 120, 240)):
                    self.defend()

            if (self.line_while_going_for_ball >= 3 and not self.should_defend):
                self.should_defend = True
                self.defend_start_timer.reset()

            if (self.should_defend):
                if (self.defend_start_timer.get() > 2000 or not self.see_ball):
                    self.should_defend = False
                    self.line_while_going_for_ball = 0

            # if self.goalie and self.see_ball:
            #     spin = mathf.direction_to_spin(-self.heading) * con.BACK_OFF_SPIN
            #     ball_vector = Vector2(a=math.radians(self.ball_angle+self.heading), m=self.ball_dist)
            #     ball_vector.y = abs(ball_vector.x/3)
            #     if not self.should_defend:
            #         ball_vector.y = 0
            #     move_angle = math.degrees(ball_vector.get_angle())-self.heading
            #     self.motor_values = mathf.direction_to_motors(move_angle, spin, self.speed)
        else:
            xBallDisplacement = math.sin(math.radians(self.ball_angle)) * self.ball_dist
            yBallDisplacement = math.cos(math.radians(self.ball_angle)) * self.ball_dist

            #print(180 + math.degrees(math.atan((xBallDisplacement - 200) / yBallDisplacement)), self.ball_angle)

            if self.see_ball and not in_front:
                spin = mathf.direction_to_spin(self.heading) * con.SPIN_BACKWARDS_SPEED

                if yBallDisplacement < 0:
                    if abs(xBallDisplacement) < 250:
                        if self.ball_angle < 180:
                            self.motor_values = mathf.direction_to_motors(270, -spin, self.backwards_speed)
                        else:
                            self.motor_values = mathf.direction_to_motors(90, -spin, self.backwards_speed)
                    elif abs(xBallDisplacement) > 500:
                        if self.ball_angle < 180:
                            self.motor_values = mathf.direction_to_motors(90, -spin, self.backwards_speed)
                        else:
                            self.motor_values = mathf.direction_to_motors(270, -spin, self.backwards_speed)        


                    else:
                        multiplier = mathf.clamp(mathf.remap(0, 500, 1, 2, -yBallDisplacement), 1, 2)

                        self.motor_values = mathf.direction_to_motors(180, -spin, self.backwards_speed * multiplier)

                else: #mathf.is_angle_between(self.ball_angle, 0, 90) or mathf.is_angle_between(self.ball_angle, 270, 360):
                    spin = -mathf.direction_to_spin(self.ball_angle) * (con.SPIN_BACKWARDS_SPEED / 5)

                    if self.ball_angle < 180:
                        self.motor_values = mathf.direction_to_motors(180 - 30 - self.heading, -spin, self.backwards_speed)
                    else:
                        self.motor_values = mathf.direction_to_motors(180 + 30 - self.heading, -spin, self.backwards_speed)
                    
            else:
                # Look for ball eventually
                self.look_for_ball()

        # Line check
        self.check_lines()

        self.tracker.update(
            self.heading, self.anti_line_angle+180, self.seen_line,
            self.see_goal, self.goal_center_angle, self.goal_dist,
            self.motor_values, self.lock_motors
        )

        VisualizationModule.set_position_heatmap(self.tracker.heatmap)

        self.set_has_ball(self.has_ball_in(*self.ball_position[0:2]))

        #LoggerModule.log_info(self.get_status())

        ##FIXME pridane na kruzku 23.6 povodne to tu nebolo
        #if (self.get_status() != "compass shoot" and self.get_status() != "compass shooting pos"):
        #    self.motor_values = (0, 0, 0, 0)

        #LoggerModule.log_debug(f"status: \"{self.get_status()}\", compass: {self.heading}, ball_pos: {self.ball_position}, goal_pos: {self.goal_position}, goalie: {self.goalie}, bluetooth: {BluetoothModule.is_connected()} sensor_values: {sensor_values}, motor_values: {self.motor_values}, last_ball: {self.last_seen_ball_timer.get()}, last_line: {self.line_timer.get()}, has_ball_in: {self.has_ball_in(*self.ball_position[0:2])}")

        #LoggerModule.log_info(self.tracker.heatmap.index(max(self.tracker.heatmap)) + 1)
        #self.set_status(str(self.tracker.heatmap.index(max(self.tracker.heatmap)) + 1) + "                  ")


        if (not self.lock_motors):
            UndercarriageModule.set_motor_values(*self.motor_values)
        else:
            UndercarriageModule.set_motor_values(0, 0, 0, 0)
            self.seen_line = False
            self.start_button_timer.reset()

    def on_stop(self):
        UndercarriageModule.set_motor_values(0, 0, 0, 0)


    # ROBOR BEHAVIORS
    def goal_shoot(self):
        # If robot holds the ball, shoot with directing to the goal
        if (self.has_ball_in(*self.ball_position[0:2])):
            spin = mathf.direction_to_spin(self.goal_center_angle) * con.SPIN_GOAL_SPEED

            self.motor_values = mathf.direction_to_motors(0, spin, self.shooting_speed)
        # Direct to the ball to get it
        else:
            move_speed = self.get_robot_speed()

            if (con.ROBOT_PRECISE_BALL_MOVE):
                speed_multiplier = mathf.remap(150, 400, 0.6, 0.8, self.ball_dist)
                speed_multiplier = mathf.clamp(speed_multiplier, 0.7, 1.0)

                move_speed *= speed_multiplier

            else:
                move_speed *= 0.8

            spin = mathf.direction_to_spin(self.ball_angle) * con.SPIN_BALL_SPEED * 0.7
            self.motor_values = mathf.direction_to_motors(0, spin, move_speed)

    def go_to_goal_shooting_pos(self):
        # Calculate angle to be able to get to the shooting position with target_distance
        c = math.sqrt(self.ball_dist ** 2 + con.BALL_TARGET_DISTANCE ** 2 - 2 * self.ball_dist * con.BALL_TARGET_DISTANCE * math.cos(math.radians(self.ball_angle)))
        alpha = math.degrees(math.acos(mathf.clamp((c ** 2 + self.ball_dist ** 2 - con.BALL_TARGET_DISTANCE ** 2) /  (2 * c * self.ball_dist), -1, 1)))

        # Angle to get to the shooting position
        target_angle = self.ball_angle + alpha if self.ball_angle < 180 else self.ball_angle - alpha
        target_angle = target_angle % 360

        #print(target_angle, self.ball_dist)

        # Since the robot can only move in 8 the direction, calculate the closest one
        #target_angle_rounded = round(target_angle / 45) * 45

        # During the move to the shooting position try to rotate to goal
        spin = mathf.direction_to_spin(self.goal_center_angle) * con.SPIN_SHOOTPOS_SPEED

        multiplier = mathf.clamp(mathf.remap(200, 800, 0.8, 1.8, self.ball_dist), 0.8, 2)

        self.motor_values = mathf.direction_to_motors(target_angle, spin, self.speed * multiplier)

    def compass_shoot(self):
        if (self.has_ball_in(*self.ball_position[0:2])):
            spin = -mathf.direction_to_spin(self.heading) * con.SPIN_GOAL_SPEED
            spin += con.ROBOT_SPIN_OFFSET_SMALL
            self.motor_values = mathf.direction_to_motors(0, spin, self.shooting_speed)
        else:
            move_speed = self.get_robot_speed()

            if (con.ROBOT_PRECISE_BALL_MOVE):
                ball_distance = mathf.distance_from_size(self.ball_position[2:4])

                speed_multiplier = mathf.remap(150, 400, 0.6, 0.8, ball_distance)
                speed_multiplier = mathf.clamp(speed_multiplier, 0.7, 1.0)

                move_speed *= speed_multiplier
            else:
                move_speed *= 0.8 ##FIXME na kruzku 23.6 zmenene na 1.2

            spin = mathf.direction_to_spin(self.ball_angle) * con.SPIN_BALL_SPEED + con.ROBOT_SPIN_OFFSET_BIG
            self.motor_values = mathf.direction_to_motors(0, spin, move_speed)

    def go_to_compass_shooting_pos(self):
        """
        if (self.heading < 180):
            angle = 90
            direction_multiplier = 1
        else:
            direction_multiplier = -1
            angle = 270

        if self.ball_dist > con.GET_AROUND_MAX_DIST:
            angle -= 90 * direction_multiplier
        elif self.ball_dist < con.GET_AROUND_MIN_DIST:
            angle += 90 * direction_multiplier

        spin = mathf.direction_to_spin(self.ball_angle) * con.SPIN_GET_AROUND_SPEED + con.ROBOT_SPIN_OFFSET_BIG
        self.motor_values = mathf.direction_to_motors(angle, spin, self.speed)
        """

        # EXPERIMENTAL: getting over ball close to it
        # FIXME: Proportional slowing down when getting closer to the shooting pos


        if (self.heading < 180):
            angle = 90
            direction_multiplier = 1
        else:
            direction_multiplier = -1
            angle = 270

        if (self.ball_dist > con.GET_AROUND_MID_DIST):

            if (self.heading < 180):
                angle = 45
            else:
                angle = 315


        if self.ball_dist > con.GET_AROUND_MAX_DIST:
            # If too far, go to ball
            angle = 0

            spin = mathf.direction_to_spin(self.ball_angle) * con.SPIN_BALL_SPEED
            speed = mathf.direction_to_motors(angle, spin, self.speed)

        else:
            speed_multiplier = mathf.remap(0, 180, con.GET_AROUND_MIN_SPEED_MULT, 1, abs((self.heading + 180) % 360 - 180))

            spin = mathf.direction_to_spin(self.ball_angle) * (con.SPIN_BALL_SPEED / 5)
            speed = mathf.direction_to_motors(angle, spin, self.speed * speed_multiplier)

            if (direction_multiplier == 1):
                if (self.ball_angle < 360 - con.GET_AROUND_MAX_BALL_ANGLE and self.ball_angle > 180):
                    rotating_speed = con.GET_AROUND_ROTATE_SPEED * direction_multiplier
                    speed = (rotating_speed, rotating_speed, rotating_speed, 0)
            else:
                if (self.ball_angle > con.GET_AROUND_MAX_BALL_ANGLE and self.ball_angle < 180):
                    rotating_speed = con.GET_AROUND_ROTATE_SPEED * direction_multiplier
                    speed = (0, rotating_speed, rotating_speed, rotating_speed)

        self.motor_values = speed


    def look_for_ball(self, go_backwards=True):
        if (self.last_seen_ball_timer.get() < con.ROBOT_BACKWARDS_TIME and go_backwards):

            ## ??
            ## We cannot spin to north when going 45 angle
            ## Possibility of pushing ball to our goal from angle

            if self.heading < 30 or self.heading > 330:
                direction = -1 if self.last_ball_angle < 180 else 1
                self.motor_values = mathf.direction_to_motors(180 + 45 * direction, 0, self.backwards_speed)
            else:
                spin = mathf.direction_to_spin(self.heading) * con.SPIN_BACKWARDS_SPEED
                self.motor_values = mathf.direction_to_motors(180, -spin, self.backwards_speed)

            self.looking_for_ball_phase = 0
        else:
            direction = -1 if self.last_ball_angle < 180 else 1
            speed = self.rotating_speed * direction

            if time.time() * 1000 % (con.BALL_FINDING_ROTATE_TIME + con.BALL_FINDING_WAIT_TIME) > con.BALL_FINDING_WAIT_TIME:
                self.motor_values = (speed, speed, speed, speed)
            else:
                self.motor_values = (0, 0, 0, 0)

    def back_off(self):
        if self.goalie_status == 1:
            m = max(self.tracker.heatmap)
            sectors = []
            for i, v in enumerate(self.tracker.heatmap):
                if v == m:
                    sectors.append(i)
            robot_sector = sectors[(len(sectors)-1)//2]
            robot_pos = Vector2(*con.TRACKER_HEATMAP_POINTS[robot_sector])
            move_vector = con.BACK_OFF_POS - robot_pos
            result_angle = math.degrees(move_vector.get_angle()) - self.heading
            result_angle_rounded = round(result_angle / 45) * 45
            spin = mathf.direction_to_spin(-self.heading) * con.BACK_OFF_SPIN
            self.motor_values = mathf.direction_to_motors(result_angle_rounded, spin, self.speed)


            if robot_sector in con.BACK_OFF_OK_SECTORS:
                self.goalie_status = 2

        elif self.goalie_status == 2:
            if self.heading < 60 or self.heading > 180:
                self.motor_values = (-self.rotating_speed, -self.rotating_speed, -self.rotating_speed, -self.rotating_speed)
            else:
                self.goalie_status = 3
        elif self.goalie_status == 3:
            if self.heading > 300 or self.heading < 180:
                self.motor_values = (self.rotating_speed, self.rotating_speed, self.rotating_speed, self.rotating_speed)
            else:
                self.goalie_status = 4
        else:
            if mathf.is_angle_between(self.heading, 30, 330):
                self.goalie_status = 1
            else:
                self.motor_values = (-self.rotating_speed, -self.rotating_speed, -self.rotating_speed, -self.rotating_speed)

            # if self.last_ball_angle < 180:
            #     self.motor_values = mathf.direction_to_motors(90, spin, self.speed)
            # if self.last_ball_angle > 180:
                    #     self.motor_values = mathf.direction_to_motors(270, spin, self.speed)

    def defend(self):
        multiplier = 1
        if self.heading < 180:
            multiplier = -1

        move_speed = self.get_robot_speed()

        if (not self.has_ball_in(*self.ball_position[0:2])):
            speed_multiplier = mathf.remap(100, 400, 0.7, 0.8, self.ball_dist)
            speed_multiplier = mathf.clamp(speed_multiplier, 0.7, 1.0)

            move_speed *= speed_multiplier

            spin = mathf.direction_to_spin(self.ball_angle) * con.SPIN_BALL_SPEED + con.ROBOT_SPIN_OFFSET_BIG
            self.motor_values = mathf.direction_to_motors(0, spin, move_speed)
        else:
           self.motor_values = mathf.direction_to_motors(0, 0.025 * multiplier, 70)

    def check_lines(self):
        if self.seen_line:
            self.set_status("avoiding line")
            self.motor_values = mathf.direction_to_motors(self.anti_line_angle, 0, self.line_avoid_speed)

            if (self.line_timer.get() > self.line_avoid_timer):
                self.seen_line = False

        elif (len(self.active_i) > 0):
            self.set_status("avoiding line")
            self.line_while_going_for_ball += 1
            self.seen_line = True

            current_speed = max(map(abs, self.motor_values))

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
            #result_angle_rounded = round(result_angle / 45) * 45
            self.anti_line_angle = result_angle #result_angle_rounded

            self.motor_values = mathf.direction_to_motors(result_angle, 0, self.speed)


    def has_ball_in(self, ball_x, ball_y):
        sx, sy, sw, sh = FrontCameraModule.camera.slot_bounding_box
        return abs(ball_x - sx) < sw / 2 and abs(ball_y - sy) < sh / 2

robot = Robot()


if __name__ == "__main__":
    robot.run()
    sys.exit(robot.get_exit_code())
