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

        # add tracker to north code
        self.capture_ball = 0
        #self.CAPTURE_BALL_GO_ANGLE = 20
        self.speed = 0
        self.go_angle = 0
        self.last_go_angle = 0 # not set every loop, used only when the robot is backing of the line
        self.motor_stop = 0

        self.see_ball_cnt = 0
        self.see_ball_total = 0
        self.lock_motors_last = False
        self.no_ball_timer = Timer()
        self.no_ball_angle = -1
        self.no_ball_dist = -1

        self.no_ball_front = False
        self.no_ball_mirror = False

        self.avoid_line_timer = Timer()
        self.avoid_line = 0
        self.avoid_line_time = con.AVOID_LINE_TIME

        self.shoot_timer = Timer()

#########################################################################################################################
    def get_around_ball_angle(self, angle, circle_radius):
        """
            takes dist_to_ball - distance to ball, angle - how many degrees you want to move around the circumference, circle_radius
            outputs an angle robot should move in if he wants to stay a 'circle_radius' distance from the ball and move around it

        """
        if(angle <= 0 or angle >= 180):
            LoggerModule.log_error("Angle needs to be between 0 and 180")
            return -1

        #calculates the length of the remaining side of the triangle using the law of cosines
        third_side_length = (self.ball_dist**2 + circle_radius**2 - 2*self.ball_dist*circle_radius*math.cos(math.radians(angle)))**0.5
        #LoggerModule.log_info(third_side_length)
        #calculates the angle the robots should move in using the law of sines
        if(circle_radius < self.ball_dist):
            go_angle = math.degrees(math.asin(math.sin(math.radians(angle))*circle_radius/third_side_length))
        else:
            go_angle = 180 - math.degrees(math.asin(math.sin(math.radians(angle))*circle_radius/third_side_length))
        #LoggerModule.log_info(go_angle)
        return go_angle

    def get_approach_ball_angle(self, circle_radius):
        """
            calculates the angle for the robot, which he needs to go in, to get to a closer tangent point with a given circle_radius around the ball

        """
        go_angle = math.degrees(math.asin(circle_radius/self.ball_dist))
        #LoggerModule.log_info(go_angle)
        return go_angle

    def get_capture_ball_angle(self, err):
        """
            proporcional regulation for the missplacement(error) of the ball from the centre of the screen,
            expects values from 0 - 0,5

        """
        # wanted to go in 60 degrees at maximum displacement so 60/0,5 = 120
        # 170 at speed 100 works nicely
        p = 170*err + 0
        return p

    def get_speed_from_dist(self, max_speed, max_dist, min_speed, min_dist, dist):
        """
            calculates a speed the robot should move in at his dist from object,
            rest of the values are used to calculate linear proporcional regulation
        """
        m = (max_speed-min_speed) /(max_dist - min_dist)
        c = min_speed-m*min_dist
        p = m*dist + c
        if(p < min_speed):
            return min_speed
        if(p > max_speed):
            return max_speed
        else:
            return p

    def get_line_angle(self, active_indexes):
        if (len(active_indexes) == 0):
            return None
        SENSOR_GOTO_ANGLES = [ #angles the robot should go in if a given sensor sees the line
        38.325,	96.05, 136.375, 180, 221.125, 258.95, 319.175
        ]
        SENSOR_GOTO_VECTORS = [ #!!! carefull first is y second is x
            0.784505866, 0.620121396,
            -0.105396307, 0.994430298,
            -0.723870889, 0.689935458,
            -1, 0,
            -0.753276487, -0.657703987,
            -0.191665554, -0.981460297,
            0.756709875, -0.653750843
        ]
        v = Vector2(0,0)
        for i in active_indexes:
            v.x += SENSOR_GOTO_VECTORS[i*2+1]
            v.y += SENSOR_GOTO_VECTORS[i*2]

        return math.degrees(Vector2.get_angle(v))
#########################################################################################################################

    def get_turn_around_ball_movement(self, angle):
        angle = mathf.normalize_angle(angle)
        if angle > 180:
            angle -= 360
        angles = [180*a/max(con.TURNING_BALL_ANGLES) for a in con.TURNING_BALL_ANGLES]
        times = mathf.dists_to_times(angles, con.TURNING_BALL_SPINS)
        sign = mathf.sign(angle)
        # if abs(angle) > 90:
            # sign = -1 if self.tracker.position.x > con.FIELD_SIZE.x/2 else 1
        move_angle = mathf.normalize_angle(-sign*90)
        spin = mathf.distance_to_speed_time(abs(angle), times, con.TURNING_BALL_SPINS)[0]
        # LoggerModule.log_info(f'turning ball {angle:.2f} {sign} {spin:.2f}')
        return Movement(spin/180*math.pi*con.TURNING_BALL_RADIUS, move_angle, sign*spin)

    def go_shoot(self):
        if not self.shoot:
            self.shoot_timer.reset()
        self.shoot = True
        lidar_goal_vector = (Vector2(con.FIELD_SIZE.x/2, 0))-self.tracker.position
        angle = self.goal_center_angle if self.see_goal else math.degrees(lidar_goal_vector.get_angle())
        angle = mathf.normalize_angle(angle)
        if angle > 180:
            angle -= 360
        # LoggerModule.log_info(f'go shoot {angle:.2f} {self.see_goal} {mathf.normalize_angle(self.goal_center_angle):.2f} {mathf.normalize_angle(-self.heading):.2f}')
        turn = self.get_turn_around_ball_movement(angle)
        v = con.SHOOT_SPEED_TURNING*(1-abs(angle/180))
        if abs(angle) < 17:
            v = min(self.shoot_timer.get()/con.SHOOT_SPEED_TIME, 1)*con.SHOOT_SPEED
        else:
            self.shoot_timer.reset()
            # turn = Movement(0, 0, 0)
        if self.shoot_timer.get() > con.SHOOT_SPEED_TIME or (self.tracker.position.y < 700 and abs(angle) < 17):
            LoggerModule.log_info(f"{self.goal_center_angle:.2f} {math.degrees(lidar_goal_vector.get_angle()):.2f} {angle:.2f} {self.shoot_timer.get():.2f} {self.tracker.position.y:.2f}")
            self.kick = True
        self.movement = Movement(v, angle, 0)+turn

    def on_update(self) -> None:
        self.lock_motors = UndercarriageModule.get_button_one_status()

        # Get data from other modules
        self.goal_position = FrontCameraModule.get_goal_position()
        self.see_goal = self.goal_position[:2] != [-1, -1]
        self.kick = False
        self.dribbler_speed = con.DRIBBLER_SPEED

        self.line_sensor_values = UndercarriageModule.get_color_sensor_values()

        self.active_i = []
        # LoggerModule.log_info(f'line sensor vals: {self.line_sensor_values}')
        for i, v in enumerate(self.line_sensor_values):
            if v > self.get_line_sensors_threshold():
                self.active_i.append(i)
        self.line_angle = self.get_line_angle(self.active_i)

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
            self.ball_position = [-1, -1, 0, 0]

        if self.see_ball:
            self.no_ball_timer.reset()
            self.no_ball_angle, self.no_ball_dist = self.ball_angle, self.ball_dist
            self.no_ball_front = see_ball_front
            self.no_ball_mirror = see_ball_mirror
            #LoggerModule.log_info(f'se see {self.no_ball_angle:.2f} {self.no_ball_dist:.2f}')
        elif self.no_ball_angle >= 0 and self.no_ball_dist >= 0 and self.no_ball_timer.get() < 100:
            self.ball_angle, self.ball_dist = self.no_ball_angle, self.no_ball_dist
            self.see_ball = True
            see_ball_front = self.no_ball_front
            see_ball_mirror = self.no_ball_mirror
            #LoggerModule.log_info(f'no see {self.no_ball_angle:.2f} {self.no_ball_dist:.2f}')

        self.goal_center_angle, self.goal_dist = FrontCameraModule.get_angle_dist(self.goal_position, con.GOAL_REAL_SIZE, 1)

        self.tracker.update(self.movement, self.ball_position != self.last_ball_position, self.see_ball, ball_angle, ball_dist)
        self.heading = self.tracker.heading
        self.ball_angle = mathf.normalize_angle(math.degrees(self.tracker.get_relative_ball_position().get_angle())-self.heading)
        self.ball_dist = self.tracker.get_relative_ball_position().get_magnitude()

        # LoggerModule.log_info(f'trcker position {self.tracker.position}')
        if (self.tracker.position.x < con.SIDE_SECTOR_WIDTH or  self.tracker.position.x > con.FIELD_SIZE.x - con.SIDE_SECTOR_WIDTH):
            if self.tracker.ball_seen():
                if not self.has_ball_in(self.ball_position[0], self.ball_position[1]):
                    if not self.ball_time_started:
                        self.get_to_ball_timer.reset()
                        self.ball_time_started = True
                    # self.dribbler_speed = con.DRIBBLER_SPEED
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
                    if self.ball_time_started:
                        self.ball_time_started = False
                        LoggerModule.log_info(f"Ball got in {self.get_to_ball_timer.get():.2f} ms")
            else:
                # self.dribbler_speed = 0
                self.brake()
                a = mathf.normalize_angle(self.turn_angle-self.heading)
                if a > 180:
                    a -= 360
                if abs(a) > con.BALL_CHASING_WATCH_TRESHOLD:
                    turn_speed = mathf.sign(a)*mathf.distance_to_speed_time(abs(a), con.WATCHING_TIMES, con.WATCHING_SPINS)[0]
                    self.movement = Movement(spin=turn_speed)

            self.last_ball_position = self.ball_position

        else:
            #add a locking/unlocking mechanism for circling around the ball - probably unnecessary
            self.go_angle = 0
            self.motor_stop = 0

            # LoggerModule.log_info(f'ball_dist: {self.ball_dist:.2f}, ball_angle: {self.ball_angle:.2f}')
            if see_ball_front and self.see_ball:
                #LoggerModule.log_info("ball_dist: {}".format(self.ball_dist))
                #LoggerModule.log_info("ball_angle: {}".format(self.ball_angle))
                #LoggerModule.log_info("ball seen front")

                self.capture_ball = 0
                if(self.ball_dist > con.APPROACH_BALL_DIST_FRONT):
                    self.capture_ball = 0
                    #FIXME add going to pos behind the ball instead of directly to the ball same as around ball angle
                    self.go_angle = self.ball_angle
                    self.speed = self.get_speed_from_dist(500, con.APPROACH_BALL_DIST_FRONT+150, 150, con.APPROACH_BALL_DIST_FRONT, self.ball_dist) #200 #SPEED
                elif((self.ball_angle > con.CAPTURE_BALL_ANGLE and self.ball_angle < 360-con.CAPTURE_BALL_ANGLE and not self.capture_ball) or (self.ball_angle > con.CAPTURE_BALL_ANGLE_QUIT and self.ball_angle < 360-con.CAPTURE_BALL_ANGLE_QUIT and self.capture_ball)):
                    #dodge_angle = self.get_around_ball_angle(10, con.AROUND_BALL_DIST_FRONT)
                    #LoggerModule.log_info("dodge_angle: {}".format(dodge_angle))
                    self.capture_ball = 0
                    if(self.ball_position[0] > 0.5):
                        #LoggerModule.log_info("ball right")
                        #we want to get behind the ball there is no longer an issue about ramming the ball(at big enough AROUND_BALL_DIST)
                        angle = self.ball_angle #10
                        self.go_angle = self.ball_angle + self.get_around_ball_angle(angle, con.AROUND_BALL_DIST_FRONT)
                    else:
                        #LoggerModule.log_info("ball left")
                        angle = 360-self.ball_angle #10
                        self.go_angle = self.ball_angle - self.get_around_ball_angle(10, con.AROUND_BALL_DIST_FRONT)
                    self.speed = 100 #SPEED
                else:
                    #LoggerModule.log_info("ball capture")
                    if not self.has_ball_in(self.ball_position[0], self.ball_position[1]):
                        self.capture_ball = 1
                        if(self.ball_position[0] < 0.5):
                            self.go_angle = 360 - self.get_capture_ball_angle(abs(self.ball_position[0]- 0.5))
                        else:
                            self.go_angle = self.get_capture_ball_angle(abs(self.ball_position[0]- 0.5))
                        self.speed = 100 #SPEED
                    # else:
                    #     #LoggerModule.log_info("ball captured")
                    #     self.motor_stop = 0
                    #     self.go_shoot()

            elif see_ball_mirror and self.see_ball:
                #LoggerModule.log_info("ball_dist: {}".format(self.ball_dist))
                #LoggerModule.log_info("ball_angle: {}".format(self.ball_angle))
                #LoggerModule.log_info("ball seen mirror")

                if(self.ball_dist > con.APPROACH_BALL_DIST_MIRROR):
                    approach_ball_angle = self.get_approach_ball_angle(con.AROUND_BALL_DIST_MIRROR)
                    if(abs(self.ball_angle + approach_ball_angle - 180) < abs(self.ball_angle - approach_ball_angle - 180)):
                        self.go_angle = self.ball_angle + approach_ball_angle
                    else:
                        self.go_angle = self.ball_angle - approach_ball_angle
                    self.speed = self.get_speed_from_dist(500, con.APPROACH_BALL_DIST_MIRROR+150, 200, con.APPROACH_BALL_DIST_MIRROR, self.ball_dist) #200 #SPEED
                else:
                    # self.dribbler_speed = con.DRIBBLER_SPEED
                    #dodge_angle = self.get_around_ball_angle(10, con.AROUND_BALL_DIST_MIRROR)
                    #LoggerModule.log_info("dodge_angle: {}".format(dodge_angle))
                    #LoggerModule.log_info("ball_pos: {}".format(self.ball_position))
                    if(self.ball_position[0] > 0.5):
                        #LoggerModule.log_info("ball right")
                        self.go_angle = self.ball_angle + self.get_around_ball_angle(10, con.AROUND_BALL_DIST_MIRROR)
                    else:
                        #LoggerModule.log_info("ball left")
                        self.go_angle = self.ball_angle - self.get_around_ball_angle(10, con.AROUND_BALL_DIST_MIRROR)
                    self.speed = 200 #SPEED
            else:
                #LoggerModule.log_info("ball not seen")
                self.see_ball_cnt += 1
                # kill = 100/0
                if(self.heading > 350 or self.heading < 10):

                    self.motor_stop = 1
                else:
                    self.motor_stop = 0

            if(self.heading > 180):
                spin_heading = self.heading -360
            else:
                spin_heading = self.heading

            if(abs(spin_heading) < 10):
                spin = 0
            else:
                #FIXME tweak spin values
                if(self.speed < 300):
                    #spin = mathf.direction_to_spin(-self.heading)*0.1
                    spin = -spin_heading / 0.25
                else:
                    #spin = mathf.direction_to_spin(-self.heading)*0.05
                    spin = -spin_heading / 0.5
                # LoggerModule.log_info(f"spin: {spin}, spin_heading: {spin_heading}")
            if not self.shoot:
                self.movement = Movement(self.speed*math.pi, self.go_angle, spin)
            if(self.motor_stop):
                self.brake()

        if self.has_ball_in(self.ball_position[0], self.ball_position[1]):
            self.go_shoot()
        else:
            self.shoot = False

        center_vec = con.FIELD_SIZE/2-self.tracker.position
        dist = center_vec.get_magnitude()
        if not self.tracker.ball_seen(): # TEST THIS - GO TO CENTER TODO: or when ball in goal
            spin_angle = mathf.normalize_angle(-self.heading)
            if spin_angle > 180:
                spin_angle -= 360
            if abs(spin_angle) < 10:
                spin_angle = 0
            v = mathf.distance_to_speed_time(dist, con.BALL_CHASING_TIMES, con.BALL_CHASING_SPEEDS)[0]
            if dist < 100:
                v = 0
            center_angle = math.degrees(center_vec.get_angle())
            move_angle = 180-(self.heading+center_angle)
            # LoggerModule.log_info(f'going to center {center_angle} {dist:.2f}')
            spin = mathf.sign(spin_angle)*60
            self.movement = Movement(v, move_angle, spin)

        # Slow down based on the distance from the nearest ball, linearly
        # if(self.nearest_wall_dist() < 500):
        #     self.movement = Movement(self.movement.get_magnitude()*self.nearest_wall_dist()/500, self.movement.get_angle(), self.movement.spin)

        #add if line was seen by all sensors revert last movement
        #maybe add backing off based on the speed he enetered
        #maybe add backing off until he really exits the line (measured by again crossing the line with line sensors)
        #maybe add heading to backing off
        if(self.avoid_line and self.avoid_line_timer.get() < self.avoid_line_time):
            self.go_angle = self.last_go_angle
            self.speed = con.AVOID_LINE_SPEED
            spin = 0
            # LoggerModule.log_info(f"go_angle: {self.go_angle}")
            self.movement = Movement(self.speed*math.pi, self.go_angle, spin)
        else:
            self.avoid_line = 0
            if(self.line_angle is not None):
                self.go_angle = mathf.normalize_angle(self.line_angle+180)
                line_abs_angle = mathf.normalize_angle(self.line_angle+self.heading)
                # if self.shoot and (line_abs_angle > 300 or line_abs_angle < 60):
                #     self.kick = True
                v = self.movement.get_magnitude()
                self.avoid_line_time = max(1, v/500)*con.AVOID_LINE_TIME
                self.last_go_angle = self.go_angle
                self.speed = con.AVOID_LINE_SPEED
                spin = 0
                self.avoid_line = 1
                self.avoid_line_timer.reset()
                # LoggerModule.log_info(f"go_angle: {self.go_angle}")
                self.movement = Movement(self.speed*math.pi, self.go_angle, spin)

        if not self.lock_motors:
            UndercarriageModule.set_dribbler_speed(self.dribbler_speed)
            if self.kick:
                self.do_kick()
        else:
            self.ball_time_started = False
            self.start_time = time.time()

            self.movement = Movement(0, 0)
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

    def nearest_wall_dist(self):
        return min(self.tracker.position.x, self.tracker.position.y, con.FIELD_SIZE.x - self.tracker.position.x, con.FIELD_SIZE.y - self.tracker.position.y)

    def brake(self):
        self.movement = Movement(spin=mathf.rpm_to_angular_speed(self.braking_v))
        self.braking_v = -mathf.sign(self.braking_v)

robot = Robot()

if __name__ == "__main__":
    robot.run()
    sys.exit(robot.get_exit_code())
