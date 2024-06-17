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

SPEED = 200

class Robot(SoccerRobot):
    def on_start(self) -> None:
        LoggerModule.log_info("start")
        # time.sleep(10)
        UndercarriageModule.set_motor_values(0, 0, 0, 0)

        self.camera_module = FrontCameraModule
        self.last_checked_front_timer = Timer()
        self.front_timer = Timer()
        self.checking_front_camera = False

        self.motor_values = (0, 0, 0, 0)
        self.dribbler_speed = 0 #400

        self.ball_angle = 0
        self.ball_dist = 0

        self.capture_ball = 0
        #self.CAPTURE_BALL_GO_ANGLE = 20
        self.speed = SPEED
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

    def on_update(self) -> None:
        self.lock_motors = UndercarriageModule.get_button_one_status()
        
        if self.lock_motors != self.lock_motors_last:
            if not self.lock_motors:
                self.see_ball_cnt = 0
                self.see_ball_total = 0
        self.lock_motors_last = self.lock_motors
        self.see_ball_total += 1

        # Get data from other modules
        max_angle = FrontCameraModule.camera.center_fov / 2 - 15

        self.heading = UndercarriageModule.get_heading()
        self.line_sensor_values = UndercarriageModule.get_color_sensor_values()
        
        self.active_i = []
        LoggerModule.log_info(f'line sensor vals: {self.line_sensor_values}')
        for i, v in enumerate(self.line_sensor_values):
            if v > self.get_line_sensors_threshold():
                self.active_i.append(i)
        self.line_angle = self.get_line_angle(self.active_i)

        self.camera_module = FrontCameraModule # MirrorCameraModule

        ball_position_front = FrontCameraModule.get_ball_position()
        LoggerModule.log_info(f'ball pos front: {ball_position_front}')
        ball_position_mirror = MirrorCameraModule.get_ball_position()
        LoggerModule.log_info(f'ball pos mirro: {ball_position_mirror}')
        see_ball_front = ball_position_front[0:2] != [-1, -1]
        see_ball_mirror = ball_position_mirror[0:2] != [-1, -1]
        if(see_ball_front):
            self.ball_position = ball_position_front
            self.see_ball = True
            self.ball_angle, self.ball_dist = FrontCameraModule.get_angle_dist(self.ball_position)
        elif(see_ball_mirror):
            self.ball_position = ball_position_mirror
            self.see_ball = True
            self.ball_angle, self.ball_dist = MirrorCameraModule.get_angle_dist(self.ball_position)
        else:
            self.ball_position = ball_position_mirror
            self.see_ball = False
            self.ball_angle, self.ball_dist = MirrorCameraModule.get_angle_dist(self.ball_position)

        #LoggerModule.log_info(f'see ball: {self.see_ball}')
        #LoggerModule.log_info(f'see ball front: {see_ball_front}')
        #LoggerModule.log_info(f'see ball mirror: {see_ball_mirror}')


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

        #add a locking/unlocking mechanism for circling around the ball - probably unnecessary
        self.go_angle = 0
        self.motor_stop = 0

        LoggerModule.log_info(f'ball_dist: {self.ball_dist:.2f}, ball_angle: {self.ball_angle:.2f}')
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
                else:
                    #LoggerModule.log_info("ball captured")
                    self.motor_stop = 1

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

        if(self.heading > 350 or self.heading < 10):
                spin = 0
        else:
            if(self.speed < 300):
                spin = mathf.direction_to_spin(-self.heading)*0.1
            else:
                spin = mathf.direction_to_spin(-self.heading)*0.05

        if(self.avoid_line and self.avoid_line_timer.get() < con.AVOID_LINE_TIME):
            self.go_angle = self.last_go_angle
            self.speed = con.AVOID_LINE_SPEED
            self.motor_stop = 0
            spin = 0
        else:
            self.avoid_line = 0
            if(self.line_angle != None):
                self.go_angle = mathf.normalize_angle(self.line_angle+180)
                self.last_go_angle = self.go_angle
                self.speed = con.AVOID_LINE_SPEED
                spin = 0
                self.motor_stop = 0
                self.avoid_line = 1
                self.avoid_line_timer.reset()


        LoggerModule.log_info("go_angle: {}".format(self.go_angle))
        if(not self.motor_stop):
            self.motor_values = mathf.direction_to_motors(self.go_angle, spin, self.speed)
        else:
            self.motor_values = (0, 0, 0, 0)

        #LoggerModule.log_info(f"motor value {self.motor_values}")
        if not self.lock_motors and not self.motor_stop:
            #LoggerModule.log_info(f"see ball cnt: {self.see_ball_cnt}, total: {self.see_ball_total}, {self.see_ball_cnt/self.see_ball_total*100:.2f}%")
            pass
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
