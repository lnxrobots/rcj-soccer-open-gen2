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
from soccer_robot.mathf.vector2 import Vector2, Movement
import soccer_robot.constants as con
from soccer_robot.utils.timer import Timer

import math
import os
import sys
import time



class Robot(SoccerRobot):
    def get_around_bottle_angle(self,bottle_dist, angle, circle_radius):
        """
            takes dist_to_ball - distance to ball, angle - how many degrees you want to move around the circumference, circle_radius
            outputs an angle robot should move in if he wants to stay a 'circle_radius' distance from the ball and move around it

        """
        if(angle <= 0 or angle >= 180):
            LoggerModule.log_error("Angle needs to be between 0 and 180")
            return -1

        #calculates the length of the remaining side of the triangle using the law of cosines
        third_side_length = (bottle_dist**2 + circle_radius**2 - 2*bottle_dist*circle_radius*math.cos(math.radians(angle)))**0.5
        #LoggerModule.log_info(third_side_length)
        #calculates the angle the robots should move in using the law of sines
        if(circle_radius < bottle_dist):
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

    def on_start(self) -> None:
        LoggerModule.log_info("start")
        UndercarriageModule.set_motor_values(0, 0, 0, 0)

        self.movement = Movement()

        self.braking_v = 1        

        self.going_for_ball = False
        self.kicking = True


        self.ball_position = (-1, -1)
        self.ball_angle = 0

        self.position = Vector2(-1, -1)
        self.last_ball_timer = Timer()


        self.has_ball_in_timer = Timer()



        self.kick_timer = Timer()
        self.kicking = False

        self.points = [Vector2(520, 720), Vector2(con.FIELD_SIZE.x - 520, 720), Vector2(520, con.FIELD_SIZE.y - 720), Vector2(con.FIELD_SIZE.x - 520, con.FIELD_SIZE.y - 720), con.FIELD_SIZE / 2]
        self.avoid_radius = 200

    def on_update(self) -> None:
        self.lock_motors = UndercarriageModule.get_button_one_status()

        new_position = LidarModule.get_position()

        if new_position.x == -1 and new_position.y == -1:
            has_new_position = False
        else:
            self.position = new_position
            has_new_position = True

        self.heading = UndercarriageModule.get_heading()



        ball_position_front = FrontCameraModule.get_ball_position()
        ball_position_mirror = MirrorCameraModule.get_ball_position()
        see_ball_front = ball_position_front[0:2] != [-1, -1]
        see_ball_mirror = ball_position_mirror[0:2] != [-1, -1]

        self.see_ball = False

        if(see_ball_front):
            self.ball_position = ball_position_front
            self.see_ball = True
            self.ball_angle, self.ball_dist = FrontCameraModule.get_angle_dist(self.ball_position)
        elif(see_ball_mirror):
            self.ball_position = ball_position_mirror
            self.see_ball = True
            self.ball_angle, self.ball_dist = MirrorCameraModule.get_angle_dist(self.ball_position)


        if self.see_ball:
            self.last_ball_timer.reset()


        #print(enemy_angle - self.heading)


        relative_ball_pos = Vector2(math.sin(math.radians(self.ball_angle + self.heading)) * self.ball_dist, math.cos(math.radians(self.ball_angle + self.heading)) * self.ball_dist)
        absolute_ball_pos = Vector2(self.position.x + relative_ball_pos.x, self.position.y - relative_ball_pos.y)


        if not self.has_ball_in(self.ball_position[0], self.ball_position[1]):
            self.has_ball_in_timer.reset()


        if self.has_ball_in_timer.get() > 500:

            min_distance = 1999999
            min_i = 4

            for i in range(len(self.points)):
                point_distance = (self.position - self.points[i]).get_magnitude()


                if point_distance < min_distance:
                    min_distance = point_distance
                    min_i = i


            robot_center_vec = self.points[min_i] - self.position

            target_angle = 180 - math.degrees(robot_center_vec.get_angle()) - self.heading

            spin = mathf.direction_to_spin(target_angle)
            self.movement = Movement(angle = 0, speed = 0, spin = spin * 200)

            target_angle_norm = mathf.normalize_angle(target_angle)

            if target_angle_norm > 180:
                target_angle_norm -= 360
            
            if abs(target_angle_norm) < 2:
                if not self.kicking:
                    self.kicking = True
                    self.kick_timer.reset()
                elif self.kick_timer.get() > 1000:
                    self.do_kick()
                    self.kicking = False

        else:
            spin = mathf.direction_to_spin(self.ball_angle)
            self.movement = Movement(angle = 0, speed = 200, spin = spin * 200)


            for i in self.points:
                point_distance = (self.position - i).get_magnitude()


                if point_distance < self.avoid_radius:
                    

                    angle = self.get_around_bottle_angle(point_distance, 20, self.avoid_radius)
    

                    LoggerModule.log_info(angle % 360)

                    self.movement = Movement(angle = angle, speed = 200, spin = spin * 200)
                    break





            spin_angle = self.ball_angle

            if spin_angle > 180:
                spin_angle -= 360
            if abs(spin_angle) > 10:
                self.movement = Movement(angle = 0, speed = 0, spin = spin * 200)




        if not self.lock_motors:
            UndercarriageModule.set_dribbler_speed(400)
        else:
            self.movement = Movement()

            UndercarriageModule.set_dribbler_speed(0)
            UndercarriageModule.set_kicker_state(False)

        UndercarriageModule.set_robot_movement(self.movement)


    def on_stop(self):
        UndercarriageModule.set_motor_values(0, 0, 0, 0)
        UndercarriageModule.set_dribbler_speed(0)
        UndercarriageModule.set_kicker_state(False)
        time.sleep(0.1)

    def brake(self):
        self.movement = Movement(spin=mathf.rpm_to_angular_speed(self.braking_v))
        self.braking_v = -mathf.sign(self.braking_v)

    def has_ball_in(self, ball_x, ball_y):
        sx, sy, sw, sh = FrontCameraModule.camera.slot_bounding_box
        return abs(ball_x - sx) < sw / 2 and abs(ball_y - sy) < sh / 2

robot = Robot()

if __name__ == "__main__":
    robot.run()
    sys.exit(robot.get_exit_code())
    