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
    def on_start(self) -> None:
        LoggerModule.log_info("start")
        UndercarriageModule.set_motor_values(0, 0, 0, 0)

        self.movement = Movement()

        self.braking_v = 1        

        self.going_for_ball = False
        self.kicking = True

        self.kicking_timer = Timer()

        self.ball_position = (-1, -1)
        self.ball_angle = 0

        self.position = Vector2(-1, -1)

        self.last_ball_timer = Timer()

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


        circleCenter = Vector2(0, -180 + 240)
        circleRadiusSquered = 234900 * 1.2



        goal_relative_to_field_relative = Vector2(con.FIELD_SIZE.x / 2, con.FIELD_SIZE.y)

        


        circle_robot_vec = robot.position - (goal_relative_to_field_relative - circleCenter)
        circle_ball_vec = absolute_ball_pos - (goal_relative_to_field_relative - circleCenter)


        cos = (circle_robot_vec.x * circle_ball_vec.x + circle_robot_vec.y * circle_ball_vec.y) / (circle_robot_vec.get_magnitude() * circle_ball_vec.get_magnitude())

        cos = mathf.clamp(cos, -1, 1)

        robot_ball_angle = math.acos(cos)

        if circle_ball_vec.normalized().x > circle_robot_vec.normalized().x:
            robot_ball_angle *= -1
        

        newAngle = math.degrees(circle_robot_vec.get_angle()) + mathf.clamp(math.degrees(robot_ball_angle), -30, 30)

        circle_ball_vec_norm_final = Vector2(a = math.radians(mathf.clamp(math.degrees(circle_ball_vec.get_angle()), 125, 235)), m = 1)
        circle_ball_vec_norm = Vector2(a = math.radians(mathf.clamp(newAngle, 125 - 10, 235 + 10)), m = 1)

        if not self.see_ball and self.last_ball_timer.get() > 2000:
            circle_ball_vec_norm = Vector2(a = math.radians(180), m = 1)


        targetPosition = (goal_relative_to_field_relative - circleCenter) + circle_ball_vec_norm * math.sqrt(circleRadiusSquered)  #goal_relative_to_field_relative - (circleCenter + Vector2(a = math.radians(angle), m = math.sqrt(circleRadiusSquered)))
        direction = self.position - targetPosition

        targetPosition_final = (goal_relative_to_field_relative - circleCenter) + circle_ball_vec_norm_final * math.sqrt(circleRadiusSquered)
        direction_final = self.position - targetPosition_final

        spin = mathf.direction_to_spin(360 - (360 - math.degrees(circle_robot_vec.get_angle()) - self.heading))


        if abs(targetPosition.x - self.position.x) < 50 and abs(targetPosition.y - self.position.y) < 50 or self.going_for_ball:
            if relative_ball_pos.y > 0 and self.ball_dist < 400 and self.last_ball_timer.get() < 100:
                spin = mathf.direction_to_spin(self.ball_angle)
                self.movement = Movement(angle = 0, speed = 400, spin = spin * 200)
                self.going_for_ball = True

                UndercarriageModule.set_kicker_state(False)

                if self.has_ball_in(self.ball_position[0], self.ball_position[1]) or self.kicking:
                    if not self.kicking:
                        self.kicking = True
                        self.kicking_timer.reset()

                    if self.kicking_timer.get() > 500 or self.position.y < con.FIELD_SIZE.y / 2:
                        UndercarriageModule.set_kicker_state(True)
                        self.kicking = False
                    
                    else:
                        direction = Vector2(con.FIELD_SIZE.x / 2, 0) - self.position
                    
                        spin = mathf.direction_to_spin(math.degrees(direction.get_angle()) + self.heading)
                        self.movement = Movement(angle = 0, speed = 700, spin = spin * 80)
                        self.going_for_ball = True



            else:
                self.brake()
                self.going_for_ball = False
            
        else:
            speed = mathf.clamp(mathf.remap(0, 400, 60, 800, direction_final.get_magnitude()), 60, 800)

            self.movement = Movement(angle = 360 - math.degrees(direction.get_angle()) - self.heading, speed = speed, spin = spin * 50)

        if not has_new_position:
            self.movement = Movement()



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
    