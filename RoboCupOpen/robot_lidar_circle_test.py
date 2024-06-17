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

        self.last_enemy = Vector2(0, 0)
        self.index = 0
        
    def on_update(self) -> None:
        self.lock_motors = UndercarriageModule.get_button_one_status()

        self.position = LidarModule.get_position()
        self.n_field_robots = LidarModule.get_n_field_robots()
        self.field_robots = LidarModule.get_field_robot_positions()

        self.heading = UndercarriageModule.get_heading()

        self.enemy = Vector2(-1, -1)


        if self.n_field_robots > 0:
            self.enemy.x = self.field_robots[0]
            self.enemy.y = self.field_robots[1]

            self.last_enemy = self.enemy

        else:
            self.enemy = self.last_enemy


        enemy_angle = 360 - math.degrees((self.position - self.enemy).get_angle())

        #print(enemy_angle - self.heading)

        spin = mathf.direction_to_spin(enemy_angle - self.heading) * con.SPIN_GOAL_SPEED
        

        target_positons = (Vector2(460, 430), Vector2(460, 2060 - 430), Vector2(1780 - 480, 2060 - 430), Vector2(1780 - 480, 430))

        print(self.index)

        direction = self.position - target_positons[self.index % 4]

        if abs(target_positons[self.index % 4].x - self.position.x) < 50 and abs(target_positons[self.index % 4].y - self.position.y) < 50:
            self.index += 1
        else:
            self.motor_values = mathf.direction_to_motors(360 - math.degrees(direction.get_angle()) - self.heading, spin, 120)


        if self.lock_motors:
            UndercarriageModule.set_motor_values(0, 0, 0, 0)
        else:
            UndercarriageModule.set_motor_values(*self.motor_values)

    def on_stop(self):
        UndercarriageModule.set_motor_values(0, 0, 0, 0)
        UndercarriageModule.set_dribbler_speed(0)
        UndercarriageModule.set_kicker_state(False)
        time.sleep(0.1)

robot = Robot()

if __name__ == "__main__":
    robot.run()
    sys.exit(robot.get_exit_code())