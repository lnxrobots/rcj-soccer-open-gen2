from soccer_robot.soccer_robot import SoccerRobot

from soccer_robot.logger.logger_module import LoggerModule
from soccer_robot.interface.camera_module.camera_module import CameraModule
from soccer_robot.interface.undercarriage_module import UndercarriageModule
from soccer_robot.interface.compass_module import CompassModule
from soccer_robot.interface.ui_module.ui_module import UIModule

from soccer_robot.visualization.visualizer_module import VisualizationModule

from soccer_robot.mathf import mathf
from soccer_robot.mathf.vector2 import Vector2
import soccer_robot.constants as con
from soccer_robot.utils.timer import Timer

import math
import os
import sys
import time

KICKOFF_PUSH_TIME = 0

class Robot(SoccerRobot):
    def on_start(self) -> None:
        LoggerModule.log_debug("Main Process pid: " + str(os.getpid()))

        self.motor_values = (0, 0, 0, 0)
        UndercarriageModule.set_motor_values(*self.motor_values)

        self.ball_position = [0, 0, 0, 0]
        self.goal_position = [0, 0, 0, 0]

        self.ball_angle = 0
        self.ball_dist = 0
        self.goal_center_angle = 0
        self.goal_dist = 0

        self.heading = 0

        self.active_i = []
        self.line = []

        self.speed = 0
        self.shooting_speed = 0
        self.rotating_speed = 0

        self.backwards_speed = 0
        self.line_avoid_speed = 0

        self.seen_line = False
        self.anti_line_angle = 0
        self.line_timer = Timer()
        self.line_avoid_timer = con.LINE_AVOID_FAST

        self.start_button_timer = Timer()

        self.rotating = False
        self.rotating_timer = Timer()

    def on_update(self) -> None:
        self.motor_values = (0, 0, 0, 0)

        # Set speeds from calibration
        self.speed = self.get_robot_speed()
        self.shooting_speed = self.get_robot_shooting_speed()
        self.rotating_speed = self.get_robot_rotating_speed()

        self.backwards_speed = self.speed
        self.line_avoid_speed = self.speed


        # Get data from other modules
        self.ball_position = CameraModule.get_ball_position()
        self.goal_position = CameraModule.get_goal_position()

        self.heading = CompassModule.get_heading()
        sensor_values = UndercarriageModule.get_color_sensor_values()
        lock_motors = UIModule.get_button_one_status()

        self.active_i = []
        for i, v in enumerate(sensor_values):
            if v > self.get_line_sensors_threshold():
                self.active_i.append(i)
        self.line = mathf.get_line_pos(self.active_i)

        see_ball = self.ball_position[2] > con.BALL_MIN_SIZE[0] and self.ball_position[3] > con.BALL_MIN_SIZE[1]
        see_goal = self.goal_position[2] > con.GOAL_MIN_SIZE[0] and self.goal_position[3] > con.GOAL_MIN_SIZE[1]

        self.ball_angle, self.ball_dist = mathf.img_coords_to_angle_dist(self.ball_position)
        self.goal_center_angle, self.goal_dist = mathf.img_coords_to_angle_dist(self.goal_position, con.GOAL_REAL_SIZE, 1)

        m = max(self.tracker.heatmap)
        sectors = []
        for i, v in enumerate(self.tracker.heatmap):
            if v == m:
                sectors.append(i)

        robot_sector = sectors[(len(sectors)-1)//2]
        robot_pos = Vector2(*con.TRACKER_HEATMAP_POINTS[robot_sector])
        new_index = 7#int(time.time()/10) % 9
        target_pos = Vector2(*con.TRACKER_HEATMAP_POINTS[new_index])
        move_vector = target_pos - robot_pos
        result_angle = math.degrees(move_vector.get_angle()) - self.heading
        result_angle_rounded = round(result_angle / 45) * 45
        spin = mathf.direction_to_spin(-self.heading) * 0.6
        self.motor_values = mathf.direction_to_motors(result_angle_rounded, spin)
        if robot_sector == new_index:# and self.tracker.heatmap[robot_sector] >= 0.55:
            self.motor_values = [0, 0, 0, 0]

        LoggerModule.log_debug(f'<heatmap> {time.time()} {",".join(map(str, self.tracker.heatmap))}')

        # Line check
        self.check_lines()

        self.tracker.update(
            self.heading, self.anti_line_angle+180, self.seen_line,
            see_goal, self.goal_center_angle, self.goal_dist,
            self.motor_values, lock_motors
        )

        VisualizationModule.set_position_heatmap(self.tracker.heatmap)

        self.set_has_ball(self.has_ball_in(*self.ball_position[0:2]))

        # LoggerModule.log_debug("compass: {}, goal_pos: {}, sensor_values: {}, motor_values: {}, heatmap: {}".format(self.heading, self.goal_position, sensor_values, self.motor_values, self.tracker.heatmap))
        self.set_status(f'{round(self.ball_dist)} {robot_sector}')
        if (not lock_motors):
            if (con.ROBOT_PRECISE_BALL_MOVE):
                self.motor_values = (self.motor_values[0], self.motor_values[1], self.motor_values[2], int(self.motor_values[3] * 1.1))

            UndercarriageModule.set_motor_values(*self.motor_values)
        else:
            UndercarriageModule.set_motor_values(0, 0, 0, 0)
            self.seen_line = False
            self.start_button_timer.reset()

    def on_stop(self):
        UndercarriageModule.set_motor_values(0, 0, 0, 0)

    def check_lines(self):
        if self.seen_line:
            self.set_status("avoiding line")
            self.motor_values = mathf.direction_to_motors(self.anti_line_angle, 0, self.line_avoid_speed)

            if (self.line_timer.get() > self.line_avoid_timer):
                self.seen_line = False

        elif (len(self.active_i) > 0):
            self.set_status("avoiding line")
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
            result_angle_rounded = round(result_angle / 45) * 45
            self.anti_line_angle = result_angle_rounded

            self.motor_values = mathf.direction_to_motors(result_angle_rounded, 0, self.speed)


    def has_ball_in(self, ball_x, ball_y):
        sx, sy, sw, sh = con.SLOT_BOUNDING_BOX
        return abs(ball_x - sx) < sw / 2 and abs(ball_y - sy) < sh / 2

robot = Robot()


if __name__ == "__main__":
    robot.run()
    sys.exit(robot.get_exit_code())
