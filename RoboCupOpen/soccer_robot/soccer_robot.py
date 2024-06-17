from soccer_robot.module.module_manager import ModuleManager

from soccer_robot.logger.logger_module import LoggerModule
from soccer_robot.interface.camera_module.front_camera_module import FrontCameraModule
from soccer_robot.interface.camera_module.mirror_camera_module import MirrorCameraModule
from soccer_robot.interface.lidar.lidar_module import LidarModule
from soccer_robot.interface.ui_module.ui_module import UIModule
from soccer_robot.interface.undercarriage_module import UndercarriageModule
from soccer_robot.visualization.visualizer_module import VisualizationModule
from soccer_robot.utils.timer import Timer

import soccer_robot.constants as con

import time
import signal
import traceback
import sys
import os
import json
import enum
import dataclasses

#https://stackoverflow.com/questions/35988/c-like-structures-in-python
#https://stackoverflow.com/questions/72604922/how-to-convert-python-dataclass-to-dictionary-of-string-literal
@dataclasses.dataclass
class CalibrationData():
    front_ball_color_bounds: list = dataclasses.field(default_factory=lambda: [0, 0, 0, 0, 0, 0])
    front_blue_goal_color_bounds: list = dataclasses.field(default_factory=lambda: [0, 0, 0, 0, 0, 0])
    front_yellow_goal_color_bounds: list = dataclasses.field(default_factory=lambda: [0, 0, 0, 0, 0, 0])

    mirror_ball_color_bounds: list = dataclasses.field(default_factory=lambda: [0, 0, 0, 0, 0, 0])
    mirror_blue_goal_color_bounds: list = dataclasses.field(default_factory=lambda: [0, 0, 0, 0, 0, 0])
    mirror_yellow_goal_color_bounds: list = dataclasses.field(default_factory=lambda: [0, 0, 0, 0, 0, 0])

    line_sensor_threshold: int = 0
    robot_speed: int = 0
    robot_shooting_speed: int = 0
    robot_rotating_speed: int = 0

class Goal(enum.Enum):
        NONE = -1
        BLUE = 0
        YELLOW = 1

class SoccerRobot():
    def __init__(self) -> None:
        self._exit_code = 0
        self._calibration_data: CalibrationData = CalibrationData()
        self._should_stop: bool = False
        self._updates_per_second: int = 120

        self._target_goal = Goal.NONE
        self._debug_mode = False
        self._status = "None"
        self._has_ball = False
        self._to_kick = False
        self._kick_requested = False
        self._kick_timer = Timer()
        self.see_ball = False
        self.ball_angle = 0
        self.ball_dist = 0
        self.goalie = False
        self.lock_motors = True

        # set CTRL + C to stop the program
        def sigint_handler(sig, frame):
            self.stop()

        signal.signal(signal.SIGINT, sigint_handler)

        # Init module manager and all modules
        LoggerModule.init()
        self.module_manager = ModuleManager()

        # Set target goal and debug mode from command line arguments
        self._target_goal, self._debug_mode = self._handle_command_line_arguments()

        FrontCameraModule.init(self._debug_mode)
        MirrorCameraModule.init(self._debug_mode)
        UndercarriageModule.init()
        LidarModule.init()
        UIModule.init()

        self.module_manager.push_module(FrontCameraModule, 1000)
        self.module_manager.push_module(MirrorCameraModule, 1000)
        self.module_manager.push_module(UndercarriageModule, 1000)
        self.module_manager.push_module(LidarModule, 1000)
        self.module_manager.push_module(UIModule, 1)

        #self.module_manager.push_module(BluetoothModule)

        # Read/Create Calibration file
        if (not os.path.isfile(con.CALIBRATION_FILE)):
            LoggerModule.log_warning("Calibration file not found ({}), creating a default one".format(con.CALIBRATION_FILE))
            self._serialize_calibration_data(con.CALIBRATION_FILE)
        else:
            file = open(con.CALIBRATION_FILE)

            try:
                calibration_file_json = json.load(file)

                self._calibration_data.front_ball_color_bounds = calibration_file_json["front_ball_color_bounds"]
                self._calibration_data.front_blue_goal_color_bounds = calibration_file_json["front_blue_goal_color_bounds"]
                self._calibration_data.front_yellow_goal_color_bounds = calibration_file_json["front_yellow_goal_color_bounds"]

                self._calibration_data.mirror_ball_color_bounds = calibration_file_json["mirror_ball_color_bounds"]
                self._calibration_data.mirror_blue_goal_color_bounds = calibration_file_json["mirror_blue_goal_color_bounds"]
                self._calibration_data.mirror_yellow_goal_color_bounds = calibration_file_json["mirror_yellow_goal_color_bounds"]

                self._calibration_data.line_sensor_threshold = calibration_file_json["line_sensor_threshold"]
                self._calibration_data.robot_speed = calibration_file_json["robot_speed"]
                self._calibration_data.robot_shooting_speed = calibration_file_json["robot_shooting_speed"]
                self._calibration_data.robot_rotating_speed = calibration_file_json["robot_rotating_speed"]

            except Exception as e:
                LoggerModule.log_warning("Calibration file is broken, creating a default one: {}".format(con.CALIBRATION_FILE))
                self._serialize_calibration_data(con.CALIBRATION_FILE)

            file.close()

        # Set ball and goal based on calibration data
        FrontCameraModule.set_ball_color_bounderies(*self._calibration_data.front_ball_color_bounds)
        MirrorCameraModule.set_ball_color_bounderies(*self._calibration_data.mirror_ball_color_bounds)
        self.set_goal(self._target_goal, FrontCameraModule)
        self.set_goal(self._target_goal, MirrorCameraModule)

        if (self._debug_mode):
            # Visualizer has to be inited with calibration data, so they can be send to client
            VisualizationModule.init(
                self._calibration_data.front_ball_color_bounds,
                self._calibration_data.front_blue_goal_color_bounds,
                self._calibration_data.front_yellow_goal_color_bounds,
                self._calibration_data.line_sensor_threshold,
                (
                    self._calibration_data.robot_speed,
                    self._calibration_data.robot_shooting_speed,
                    self._calibration_data.robot_rotating_speed
                )
            )
            self.module_manager.push_module(VisualizationModule, 60)

    # Called once at the beggining
    def on_start(self) -> None:
        pass

    # Called in a loop till _should_stop
    def on_update(self) -> None:
        pass

    # Called once at the program stop
    def on_stop(self) -> None:
        pass

    def run(self) -> None:
        self.module_manager.start_modules()

        try:
            self.on_start()

            while (not self._should_stop):
                update_start = time.time()


                if (self._debug_mode and VisualizationModule.is_new_calibration()):
                    calibration = VisualizationModule.get_new_calibration()

                    calibration_target = calibration[0]
                    calibration_data = calibration[1]

                    if (calibration_target == 0):
                        self._calibration_data.front_ball_color_bounds = calibration_data
                        FrontCameraModule.set_ball_color_bounderies(*self._calibration_data.front_ball_color_bounds)
                    elif (calibration_target == 1):
                        self._calibration_data.front_blue_goal_color_bounds = calibration_data
                        self.set_goal(self._target_goal, FrontCameraModule)
                    elif (calibration_target == 2):
                        self._calibration_data.front_yellow_goal_color_bounds = calibration_data
                        self.set_goal(self._target_goal, FrontCameraModule)
                    elif (calibration_target == 3):
                        self._calibration_data.mirror_ball_color_bounds = calibration_data
                        MirrorCameraModule.set_ball_color_bounderies(*self._calibration_data.mirror_ball_color_bounds)
                    elif (calibration_target == 4):
                        self._calibration_data.mirror_blue_goal_color_bounds = calibration_data
                        self.set_goal(self._target_goal, MirrorCameraModule)
                    elif (calibration_target == 5):
                        self._calibration_data.mirror_yellow_goal_color_bounds = calibration_data
                        self.set_goal(self._target_goal, MirrorCameraModule)
                    elif (calibration_target == 6):
                        self._calibration_data.line_sensor_threshold = calibration_data
                    elif (calibration_target == 7):
                        self._calibration_data.robot_speed = calibration_data[0]
                        self._calibration_data.robot_shooting_speed = calibration_data[1]
                        self._calibration_data.robot_rotating_speed = calibration_data[2]

                    self._serialize_calibration_data(con.CALIBRATION_FILE)

                # Break if any module fails
                if not self.module_manager.update():
                    self._exit_code = 3
                    break

                self.set_status("None")
                self.on_update()

                if self._to_kick:
                    self._to_kick = False
                    self._kick_timer.reset()
                if self._kick_requested:
                    if self._kick_timer.get() > con.KICK_START_TIME+con.KICK_TIME:
                        UndercarriageModule.set_kicker_state(False)
                        self._kick_requested = False
                    else:
                        UndercarriageModule.set_dribbler_speed(0)
                        if self._kick_timer.get() > con.KICK_START_TIME:
                            UndercarriageModule.set_dribbler_speed(0)
                            UndercarriageModule.set_kicker_state(True)

                if UndercarriageModule.get_to_terminate():
                    self.stop()

                #UIModule.set_status(self._status)
                #UIModule.set_goalie(self.goalie)

                """
                BluetoothModule.set_own_data(
                    self._has_ball,
                    self.tracker.heatmap,
                    self.see_ball,
                    (self.ball_angle, self.ball_dist),
                    self.goalie,
                    self.lock_motors
                )
                """

                sleep_duration = 1 / (self._updates_per_second + 10) - (time.time() - update_start)

                if (sleep_duration > 0):
                    time.sleep(sleep_duration)

        except Exception as e:
            LoggerModule.log_critical("A critical error has occured: ")
            LoggerModule.log_critical(traceback.format_exc())

            self._exit_code = 2

        self.on_stop()

        LoggerModule.log_info("Terminating, waiting for all processes to terminate")
        self.module_manager.terminate()

    # Stops the robot and the program
    def stop(self):
        self._should_stop = True

    def do_kick(self):
        if not self._kick_requested:
            self._kick_requested = True
            self._to_kick = True

    def set_goal(self, goal: Goal, target_camera) -> None:
        self._target_goal = goal

        if (goal == Goal.BLUE):
            target_camera.set_enable_goal(True)
            target_camera.set_goal_color_bounderies(*self._calibration_data.front_blue_goal_color_bounds)
        elif (goal == Goal.YELLOW):
            target_camera.set_enable_goal(True)
            target_camera.set_goal_color_bounderies(*self._calibration_data.front_yellow_goal_color_bounds)
        elif (goal == Goal.NONE):
            target_camera.set_enable_goal(False)

    def get_line_sensors_threshold(self) -> int:
        return self._calibration_data.line_sensor_threshold

    def get_robot_speed(self) -> int:
        return self._calibration_data.robot_speed

    def get_robot_shooting_speed(self) -> int:
        return self._calibration_data.robot_shooting_speed

    def get_robot_rotating_speed(self) -> int:
        return self._calibration_data.robot_rotating_speed

    def set_has_ball(self, has_ball):
        self._has_ball = has_ball

    def set_status(self, status: str) -> None:
        self._status = status

    def get_status(self) -> str:
        return self._status

    def is_debug_mode(self) -> bool:
        return self._debug_mode

    def get_exit_code(self) -> int:
        return self._exit_code

    def _serialize_calibration_data(self, path) -> None:
        file = open(path, "w")
        file.write(json.dumps(dataclasses.asdict(self._calibration_data), indent = True))
        file.close()

    def _handle_command_line_arguments(self) -> tuple:
        target_goal = Goal.NONE
        debug_mode = True

        if (len(sys.argv) > 1):
            if (sys.argv[1].lower() == "blue"):
                target_goal = Goal.BLUE
            elif (sys.argv[1].lower() == "yellow"):
                target_goal = Goal.YELLOW
            elif (sys.argv[1].lower() == "none"):
                target_goal = Goal.NONE
            else:
                target_goal = Goal.BLUE
                LoggerModule.log_warning("Unknown goal: {}, defaulting to blue".format(sys.argv[1]))
        else:
            target_goal = Goal.BLUE
            LoggerModule.log_warning("Goal was not set, defaulting to blue")

        if (len(sys.argv) > 2):
            if (sys.argv[2].lower() == "true"):
                debug_mode = True
            elif (sys.argv[2].lower() == "false"):
                debug_mode = False
            else:
                debug_mode = True
                LoggerModule.log_warning("debug_mode should be True or False ({}), defaulting to True".format(sys.argv[2]))
        else:
            LoggerModule.log_warning("debug_mode was not set, defaulting to True")

        return (target_goal, debug_mode)
