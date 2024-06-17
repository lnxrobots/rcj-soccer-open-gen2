from soccer_robot.soccer_robot import SoccerRobot

from soccer_robot.logger.logger_module import LoggerModule
from soccer_robot.interface.undercarriage_module import UndercarriageModule

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

class Robot(SoccerRobot):
    def on_start(self) -> None:
        self.move_state = 0
        
        #0 - movement_spin, 1 - movement_forward, 2 - movemen_lateral, 3 - movement_brake_test, 4 - movement_spin_kick
        self.MOVEMENT_SELECT = 2
        self.MOVEMENT_DRIVE_SPEED = 70
        self.MOVEMENT_DRIBBLER_SPEED = 400
        self.MOVEMENT_DRIBBLER_PAUSE = 5000
        self.movement_state = 0
        self.movement_counter = 0   
        self.movement_timer = Timer()

    def movement_spin(self):
        mt = 10000 + self.MOVEMENT_DRIBBLER_PAUSE
        mv = self.MOVEMENT_DRIVE_SPEED
        if(self.movement_timer.get() < self.MOVEMENT_DRIBBLER_PAUSE):
            motor_vals = (0, 0, 0, 0)
            UndercarriageModule.set_motor_values(*motor_vals)
            UndercarriageModule.set_dribbler_speed(self.MOVEMENT_DRIBBLER_SPEED)
        elif(self.movement_timer.get() < mt):
            motor_vals = (mv, mv, mv, mv)
            UndercarriageModule.set_motor_values(*motor_vals)
            #LoggerModule.log_info("motor vals: {}".format(motor_vals))
        elif(self.movement_timer.get() < mt+700):
            motor_vals = (0, 0, 0, 0)
            UndercarriageModule.set_motor_values(*motor_vals)
        else:
            UndercarriageModule.set_dribbler_speed(0)
            return 1
        return 0
    
    def movement_spin_kick(self):
        mt = 700 + self.MOVEMENT_DRIBBLER_PAUSE
        mv = self.MOVEMENT_DRIVE_SPEED
        if(self.movement_timer.get() < self.MOVEMENT_DRIBBLER_PAUSE):
            UndercarriageModule.set_dribbler_speed(self.MOVEMENT_DRIBBLER_SPEED)
        elif(self.movement_timer.get() < mt):
            motor_vals = (mv, mv, mv, mv)
            UndercarriageModule.set_motor_values(*motor_vals)
            #LoggerModule.log_info("motor vals: {}".format(motor_vals))
        elif(self.movement_timer.get() < mt+500):
            UndercarriageModule.set_dribbler_speed(0)
            UndercarriageModule.set_kicker_state(1)
        else:
            motor_vals = (0, 0, 0, 0)
            UndercarriageModule
            UndercarriageModule.set_motor_values(*motor_vals)
            UndercarriageModule.set_dribbler_speed(0)
            UndercarriageModule.set_kicker_state(1)
            return 1
        return 0

    def movement_forward(self):
        mt = 0 + self.MOVEMENT_DRIBBLER_PAUSE
        if(self.movement_timer.get() < self.MOVEMENT_DRIBBLER_PAUSE):
            UndercarriageModule.set_dribbler_speed(self.MOVEMENT_DRIBBLER_SPEED)
        elif(self.movement_timer.get() < mt):
            motor_vals = mathf.direction_to_motors(0, 0, self.MOVEMENT_DRIVE_SPEED)
            UndercarriageModule.set_motor_values(*motor_vals)
            UndercarriageModule.set_dribbler_speed(self.MOVEMENT_DRIBBLER_SPEED)
            #LoggerModule.log_info("motor vals: {}".format(motor_vals))
        else:
            motor_vals = (0, 0, 0, 0)
            UndercarriageModule
            UndercarriageModule.set_motor_values(*motor_vals)
            UndercarriageModule.set_dribbler_speed(0)
            return 1
        return 0
    
    def movement_lateral(self):
        mt = 10000 
        if(self.movement_timer.get() < self.MOVEMENT_DRIBBLER_PAUSE):
            UndercarriageModule.set_dribbler_speed(self.MOVEMENT_DRIBBLER_SPEED)
        elif(self.movement_timer.get() < mt + self.MOVEMENT_DRIBBLER_PAUSE):
            motor_vals = mathf.direction_to_motors(90, 0, self.MOVEMENT_DRIVE_SPEED)
            UndercarriageModule.set_motor_values(*motor_vals)
            UndercarriageModule.set_dribbler_speed(self.MOVEMENT_DRIBBLER_SPEED)
        elif(self.movement_timer.get() < mt*2 + self.MOVEMENT_DRIBBLER_PAUSE):
            motor_vals = mathf.direction_to_motors(270, 0, self.MOVEMENT_DRIVE_SPEED)
            UndercarriageModule.set_motor_values(*motor_vals)
            UndercarriageModule.set_dribbler_speed(self.MOVEMENT_DRIBBLER_SPEED)
        elif(self.movement_timer.get() < mt*3 + self.MOVEMENT_DRIBBLER_PAUSE):
            motor_vals = (0, 0, 0, 0)
            UndercarriageModule.set_motor_values(*motor_vals)
            UndercarriageModule.set_dribbler_speed(self.MOVEMENT_DRIBBLER_SPEED)
        elif(self.movement_timer.get() < mt*4 + self.MOVEMENT_DRIBBLER_PAUSE):
            motor_vals = mathf.direction_to_motors(270, 0, self.MOVEMENT_DRIVE_SPEED)
            UndercarriageModule.set_motor_values(*motor_vals)
            UndercarriageModule.set_dribbler_speed(self.MOVEMENT_DRIBBLER_SPEED)
        elif(self.movement_timer.get() < mt*5 + self.MOVEMENT_DRIBBLER_PAUSE):
            motor_vals = mathf.direction_to_motors(90, 0, self.MOVEMENT_DRIVE_SPEED)
            UndercarriageModule.set_motor_values(*motor_vals)
            UndercarriageModule.set_dribbler_speed(self.MOVEMENT_DRIBBLER_SPEED)
        else:
            motor_vals = (0, 0, 0, 0)
            UndercarriageModule
            UndercarriageModule.set_motor_values(*motor_vals)
            UndercarriageModule.set_dribbler_speed(0)
            return 1
        return 0
    
    def movement_brake_test(self):
        mt = 1000 + self.MOVEMENT_DRIBBLER_PAUSE
        if(self.movement_timer.get() < self.MOVEMENT_DRIBBLER_PAUSE):
            UndercarriageModule.set_dribbler_speed(self.MOVEMENT_DRIBBLER_SPEED)
        elif(self.movement_timer.get() < mt):
            motor_vals = mathf.direction_to_motors(0, 0, self.MOVEMENT_DRIVE_SPEED)
            UndercarriageModule.set_motor_values(*motor_vals)
            UndercarriageModule.set_dribbler_speed(self.MOVEMENT_DRIBBLER_SPEED)
        elif(self.movement_timer.get() < mt*1.5):
            motor_vals = mathf.direction_to_motors(180, 0, self.MOVEMENT_DRIVE_SPEED)
            UndercarriageModule.set_motor_values(*motor_vals)
            UndercarriageModule.set_dribbler_speed(self.MOVEMENT_DRIBBLER_SPEED)
        else:
            motor_vals = (0, 0, 0, 0)
            UndercarriageModule
            UndercarriageModule.set_motor_values(*motor_vals)
            UndercarriageModule.set_dribbler_speed(0)
            return 1
        return 0
    
    def movement_switch(self):
        match self.MOVEMENT_SELECT:
            case 0:
                if(self.movement_spin() == 1):
                    return 1
                return 0
            case 1:
                if(self.movement_forward() == 1):
                    return 1
                return 0
            case 2:
                if(self.movement_lateral() == 1):
                    return 1
                return 0
            case 3:
                if(self.movement_brake_test() == 1):
                    return 1
                return 0
            case 4:
                if(self.movement_spin_kick() == 1):
                    return 1
                return 0
            case _:
                LoggerModule.log_error("nonexistent movement selected")
                return 0

    def on_update(self) -> None:
        match self.move_state:
            case 0:
                self.movement_timer.reset()
                self.move_state = 1
            case 1:
                if(self.movement_switch() == 1):
                    LoggerModule.log_info("movement finished succesfully")
                    self.move_state = 2                
            case 2:
                self.on_stop()
                self.stop()
            case _:
                LoggerModule.log_error("nonexistent state reached")
        

    def on_stop(self):
        UndercarriageModule.set_motor_values(0, 0, 0, 0)
        UndercarriageModule.set_dribbler_speed(0)
        UndercarriageModule.set_kicker_state(0)

robot = Robot()

if __name__ == "__main__":
    robot.run()
    sys.exit(robot.get_exit_code())
