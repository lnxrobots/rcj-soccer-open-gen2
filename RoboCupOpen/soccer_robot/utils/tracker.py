import soccer_robot.constants as con
from soccer_robot.mathf import mathf
from soccer_robot.mathf.vector2 import Vector2, Movement
from soccer_robot.utils.timer import Timer
from soccer_robot.logger.logger_module import LoggerModule
from soccer_robot.interface.lidar.lidar_module import LidarModule
from soccer_robot.interface.undercarriage_module import UndercarriageModule

import math
import time
import statistics

class Tracker:
    """Class for estimating position and sensor data of the robot"""

    def __init__(self):
        self.position = Vector2(0, 0)
        self.heading = 0

        self.ball_position = Vector2(0, 0)
        self.ball_velocity = Vector2(0, 0)
        self.ball_pos_history = []
        self.ball_seen_timer = Timer()
        self.ball_vel_timer = Timer()

        self._timer = Timer()
        self._frame_timer = Timer()
        self._last_lidar = Vector2(0, 0)
        self._lidar_timer = Timer()
        self._last_compass = 0
        self._last_ball_pos = Vector2(0, 0)

    def update(self, movement: Movement, new_frame, see_ball, ball_angle, ball_dist):
        """Needs to be called evety frame to update the position of the robot"""

        since_update = self._timer.get()/1000
        self._timer.reset()

        # Calculate robot movement since last update
        turned = movement.spin * since_update
        turned_rad = math.radians(turned)
        distance = movement.get_magnitude() * since_update
        relative_pos = Vector2(movement.x, movement.y)
        if turned_rad != 0:
            r = distance/turned_rad
            relative_pos = r*Vector2(1-math.cos(turned_rad), math.sin(turned_rad)).rotated(movement.get_angle())

        # Update position
        # self.position += relative_pos
        # self.heading = mathf.normalize_angle(self.heading + turned)

        # Or override by lidar...
        lidar_pos = LidarModule.get_position()
        delta_dist = (lidar_pos - self._last_lidar).get_magnitude()
        if lidar_pos != Vector2(-1, -1) and lidar_pos != self._last_lidar and delta_dist/(self._lidar_timer.get()/1000) < con.ROBOT_MAX_SPEED:
            self._lidar_timer.reset()
            self.position = lidar_pos
            self._last_lidar = lidar_pos
        # ...and compass
        compass = UndercarriageModule.get_heading()
        if compass != self._last_compass:
            self.heading = compass
            self._last_compass = compass

        # Remove old ball velocities
        new_ball_vs = []
        for t, v in self.ball_pos_history:
            if time.time()-t < con.BALL_POS_HISTORY_TIME/1000:
                new_ball_vs.append((t, v))
        self.ball_pos_history = new_ball_vs

        # Update ball position
        if new_frame:
            since_frame = self._frame_timer.get()/1000
            new_ball_pos = Vector2(a=math.radians(ball_angle+self.heading), m=ball_dist)+self.position
            if see_ball:# and ((self.ball_velocity-last_frame_v)/since_frame).get_magnitude() < 2000:
                self.ball_pos_history.append((time.time(), new_ball_pos))
                if len(self.ball_pos_history) > 1:
                    half = int(len(self.ball_pos_history)/2)
                    ts, vs = zip(*self.ball_pos_history)
                    p1 = mathf.vector_mean(vs[:half])
                    t1 = statistics.mean(ts[:half])
                    p2 = mathf.vector_mean(vs[half:])
                    t2 = statistics.mean(ts[half:])
                    self.ball_velocity = (p1-p2)/(t2-t1)
                self.ball_position = new_ball_pos
                self._last_ball_pos = self.ball_position
                self.ball_seen_timer.reset()
            elif self.ball_seen():
                self.ball_velocity = Vector2(0, 0)
            else: # Ball lost
                self.ball_position = Vector2(0, 0)
                self.ball_velocity = Vector2(0, 0)
                self._last_ball_pos = Vector2(0, 0)
            self._frame_timer.reset()

    def ball_seen(self):
        return self.ball_seen_timer.get() <= con.BALL_LOST_TIMEOUT

    def get_relative_ball_position(self):
        return self.ball_position - self.position

    def get_ball_catch_pos_time(self, times, speeds):
        catch_pos = self.ball_position
        for _ in range(1, 100):
            dist = catch_pos.get_magnitude()
            time = mathf.distance_to_speed_time(dist, times, speeds)[1]
            catch_pos = self.ball_velocity * time + self.ball_position
        LoggerModule.log_info(f"{dist:.2f} {time:.2f} {self.ball_velocity.get_magnitude():.2f}")
        return catch_pos, time
