from soccer_robot.mathf.vector2 import Vector2
from soccer_robot.mathf import mathf

import socket
import math

hostname = socket.gethostname()

ROBOT_NAMES = ["lnx-robot3", "lnx-robot4"]

try:
    ROBOT_INDEX = ROBOT_NAMES.index(hostname)
except ValueError:
    raise ValueError("Unknown hostname: {}".format(hostname))

if ROBOT_INDEX == 0:
    from .constants_robot1 import *
elif ROBOT_INDEX == 1:
    from .constants_robot2 import *
else:
    raise ImportError("No constants file for {}".format(hostname))


WATCHING_TIMES = mathf.dists_to_times(WATCHING_ANGLES, WATCHING_SPINS)
BALL_CHASING_TIMES = mathf.dists_to_times(BALL_CHASING_DISTS, BALL_CHASING_SPEEDS)

SENSOR_DATA = list(zip(SENSOR_ANGLES, SENSOR_DIST))
MOTOR_SPEED_TO_MOTION = MOTOR_SPEED_TO_RPM/60 * ROBOT_WHEEL_D*math.pi
