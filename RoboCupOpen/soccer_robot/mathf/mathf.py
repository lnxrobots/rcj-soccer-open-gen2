import soccer_robot.constants as con
from soccer_robot.mathf.vector2 import Vector2
from soccer_robot.utils import camera

import math

def get_line_pos(active_indexes):
    active = [con.SENSOR_DATA[i] for i in active_indexes]

    if len(active) == 0:
        return None
    elif len(active) == 1:
        return active[0]

    # Add the first sensor to the list for comparison
    active_temp = active + [(active[0][0]+360, active[0][1])]

    # Find the sensors with largest angle between them - marginal sensors
    marginal = None
    max_angle = 0
    for i in range(len(active_temp)-1):
        s1 = active_temp[i]
        s2 = active_temp[i+1]
        diff_angle = s2[0]-s1[0]
        if diff_angle > max_angle:
            max_angle = diff_angle
            marginal = [s1, s2]
            # Flip the sensors to flip the angle, if angle too large for a \triangle
            if diff_angle > 180:
                marginal = [s2, s1]

    (a1, r1), (a2, r2) = marginal
    g = math.radians(a2-a1)

    # Calculate height of the triangle center-s1-s2 (using area) and angle between s1 and height
    h = r1*r2*math.sin(g) / math.sqrt(r1*r1 + r2*r2 - 2*r1*r2*math.cos(g))
    l = (a1 + math.degrees(math.acos(h/r1))) % 360
    return l, h

def dists_to_times(dists: list, speeds: list):
    times = [0]
    for i in range(len(dists)-1):
        times.append(2*(dists[i+1]-dists[i])/(speeds[i]+speeds[i+1])+times[-1])
    return times

def time_to_speed(time, times, speeds):
    segments = sorted(list(zip(times, speeds)), key=lambda x: x[0])
    for i in range(len(segments)-1):
        t1, v1 = segments[i]
        t2, v2 = segments[i+1]
        if t1 <= time <= t2:
            return (time-t1)/(t2-t1)*(v2-v1)+v1

def distance_to_speed_time(dist, times, speeds):
    segments = sorted(list(zip(times, speeds)), key=lambda x: x[0])
    dist_sum = 0
    for i in range(len(segments)-1):
        t1, v1 = segments[i]
        t2, v2 = segments[i+1]
        segment_d = (t2-t1)*(v1+v2)/2
        if dist_sum + segment_d >= dist:
            dt1 = (dist - dist_sum) / (v1 + v2) * 2
            if (t2-t1)*(v2-v1) == 0:
                return v1, t1 + dt1
            return dt1/(t2-t1)*(v2-v1)+v1, t1 + dt1
        dist_sum += segment_d
    return speeds[-1], times[-1]

def angular_speed_to_rpm(omega):
    return round(omega / 180 * con.ROBOT_WHEEL_DISTANCE / con.ROBOT_WHEEL_D * 60)

def rpm_to_angular_speed(rpm):
    return rpm / 60 * con.ROBOT_WHEEL_D / con.ROBOT_WHEEL_DISTANCE * 180

def direction_to_motors_abs(a, spin=0, v=80, should_round=True) -> list:
    """Speeds and spin in motor units. Angle in degrees"""

    a = normalize_angle(a)
    motors = [0, 0, 0, 0]
    for i, m_a in enumerate(con.MOTOR_ANGLES):
        motors[i] = (math.sin(math.radians(m_a-a))) * v - spin

    if should_round:
        return list(map(round, motors))
    return motors

def direction_to_motors(a, spin=0, v=80, should_round=True) -> list:
    """Works only for 4 motors. Spin -1 to 1"""
    #spin += 0.015

    a = normalize_angle(a)
    motors = [0, 0, 0, 0]
    for i, m_a in enumerate(con.MOTOR_ANGLES):
        motors[i] = (math.sin(math.radians(m_a-a)) - spin * 7) * v

    if should_round:
        return list(map(round, motors))
    return motors

def motors_to_vel(motors):
    result = Vector2(0, 0)
    for m, a in zip(motors, [315, 45, 135, 225]):
        result += Vector2(a=math.radians(a), m=m)
    return result/4 * con.MOTOR_SPEED_TO_MOTION

def direction_to_spin(a):
    return clamp(((a + 180) % 360 - 180) / con.SPIN_MAX_SPEED_ANGLE, -1.0, 1.0)

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def lerp(a: float, b: float, t: float) -> float:
    return (1.0 - t) * a + b * t

def inv_lerp(a: float, b: float, v: float) -> float:
    return (v - a) / (b - a)

def remap(iMin: float, iMax: float, oMin: float, oMax: float, v: float):
    t: float = inv_lerp(iMin, iMax, v)
    return lerp(oMin, oMax, t)

def sign(value) -> int:
    if value > 0:
        return 1
    elif value < 0:
        return -1
    return 0

def is_angle_between(angle, first, second):
    second = (second - first) % 360
    angle = (angle - first) % 360
    return (angle < second)

def normalize_angle(a):
    if a < 0:
        a += 360
    return a % 360

def vector_mean(vectors: list) -> Vector2:
    result = Vector2(0, 0)
    for v in vectors:
        result += v
    return result / len(vectors)

# def distance_from_angle(v_angle, height=con.BALL_DIAMETER):
#     camera_height_relative = con.CAMERA_MOUNT_POSITION[2] - height/2
#     return camera_height_relative / math.tan(math.radians(v_angle))
