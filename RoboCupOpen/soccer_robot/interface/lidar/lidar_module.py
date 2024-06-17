from soccer_robot.module.module import Module
from soccer_robot.logger.logger_module import LoggerModule
from soccer_robot.mathf import mathf
from soccer_robot.mathf.vector2 import Vector2
import soccer_robot.constants as con

import multiprocessing
from multiprocessing import shared_memory
import numpy

import math
import time
import struct

class ScanPoint():
    def __init__(self, angle: float, distance: int) -> None:
        self.angle: float = angle
        self.distance: int = distance

        # Calculate cartesian coordinates
        self.x = math.cos(math.radians(angle)) * distance
        self.y = math.sin(math.radians(angle)) * distance


class HoughSpaceLine():
    def __init__(self, normal_angle, distance):
        self.normal_angle = normal_angle
        self.distance = distance


class LidarModule(Module):
    module_name = "LidarModule"

    @classmethod
    def init(cls) -> None:
        super().init()

        #if cls._debug_mode:
            #cls._lidar_buffer = shared_memory.SharedMemory(create = True, size = con.CAMERA_FRAMEBUFFER_SIZE)

        cls._positionX = multiprocessing.Value("f", 0)
        cls._positionY = multiprocessing.Value("f", 0)
        cls._n_field_robots = multiprocessing.Value("i", 0)
        cls._field_robot_positions = multiprocessing.Array("f", [-1] * 6)

        cls._in_lidar_data_queue = multiprocessing.Queue()

        cls._lidar_buffer = shared_memory.SharedMemory(create = True, size = 14400)
        cls._lidar_buffer_size = multiprocessing.Value("i", 0)
        cls._lidar_buffer_lock = multiprocessing.Lock()

    @classmethod
    def on_start(cls) -> None:
        cls._scan_points = list()
        cls._working_scan_points = list()
        cls._max_scan_angle = 0

        cls.angleConst = 1
        cls.thetas = numpy.deg2rad(numpy.arange(-90 / cls.angleConst, 90 / cls.angleConst))

        cls.cached_sines = list()
        cls.cached_cosines = list()

        for angle in cls.thetas:
            cls.cached_sines.append(math.sin(angle * cls.angleConst))
            cls.cached_cosines.append(math.cos(angle * cls.angleConst))

    @classmethod
    def on_update(cls) -> None:

        while cls._in_lidar_data_queue.qsize() > 0:
            lidar_data = cls._in_lidar_data_queue.get()

            start_angle = lidar_data[0]
            end_angle = lidar_data[1]
            distances = lidar_data[2]

            if start_angle == 0 and end_angle == 0:
                continue

            # In case of angle going through zero ...
            if start_angle > end_angle:
                end_angle += 360

            # Since there are 12 measurement in each packet we divide the scan range into equally sized steps
            step = (end_angle - start_angle) / 11

            for i in range(12):
                angle = (start_angle + step * i) % 360
                distance = distances[i]

                if angle < cls._max_scan_angle:
                    # Scan is completed
                    cls._scan_points = cls._working_scan_points.copy()

                    cls._process_scan_points()

                    cls._max_scan_angle = 0
                    cls._working_scan_points.clear()
                else:
                    cls._max_scan_angle = angle

                from soccer_robot.interface.undercarriage_module import UndercarriageModule
                heading = UndercarriageModule.get_heading()

                cls._working_scan_points.append(ScanPoint(angle + heading, distance))



    @classmethod
    def on_stop(cls) -> None:
        cls._lidar_buffer.close()
        cls._lidar_buffer.unlink()

    @classmethod
    def _process_scan_points(cls) -> None:
        scan_points = cls._scan_points
        pointIDs = [0] * len(scan_points)

        # Generate hough space from lidar data
        # TODO: Consider generating hough only from zero and right angles

        hough_space = cls._generate_hough_space(scan_points)

        # Find the "Primary Line" - line which contains the most points

        primary_index = numpy.argmax(hough_space)
        primary_angle = round(math.degrees(cls.thetas[primary_index % hough_space.shape[1]] * cls.angleConst))
        primary_distance = ((primary_index // hough_space.shape[1]) - 249) * 24

        primary_index_shifted = primary_index

        if primary_distance >= 0:
            searchRange = range(primary_index // hough_space.shape[1], hough_space.shape[0], 1)
        else:
            searchRange = range(primary_index // hough_space.shape[1], 0, -1)

        for i in searchRange:
            nPoints = hough_space[i, primary_index % hough_space.shape[1]]

            if nPoints > 10:
                primary_index_shifted = i * hough_space.shape[1] + primary_index % hough_space.shape[1]


        primary_angle_shifted = round(math.degrees(cls.thetas[primary_index_shifted % hough_space.shape[1]] * cls.angleConst))
        primary_distance_shifted = ((primary_index_shifted // hough_space.shape[1]) - 249) * 24

        primary_line = HoughSpaceLine(primary_angle, primary_distance)
        primary_line_shifted = HoughSpaceLine(primary_angle_shifted, primary_distance_shifted)


        # Find Orthogonal line to "Primary line"

        maxX = 0
        maxY = 0

        for i in range(hough_space.shape[1]):
            if 87 < abs(math.degrees(cls.thetas[i] * cls.angleConst) - primary_line.normal_angle) < 93:
                y = i
                x = numpy.argmax(hough_space[:, y])

                if hough_space[x, y] > hough_space[maxX, maxY]:
                    maxX = x
                    maxY = y


        orthogonal_index = maxX * hough_space.shape[1] + maxY
        orthogonal_angle = (maxY * cls.angleConst) - 90
        orthogonal_distance = (maxX - 249) * 24


        orthogonal_index_shifted = orthogonal_index


        if orthogonal_distance >= 0:
            searchRange = range(orthogonal_index // hough_space.shape[1], hough_space.shape[0], 1)
        else:
            searchRange = range(orthogonal_index // hough_space.shape[1], 0, -1)

        newMaxX = maxX

        for i in searchRange:
            nPoints = hough_space[i, orthogonal_index % hough_space.shape[1]]

            if nPoints > 10:
                orthogonal_index_shifted = i * hough_space.shape[1] + orthogonal_index % hough_space.shape[1]


        orthogonal_angle_shifted = round(math.degrees(cls.thetas[orthogonal_index_shifted % hough_space.shape[1]] * cls.angleConst))
        orthogonal_distance_shifted = ((orthogonal_index_shifted // hough_space.shape[1]) - 249) * 24


        orthogonal_line = HoughSpaceLine(orthogonal_angle, orthogonal_distance)
        orthogonal_line_shifted = HoughSpaceLine(orthogonal_angle_shifted, orthogonal_distance_shifted)


        # Calculate position from primary and orthogonal lines
        left_right = 0
        up_down = 0

        xCoord = 0
        yCoord = 0

        if abs(primary_line_shifted.normal_angle - 0) < 20 or abs(primary_line_shifted.normal_angle - 180) < 20:
            # First line is the primary line
            if abs(primary_line_shifted.normal_angle - 0) < 20 and primary_line_shifted.distance < 0:
                left_right = 1
                xCoord = abs(primary_line_shifted.distance)
            else:
                left_right = -1
                xCoord = con.FIELD_SIZE.x - abs(primary_line_shifted.distance)

            if (abs(orthogonal_line_shifted.normal_angle - (-90)) < 20 and orthogonal_line_shifted.distance > 0) or (abs(orthogonal_line_shifted.normal_angle - 90) < 20 and orthogonal_line_shifted.distance < 0):
                up_down = 1
                yCoord = abs(orthogonal_line_shifted.distance)
            else:
                up_down = -1
                yCoord = con.FIELD_SIZE.y - abs(orthogonal_line_shifted.distance)

        else:
            if abs(orthogonal_line_shifted.normal_angle - 0) < 20 and orthogonal_line_shifted.distance < 0:
                left_right = 1
                xCoord = abs(orthogonal_line_shifted.distance)
            else:
                left_right = -1
                xCoord = con.FIELD_SIZE.x - abs(orthogonal_line_shifted.distance)


            if (abs(primary_line_shifted.normal_angle - (-90)) < 20 and primary_line_shifted.distance > 0) or (abs(primary_line_shifted.normal_angle - 90) < 20 and primary_line_shifted.distance < 0):
                up_down = 1
                yCoord = abs(primary_line_shifted.distance)
            else:
                up_down = -1
                yCoord = con.FIELD_SIZE.y - abs(primary_line_shifted.distance)


        if abs(primary_line_shifted.normal_angle - 0) < 45 or abs(primary_line_shifted.normal_angle - 180) < 45:
            primary_parallel_line = HoughSpaceLine(primary_line_shifted.normal_angle, primary_line_shifted.distance + con.FIELD_SIZE.x * left_right)
            orthogonal_parallel_line = HoughSpaceLine(orthogonal_line_shifted.normal_angle, orthogonal_line_shifted.distance + con.FIELD_SIZE.y * up_down * mathf.sign(orthogonal_line_shifted.normal_angle))
        else:
            primary_parallel_line = HoughSpaceLine(primary_line_shifted.normal_angle, primary_line_shifted.distance - con.FIELD_SIZE.y * up_down * -mathf.sign(primary_line_shifted.normal_angle))
            orthogonal_parallel_line = HoughSpaceLine(orthogonal_line_shifted.normal_angle, orthogonal_line_shifted.distance + con.FIELD_SIZE.x * left_right)


        # Calculate points which belong to the playfield

        primary_line_points = cls._get_close_line_points(scan_points, primary_line_shifted, 180)
        orthogonal_line_points = cls._get_close_line_points(scan_points, orthogonal_line_shifted, 180)
        primary_parallel_line_points = cls._get_close_line_points(scan_points, primary_parallel_line, 180)
        orthogonal_parallel_line_points = cls._get_close_line_points(scan_points, orthogonal_parallel_line, 180)


        for i in primary_line_points + orthogonal_line_points + primary_parallel_line_points + orthogonal_parallel_line_points:
            pointIDs[i] = 1


        # Remove points too close/far

        for i in range(len(scan_points)):
            if not (150 < scan_points[i].distance < 3000):
                pointIDs[i] = -1


        # Verify

        n_ignore_points = pointIDs.count(-1)
        field_points_ratio = 0 #FIXME Why so few points?
        if len(pointIDs) > n_ignore_points:
            field_points_ratio = pointIDs.count(1) / (len(pointIDs) - n_ignore_points)

        if field_points_ratio < 0.6:
            verification = False
            xCoord = -1
            yCoord = -1
        else:
            verification = True


        # SUPER DUPER ENEMY DETECTIN A.K.A. ORBOCULUM

        field_robots = list()

        if verification:
            point_groups = list()

            for i in range(len(scan_points)):
                if pointIDs[i] == 0:
                    found = False

                    for j in range(len(point_groups)):
                        enemy_x = point_groups[j][1]
                        enemy_y = point_groups[j][2]

                        if (enemy_x - scan_points[i].x) ** 2 + (enemy_y - scan_points[i].y) ** 2 < 300 ** 2:
                            point_groups[j] = (point_groups[j][0] + 1, (point_groups[j][1] * point_groups[j][0] + scan_points[i].x) / (point_groups[j][0] + 1), (point_groups[j][2] * point_groups[j][0] + scan_points[i].y) / (point_groups[j][0] + 1))
                            found = True
                            break

                    if not found:
                        point_groups.append((1, scan_points[i].x, scan_points[i].y))


            point_groups.sort(key = lambda x: x[0])

            for i in range(min(3, len(point_groups))):
                if point_groups[i][0] > 5:
                    field_robots.append(cls._point_position_to_field_position(point_groups[i][1], point_groups[i][2], primary_line_shifted, orthogonal_line_shifted, left_right, up_down, con.FIELD_SIZE.x, con.FIELD_SIZE.y))



        # Save to buffer

        with cls._lidar_buffer_lock:
            lidar_data = numpy.ndarray((1800), float, buffer = cls._lidar_buffer.buf)
            cls._lidar_buffer_size.value = len(scan_points)

            for i in range(len(scan_points)):
                lidar_data[i * 3] = scan_points[i].angle
                lidar_data[i * 3 + 1] = float(scan_points[i].distance)
                lidar_data[i * 3 + 2] = pointIDs[i]

            lidar_data[len(scan_points)] = xCoord / con.FIELD_SIZE.x
            lidar_data[len(scan_points) + 1] = yCoord / con.FIELD_SIZE.y

            lidar_data[len(scan_points) + 2] = 4

            lidar_data[len(scan_points) + 3] = primary_line_shifted.distance
            lidar_data[len(scan_points) + 4] = primary_line_shifted.normal_angle

            lidar_data[len(scan_points) + 5] = orthogonal_line_shifted.distance
            lidar_data[len(scan_points) + 6] = orthogonal_line_shifted.normal_angle

            lidar_data[len(scan_points) + 7] = primary_parallel_line.distance
            lidar_data[len(scan_points) + 8] = primary_parallel_line.normal_angle

            lidar_data[len(scan_points) + 9] = orthogonal_parallel_line.distance
            lidar_data[len(scan_points) + 10] = orthogonal_parallel_line.normal_angle

            lidar_data[len(scan_points) + 11] = len(field_robots)

            for i in range(len(field_robots)):
                lidar_data[len(scan_points) + 12 + i * 2] = field_robots[i][0] / con.FIELD_SIZE.y
                lidar_data[len(scan_points) + 12 + i * 2 + 1] = field_robots[i][1] / con.FIELD_SIZE.x


        with cls._positionX.get_lock() and cls._positionY.get_lock() and cls._n_field_robots.get_lock() and cls._field_robot_positions.get_lock():
            if verification:
                cls._positionX.value = xCoord
                cls._positionY.value = yCoord

                cls._n_field_robots.value = len(field_robots)

                field_robot_positions = [-1] * 6

                for i in range(len(field_robots)):
                    field_robot_positions[i * 2] = field_robots[i][0]
                    field_robot_positions[i * 2 + 1] = field_robots[i][1]

                cls._field_robot_positions[:] = field_robot_positions[:]
            else:
                cls._positionX.value = -1
                cls._positionY.value = -1

                cls._n_field_robots.value = 0
                cls._field_robot_positions[:] = ([-1] * 6)[:]

    @classmethod
    def _generate_hough_space(cls, scan_points: list):
        hough_space = numpy.zeros((500, len(cls.thetas)))

        for scan_point in scan_points:
            # Filter min/max distances
            if scan_point.distance < 300 or scan_point.distance > 3000:
                continue


            for i in range(len(cls.thetas)):
                distance = scan_point.x * cls.cached_cosines[i] + scan_point.y * cls.cached_sines[i]
                hough_space[round(distance / 24) + 249, i] += 1

        return hough_space

    @classmethod
    def _least_squares_fit(cls, scan_points: list, line_points: list) -> HoughSpaceLine:
        sumX = 0
        sumY = 0

        for i in line_points:
            sumX += scan_points[i].x
            sumY += scan_points[i].y

        meanX = sumX / len(line_points)
        meanY = sumY / len(line_points)

        sum1 = 0

        for i in line_points:
            sum1 += (scan_points[i].x - meanX) * (scan_points[i].y - meanY)

        sum2 = 0

        for i in line_points:
            sum2 += (scan_points[i].x - meanX) ** 2

        m = sum1 / sum2
        b = meanY - m * meanX

        line_angle = int(math.degrees(math.atan(m)) + 90)
        line_distance = b / ((m ** 2 + 1) ** (1 / 2))

        return HoughSpaceLine(line_angle, line_distance)

    @classmethod
    def _distance_point_from_line(cls, point_x: int, point_y: int, line: HoughSpaceLine) -> float:
        line_sin = math.sin(math.radians(line.normal_angle))
        line_cos = math.cos(math.radians(line.normal_angle))

        point_distance = abs(point_x * line_cos + point_y * line_sin - line.distance)

        return point_distance

    @classmethod
    def _get_close_line_points(cls, scan_points: list, line: HoughSpaceLine, distance: int) -> list:
        line_sin = math.sin(math.radians(line.normal_angle))
        line_cos = math.cos(math.radians(line.normal_angle))

        line_points = list()

        for i in range(len(scan_points)):
            point_distance = scan_points[i].x * line_cos + scan_points[i].y * line_sin - line.distance

            if abs(point_distance) < distance:
                line_points.append(i)

        return line_points

    @classmethod
    def _point_position_to_field_position(cls, point_x, point_y, primary_line: HoughSpaceLine, orthogonal_line: HoughSpaceLine, left_right: int, up_down: int, field_width: int, field_height):
        if abs(primary_line.normal_angle - 0) < 45 or abs(primary_line.normal_angle - 180) < 45:
            xCoord = cls._distance_point_from_line(point_x, point_y, primary_line)
            yCoord = cls._distance_point_from_line(point_x, point_y, orthogonal_line)
        else:
            xCoord = cls._distance_point_from_line(point_x, point_y, orthogonal_line)
            yCoord = cls._distance_point_from_line(point_x, point_y, primary_line)

        if left_right == -1:
            xCoord = field_width - xCoord

        if up_down == -1:
            yCoord = field_height - yCoord

        return (xCoord, yCoord)

    @classmethod
    def add_lidar_scan(cls, start_angle: int, end_engle: int, distances: list) -> None:
        # Queue for lidar data

        cls._in_lidar_data_queue.put((start_angle, end_engle, distances))

    @classmethod
    def get_position(cls) -> Vector2:
        with cls._positionX.get_lock() and cls._positionY.get_lock():
            return Vector2(cls._positionX.value, cls._positionY.value)

    @classmethod
    def get_n_field_robots(cls) -> int:
        with cls._n_field_robots.get_lock():
            return cls._n_field_robots.value

    @classmethod
    def get_field_robot_positions(cls) -> list:
        with cls._field_robot_positions.get_lock():
            return cls._field_robot_positions[0:6]

    @classmethod
    def get_frame_buffer(cls) -> tuple:
        with cls._lidar_buffer_size.get_lock():
            return (cls._lidar_buffer, cls._lidar_buffer_size.value, cls._lidar_buffer_lock)
