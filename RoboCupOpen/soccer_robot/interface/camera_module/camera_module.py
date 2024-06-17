from soccer_robot.module.module import Module
from soccer_robot.interface.undercarriage_module import UndercarriageModule
from soccer_robot.logger.logger_module import LoggerModule
from soccer_robot.utils.timer import Timer
from soccer_robot.utils import camera
from soccer_robot.mathf.vector2 import Vector2
from soccer_robot.mathf import mathf

import soccer_robot.constants as con

from picamera2 import Picamera2
import cv2
import numpy

import multiprocessing
from multiprocessing import shared_memory
import os
import time
import math
import tarfile
import io
import datetime

class CameraModule(Module):
    module_name = "CameraModule"

    @classmethod
    def init(cls, camera: camera.Camera, debug=True, ball_detection=True, goal_detection=True) -> None:
        super().init()

        cls.camera = camera
        cls._picamera = None
        cls._camera_config = None
        cls._job = None
        cls._last_frame_time = 0

        cls._ball_position = multiprocessing.Array("f", [-1, -1, 0, 0])
        cls._goal_position = multiprocessing.Array("f", [-1, -1, 0, 0])

        cls._ball_color_bounds = multiprocessing.Array("i", [0] * 6)
        cls._goal_color_bounds = multiprocessing.Array("i", [0] * 6)
        cls._vertical_crop_01 = multiprocessing.Value("d", cls.camera.vertical_crop)
        cls._enable_ball_detection = multiprocessing.Value("b", ball_detection)
        cls._enable_goal_detection = multiprocessing.Value("b", goal_detection)

        cls._is_new_frame = multiprocessing.Value("b", False)
        cls._debug_mode = debug

        cls.tuning = None
        if cls.camera.cam.tuning_file:
            cls.tuning = Picamera2.load_tuning_file(os.path.join(os.getcwd(), cls.camera.cam.tuning_file))

        if cls._debug_mode:
            cls._frame_buffer = shared_memory.SharedMemory(create = True, size = cls.camera.framebuffer_size)
            cls._frame_buffer_size = multiprocessing.Value("i", 0)
            cls._frame_buffer_lock = multiprocessing.Lock()

    @classmethod
    def safe_camera_start(cls):
        camera_name = cls.module_name.replace('Module', '')
        started = False
        i = 0
        while not started and i < 5:
            try:
                cls._picamera.start()
                started = True
            except RuntimeError as e:
                if 'Broken pipe' in e.args[0]:
                    cls._picamera.close()
                    cls._picamera = Picamera2(cls.camera.port, tuning=cls.tuning)
                    LoggerModule.log_warning(f'{camera_name}: Broken pipe, trying again ...')
                    i += 1
                    time.sleep(1)
                else:
                    raise e
        if not started:
            raise RuntimeError(f"{camera_name}: Too many attempts to start camera")
        LoggerModule.log_info(f"{camera_name} started successfully")
        return True

    @classmethod
    def on_start(cls) -> None:
        # Set libcamera log level to errors only
        os.environ["LIBCAMERA_LOG_LEVELS"] = "3"

        try:
            cls._picamera = Picamera2(cls.camera.port, tuning=cls.tuning)
        except Exception as e:
            LoggerModule.log_error("Failed to init camera, PiCamera is not connected")
            return

        cls._camera_config = cls._picamera.create_video_configuration(
            main={"format": "RGB888", "size": cls.camera.cam.size},
            raw={"size": cls.camera.cam.raw_size},
            controls={"FrameDurationLimits": (int(10 ** 6 / cls.camera.cam.fps), int(10 ** 6 / cls.camera.cam.fps))}
        )
        cls._picamera.configure(cls._camera_config)
        cls._picamera.set_controls({"AwbEnable": False})
        cls.safe_camera_start()

        cv2.setUseOptimized(True)

        # TODO: Load boundaries from file

        cls.is_new_raw_frame = False

        # last_store_time = 0
        cls._last_frame_time = time.time()

        # Obtain frame from camera
        cls._job = cls._picamera.capture_array(signal_function = cls._on_frame_captured)

        cls._frame_tar = tarfile.open("frames/frames_{}_{}.tar".format(datetime.datetime.now().strftime("%Y-%m-%d-%H_%M_%S"), cls.module_name.replace("CameraModule", "")), "w|")
        cls._frame_id = 0

    @classmethod
    def on_update(cls) -> None:
        # current_time = time.time()

        if time.time() - cls._last_frame_time > 2:
            LoggerModule.log_error("Camera capturing failed, restarting ...")


            LoggerModule.log_info("Stopping")

            cls._picamera.close()

            LoggerModule.log_info("Stopped")

            time.sleep(1)

            LoggerModule.log_info("Reiniting")

            try:
                cls._picamera = Picamera2(cls.camera.port, tuning=cls.tuning)
            except Exception as e:
                LoggerModule.log_error("Failed to init camera, PiCamera is not connected")
                return

            cls._picamera.configure(cls._camera_config)
            cls._picamera.set_controls({"AwbEnable": False})
            cls.safe_camera_start()

            LoggerModule.log_info("reinited")

            cls._job = cls._picamera.capture_array(signal_function = cls._on_frame_captured)
            cls._last_frame_time = time.time()

        if cls.is_new_raw_frame:
            cls.is_new_raw_frame = False
            cls._last_frame_time = time.time()

            raw_frame = cls._picamera.wait(cls._job)

            cls._job = cls._picamera.capture_array(signal_function = cls._on_frame_captured)

            # Convert BGR to HSV
            cropped_frame = raw_frame[int(cls.camera.cam.height * cls.get_vertical_crop()) : cls.camera.cam.height, 0 : cls.camera.cam.width]
            hsv = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)

            # Draw crop line
            #cv2.line(img = raw_frame, pt1 = (0, int(CAMERA_HEIGHT * cls._vertical_crop_01.value)), pt2 = (CAMERA_WIDTH, int(CAMERA_HEIGHT * cls._vertical_crop_01.value)), color = (0, 0, 255), thickness = 2)


            #LoggerModule.log_robot_data(str(bounding_box_timer.get()) + " " + str(ball_bounding_box[2] * ball_bounding_box[3]))
            #LoggerModule.log_debug(bounding_box_timer.get())

            #cv2.rectangle(raw_frame, (goal_x, goal_y), (goal_x + goal_w, goal_y + goal_h), (0, 255, 0), 2)

            #print(cls._ball_color_bounds[:], cls.module_name)

            ball_bounding_box = (-1, -1, 0, 0)
            if (cls.get_enable_ball()):
                ball_bounding_box = cls._calculate_bounding_box(hsv, cls._ball_color_bounds)
                # LoggerModule.log_info(f'Ball bounding box {cls.is_near_edge(ball_bounding_box)} {ball_bounding_box}')

            if ball_bounding_box[2] < cls.camera.min_ball_size[0] or ball_bounding_box[3] < cls.camera.min_ball_size[1]:
                ball_bounding_box = (-1, -1, 0, 0)

            with cls._ball_position.get_lock():
                cls._ball_position[0:4] = list(ball_bounding_box)

            goal_bounding_box = (-1, -1, 0, 0)
            if (cls.get_enable_goal()):
                goal_bounding_box = cls._calculate_bounding_box(hsv, cls._goal_color_bounds, True)

            if goal_bounding_box[2] < cls.camera.min_goal_size[0] or goal_bounding_box[3] < cls.camera.min_goal_size[1]:
                goal_bounding_box = (-1, -1, 0, 0)

            with cls._goal_position.get_lock():
                cls._goal_position[0:4] = list(goal_bounding_box)

            # In debug_mode also encode and store the image to the shared memory to be later used by the visualizer
            """
            if cls._debug_mode and (current_time-last_store_time) >= VISUALIZER_MESSAGE_INTERVAL:
                last_store_time = current_time
                cls._store_image_in_frame_buffer(raw_frame)
            """
            if cls._debug_mode:
                cls._store_image_in_frame_buffer(raw_frame)

            cls._frame_id += 1

    @classmethod
    def on_stop(cls) -> None:
        cls._picamera.stop()
        cls._picamera.close()
        # Clear the shared memory
        if cls._debug_mode:
            cls._frame_buffer.close()
            cls._frame_buffer.unlink()

        cls._frame_tar.close()

    @classmethod
    def set_ball_color_bounderies(cls, minH: int, minS: int, minV: int, maxH: int, maxS: int, maxV: int) -> None:
        with cls._ball_color_bounds.get_lock():
            cls._ball_color_bounds[:] = [minH, minS, minV, maxH, maxS, maxV][:]

    @classmethod
    def set_goal_color_bounderies(cls, minH: int, minS: int, minV: int, maxH: int, maxS: int, maxV: int) -> None:
        with cls._goal_color_bounds.get_lock():
            cls._goal_color_bounds[:] = [minH, minS, minV, maxH, maxS, maxV][:]

    @classmethod
    def set_enable_ball(cls, enable_ball: bool) -> None:
        with cls._enable_ball_detection.get_lock():
            cls._enable_ball_detection.value = enable_ball

    @classmethod
    def set_enable_goal(cls, enable_goal: bool) -> None:
        with cls._enable_goal_detection.get_lock():
            cls._enable_goal_detection.value = enable_goal

    @classmethod
    def get_enable_ball(cls):
        with cls._enable_ball_detection.get_lock():
            return cls._enable_ball_detection.value

    @classmethod
    def get_enable_goal(cls):
        with cls._enable_goal_detection.get_lock():
            return cls._enable_goal_detection.value

    # Returns tuple containing (frame_buffer memory adress, size of frame_buffer, lock)
    @classmethod
    def get_frame_buffer(cls) -> tuple:
        if cls._debug_mode:
            with cls._frame_buffer_size.get_lock():
                with cls._is_new_frame.get_lock():
                    cls._is_new_frame.value = False
                return (cls._frame_buffer, cls._frame_buffer_size.value, cls._frame_buffer_lock)
        else:
            LoggerModule.log_error("Cannot get frame_buffer, set DEBUG_MODE to True to enable storing frames in frame buffers")

    @classmethod
    def get_angle_dist(cls, position=None, real_size=(con.BALL_DIAMETER, con.BALL_DIAMETER), i=0):
        if position is None:
            position = cls.get_ball_position()
        x = position[0]
        cx = (x-0.5)*cls.camera.cam.width

        c_xa = math.atan2(cx, cls.camera.cam.f_len_px)

        c_dist = cls.distance_from_size(position[2:], real_size, i)

        r_vec = Vector2(a=c_xa, m=c_dist)+Vector2(*cls.camera.mount.position[:2])

        return mathf.normalize_angle(math.degrees(r_vec.get_angle())), r_vec.get_magnitude()

    @classmethod
    def distance_from_size(cls, px_size, real_size=(con.BALL_DIAMETER, con.BALL_DIAMETER), i=0):
        if px_size[i] == 0:
            return -1
        image_size = cls.camera.cam.sensor_size[i] * px_size[i]
        distance_from_camera = real_size[i] * cls.camera.cam.f_len_mm / image_size * cls.camera.cam.distance_multiplier
        camera_height_relative = cls.camera.mount.position[2] - real_size[1]/2

        if distance_from_camera > camera_height_relative:
            return math.sqrt(distance_from_camera ** 2 - camera_height_relative ** 2)
        return 100

    @classmethod
    def get_ball_position(cls) -> tuple:
        with cls._ball_position.get_lock():
            return cls._ball_position[0:4]

    @classmethod
    def get_goal_position(cls) -> tuple:
        with cls._goal_position.get_lock():
            return cls._goal_position[0:4]

    @classmethod
    def is_new_frame(cls) -> bool:
        with cls._is_new_frame.get_lock():
            return cls._is_new_frame.value

    @classmethod
    def get_vertical_crop(cls):
        with cls._vertical_crop_01.get_lock():
            return cls._vertical_crop_01.value

    @classmethod
    def _on_frame_captured(cls, raw_frame):
        cls.is_new_raw_frame = True

    @classmethod
    def _calculate_contours(cls, hsv_frame, color_bounderies):
        lower_boundery = numpy.array(color_bounderies[0:3])
        upper_boundery = numpy.array(color_bounderies[3:6])

        if (lower_boundery[0] > upper_boundery[0]):
            first_mask = cv2.inRange(hsv_frame, numpy.array((lower_boundery[0], lower_boundery[1], lower_boundery[2])), numpy.array((180, upper_boundery[1], upper_boundery[2])))
            second_mask = cv2.inRange(hsv_frame, numpy.array((0, lower_boundery[1], lower_boundery[2])), numpy.array((upper_boundery[0], upper_boundery[1], upper_boundery[2])))

            mask = cv2.bitwise_or(first_mask, second_mask)
        else:
            mask = cv2.inRange(hsv_frame, lower_boundery, upper_boundery)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    @classmethod
    # X, Y is in the middle of bounding box, and all values are normalized between 0, 1
    # To get original the value multiple it with camera resolution
    def _calculate_bounding_box(cls, hsv_frame, color_boundaries, center_height=False):
        #cls._store_image_in_frame_buffer(second_mask)
        x, y, w, h = -1, -1, 0, 0
        contours = cls._calculate_contours(hsv_frame, color_boundaries)

        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(max_contour)

            if center_height:
                s = w*con.GOAL_HEIGHT_SAMPLE
                t = time.perf_counter()
                condition = (max_contour[:, :, 0] >= x+(w-s)/2)[:, 0] & (max_contour[:, :, 0] <= x+(w+s)/2)[:, 0]
                ys = max_contour[condition][:, 0, 1]
                upper, lower = ys[ys < y+h/2], ys[ys > y+h/2]
                if len(upper) and len(lower):
                    h = numpy.average(lower) - numpy.average(upper)

        if x == -1 or y == -1:
            return (-1, -1, 0, 0)

        return (int(x + w / 2) / cls.camera.cam.width,
                (int(y + h / 2) + cls.camera.cam.height * cls.get_vertical_crop()) / cls.camera.cam.height,
                w / cls.camera.cam.width,
                h / cls.camera.cam.height)

    @classmethod
    def _store_image_in_frame_buffer(cls, raw_frame_array) -> None:

        # Downscale the frame
        if cls.module_name == "MirrorCameraModule":
            raw_frame_array = cv2.resize(raw_frame_array, (507, 380))
        else:
            raw_frame_array = cv2.resize(raw_frame_array, (con.VISUALIZE_IMAGE_WIDTH, con.VISUALIZE_IMAGE_HEIGHT))

        # Encode the frame
        result, encoded_frame = cv2.imencode(".jpg", raw_frame_array, [int(cv2.IMWRITE_JPEG_QUALITY), cls.camera.encode_quality])

        if (not result):
            LoggerModule.log_warning("Failed to encode image")
            return

        if (encoded_frame.nbytes > cls.camera.framebuffer_size):
            LoggerModule.log_warning("Failed to store frame in the frame_buffer: frame buffer too small ({} > {}), try increasing frame_buffer size".format(encoded_frame.nbytes, cls.camera.framebuffer_size))
            return


        if cls._frame_id % 10 == 0 and not UndercarriageModule.get_button_one_status():
            jpg = io.BytesIO(encoded_frame.flatten().tobytes())

            tarInfo = tarfile.TarInfo("frame{}.jpg".format(cls._frame_id // 10))
            tarInfo.size = len(encoded_frame.flatten())
            tarInfo.mtime = time.time()

            cls._frame_tar.addfile(tarInfo, jpg)

        # Create a reference to frame_buffer and copy encoded_frame to it
        frame_buffer = numpy.ndarray(shape = encoded_frame.shape, dtype = encoded_frame.dtype, buffer = cls._frame_buffer.buf)

        with cls._frame_buffer_size.get_lock():
            cls._frame_buffer_size.value = encoded_frame.shape[0]

        with cls._frame_buffer_lock:
            frame_buffer[:] = encoded_frame[:]

        with cls._is_new_frame.get_lock():
            cls._is_new_frame.value = True
