from soccer_robot.interface.camera_module.camera_module import CameraModule
from soccer_robot.logger.logger_module import LoggerModule
from soccer_robot.utils import camera
from soccer_robot.mathf.vector2 import Vector2
from soccer_robot.mathf import mathf
import soccer_robot.constants as con
import cv2
import math

class MirrorCameraModule(CameraModule):
    module_name = "MirrorCameraModule"

    @classmethod
    def init(cls, debug=True):
        super().init(con.MIRROR_CAMERA, debug, goal_detection=False)

    @classmethod
    def _calculate_bounding_box(cls, hsv_frame, color_boundaries, center_height=False):
        #cls._store_image_in_frame_buffer(second_mask)
        cx, cy = cls.camera.mirror_center
        m_center = Vector2(cx*cls.camera.cam.width, cy*cls.camera.cam.height)
        x, y, w, h = -1, -1, 0, 0
        contours = cls._calculate_contours(hsv_frame, color_boundaries)

        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(max_contour)
            (x, y), (w, h), r = rect
            p1, p2, p3, p4 = cv2.boxPoints(rect)
            centers = [(Vector2(*p)+Vector2(*pn))/2 for p, pn in ((p1, p2), (p2, p3), (p3, p4), (p4, p1))]
            x, y = min(centers, key=lambda v: (v-m_center).get_magnitude())

        if x == -1 or y == -1:
            return (-1, -1, 0, 0)

        return (x / cls.camera.cam.width,
                y / cls.camera.cam.height,
                w / cls.camera.cam.width,
                h / cls.camera.cam.height)

    @classmethod
    def get_angle_dist(cls, position=None):
        if position is None:
            position = cls.get_ball_position()
        cx, cy = con.MIRROR_CAMERA.mirror_center
        x, y = position[:2]
        vec = Vector2((x-cx)*cls.camera.cam.width, (y-cy)*cls.camera.cam.height)
        dist = vec.get_magnitude()/(cls.camera.mirror_radius*cls.camera.cam.width)*cls.camera.mirror.field_radius

        return mathf.normalize_angle(math.degrees(vec.get_angle())), cls.camera.mirror.correct_distance(dist)
