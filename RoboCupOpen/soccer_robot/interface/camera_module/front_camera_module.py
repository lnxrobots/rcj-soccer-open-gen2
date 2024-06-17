from soccer_robot.interface.camera_module.camera_module import CameraModule
from soccer_robot.utils import camera
import soccer_robot.constants as con

class FrontCameraModule(CameraModule):
    module_name = "FrontCameraModule"

    @classmethod
    def init(cls, debug=True):
        super().init(con.FRONT_CAMERA, debug)

    @classmethod
    def is_near_edge(cls, bounding_box):
        x, y, w, h = bounding_box
        return x-w/2 <= 0.01 or x+w/2 >= 1-0.01
    
    @classmethod
    def get_ball_position(cls) -> tuple:
        with cls._ball_position.get_lock():
            return cls._ball_position[0:4] if not cls.is_near_edge(cls._ball_position[0:4]) else [-1, -1, 0, 0]
