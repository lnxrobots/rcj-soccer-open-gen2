import math

ROBOT_DIAMETER = 180

class CameraConfig:
    def __init__(self, size, raw_size, f_len_mm, fps, hfov, diag_mm, tuning_file="", distance_multiplier=1):
        self.size = size
        self.width = size[0]
        self.height = size[1]
        self.raw_size = raw_size
        self.f_len_mm = f_len_mm
        self.fps = fps
        self.hfov = hfov
        self.diag_mm = diag_mm
        self.tuning_file = tuning_file
        self.distance_multiplier = distance_multiplier
        self.f_len_px = self.width/2/math.tan(math.radians(self.hfov/2))
        self.diag_px = math.sqrt(self.width**2 + self.height**2)
        self.sensor_size = (self.diag_mm * self.width / self.diag_px, self.diag_mm * self.height / self.diag_px)

class MountConfig:
    def __init__(self, position, pitch, roll):
        self.position = position
        self.pitch = pitch
        self.roll = roll

class MirrorConfig:
    def __init__(self, field_radius, correction_coefficients):
        self.field_radius = field_radius
        self.correction_coefficients = correction_coefficients

    def correct_distance(self, distance):
        result = 0
        for i, c in enumerate(self.correction_coefficients[::-1]):
            result += c*distance**i
        return distance#result

class Camera:
    def __init__(self, port, camera_config: CameraConfig, mount_config: MountConfig, mirror: MirrorConfig=None, vertical_crop=0, slot_bounding_box=(0, 0, 0, 0), mirror_center=(0.5, 0.5), min_ball_size=(0.01, 0.01), min_goal_size=(0.03, 0.02), mirror_radius=None, framebuffer_size=1024*256, encode_quality=90):
        self.port = port
        self.cam = camera_config
        self.mount = mount_config
        self.mirror = mirror
        self.vertical_crop = vertical_crop
        self.slot_bounding_box = slot_bounding_box
        self.mirror_center = mirror_center
        self.mirror_radius = mirror_radius
        self.min_ball_size = min_ball_size
        self.min_goal_size = min_goal_size
        self.framebuffer_size = framebuffer_size
        self.encode_quality = encode_quality
        beta = math.radians(self.cam.hfov/2)
        self.center_fov = math.degrees(2*math.atan(math.sin(beta)/(math.cos(beta)+self.mount.position[2]/(ROBOT_DIAMETER/2))))
