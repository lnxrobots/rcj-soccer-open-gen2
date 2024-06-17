import soccer_robot.constants as con
from soccer_robot.module.module import Module
from soccer_robot.logger.logger_module import LoggerModule
# from .display import Display

# import gpiozero
import multiprocessing as mp
import subprocess
import time
from ctypes import c_bool

BUTTON_MIN_PRESS_INTERVAL = 0.5

class UIModule(Module):
    """Module for showing data on display and handling button events"""

    module_name = "UIModule"

    @classmethod
    def init(cls, debug=True) -> None:
        super().init()
        cls._debug_mode = debug
        cls._status = mp.Array('c', bytearray([0 for i in range(con.UI_MAX_STATUS)]))
        cls._line_threshold = mp.Value('i', 0)
        cls._goalie = mp.Value(c_bool, False)
        cls._last_display_update = 0
        cls._last_wifi_fetch = 0
        cls._last_ip_fetch = 0
        cls._wifi_on = False
        cls._ip_addr = mp.Array("i", [0] * 4)
        cls._display_info = mp.Array("i", [0] * 4)
        cls._leds = mp.Array(c_bool, [0] * 3)

    @classmethod
    def set_line_threshold(cls, thresh):
        with cls._line_threshold.get_lock():
            cls._line_threshold.value = thresh

    @classmethod
    def get_line_threshold(cls):
        with cls._line_threshold.get_lock():
            return cls._line_threshold.value

    @classmethod
    def set_goalie(cls, goalie):
        with cls._goalie.get_lock():
            cls._goalie.value = goalie

    @classmethod
    def get_goalie(cls):
        with cls._goalie.get_lock():
            return cls._goalie.value

    @classmethod
    def set_leds(cls, leds):
        with cls._leds.get_lock():
            cls._leds[0:3] = leds

    @classmethod
    def get_leds(cls):
        with cls._leds.get_lock():
            return cls._leds[0:3]

    @classmethod
    def get_ip_addr(cls):
        with cls._ip_addr.get_lock():
            return cls._ip_addr[0:4]

    @classmethod
    def set_status(cls, status):
        if len(status) > con.UI_MAX_STATUS:
            status = status[:con.UI_MAX_STATUS]
        with cls._status.get_lock():
            cls._status.value = status.encode('utf-8')

    @classmethod
    def get_display_info(cls):
        with cls._display_info.get_lock():
            return cls._display_info[0:4]

    @classmethod
    def set_display_info(cls, l1, v1, l2, v2):
        with cls._display_info.get_lock():
            cls._display_info[0:4] = [ord(l1), v1, ord(l2), v2]

    @classmethod
    def fetch_ip_addr(cls):
        ip_addr = subprocess.check_output(['hostname', '-I']).decode('ascii').strip().split()
        if not ip_addr:
            ip_addr = ['300.300.300.300']
        with cls._ip_addr.get_lock():
            cls._ip_addr[0:4] = list(map(int, ip_addr[0].split('.')))

    @classmethod
    def fetch_wifi_state(cls):
        output = subprocess.check_output(['ifconfig', 'wlan0']).decode('utf-8').strip()
        cls._wifi_on = 'UP' in output.split('\n')[0]

    @classmethod
    def on_update(cls):
        cls.fetch_ip_addr()
        cls.fetch_wifi_state()
