from soccer_robot.module.module import Module
from soccer_robot.logger.logger_module import LoggerModule
from soccer_robot.utils.timer import Timer
import soccer_robot.constants as con

import multiprocessing as mp
from ctypes import c_bool
import time
import json
from json.decoder import JSONDecodeError
from bluedot.btcomm import BluetoothServer, BluetoothClient

class RobotServer(BluetoothServer):
    """Robot Bluetooth Server"""

    def __init__(self, data_received, **kwargs):
        super().__init__(data_received, power_up_device=con.BTH_POWERUP_ADAPTER, **kwargs)

    def check_connection(self):
        return self.client_connected

    def when_client_connects(self):
        LoggerModule.log_info("Client robot connected")

    def when_client_disconnects(self):
        LoggerModule.log_warning("Client robot disconnected")

class RobotClient(BluetoothClient):
    """Robot Bluetooth Client"""

    def __init__(self, data_received, **kwargs):
        super().__init__(
            con.ROBOT_NAMES[con.BTH_SERVER_INDEX],
            data_received,
            power_up_device=con.BTH_POWERUP_ADAPTER,
            auto_connect=False,
            **kwargs
        )
        self._reconnect_timer = Timer()

    def check_connection(self):
        if not self.connected and self._reconnect_timer.get() >= con.BTH_RECONNECT_INTERVAL:
            self._reconnect_timer.reset()
            LoggerModule.log_debug("Disconnected from server robot. Trying to reconnect...")
            try:
                self.connect()
            except Exception as e:
                LoggerModule.log_debug(f"Couldn't connect to server robot: {e}")
            else:
                LoggerModule.log_debug("Connected to server robot")
        return self.connected

    def stop(self):
        self.disconnect()


class BluetoothModule(Module):
    """Module for communication with the other robot"""

    module_name = "BluetoothModule"

    @classmethod
    def init(cls) -> None:
        super().init()
        cls._send_timer = Timer()
        cls._connected = mp.Value(c_bool, False)
        cls._has_ball = mp.Value(c_bool, False)
        cls._see_ball = mp.Value(c_bool, False)
        cls._goalie = mp.Value(c_bool, False)
        cls._locked = mp.Value(c_bool, True)
        cls._ball_angle_dist = mp.Array('d', [0, 0])
        cls._heatmap = mp.Array('d', [0]*len(con.TRACKER_HEATMAP_POINTS))

        cls._has_ball_own = mp.Value(c_bool, False)
        cls._see_ball_own = mp.Value(c_bool, False)
        cls._goalie_own = mp.Value(c_bool, False)
        cls._locked_own = mp.Value(c_bool, True)
        cls._ball_angle_dist_own = mp.Array('d', [0, 0])
        cls._heatmap_own = mp.Array('d', [0]*len(con.TRACKER_HEATMAP_POINTS))

    @classmethod
    def is_connected(cls):
        with cls._connected.get_lock():
            return cls._connected.value

    @classmethod
    def get_remote_heatmap(cls):
        with cls._heatmap.get_lock():
            return cls._heatmap[:]

    @classmethod
    def get_ball_angle_dist(cls):
        with cls._goalie.get_lock():
            return cls._ball_angle_dist[:]

    @classmethod
    def see_ball(cls):
        with cls._see_ball.get_lock():
            return cls._see_ball.value

    @classmethod
    def has_ball(cls):
        with cls._has_ball.get_lock():
            return cls._has_ball.value

    @classmethod
    def is_goalie(cls):
        with cls._goalie.get_lock():
            return cls._goalie.value

    @classmethod
    def is_locked(cls):
        with cls._locked.get_lock():
            return cls._locked.value

    @classmethod
    def set_own_data(cls, has_ball, heatmap, see_ball, ball_angle_dist, goalie, locked):
        # LoggerModule.log_debug('set own data' + str(heatmap))
        with cls._heatmap_own.get_lock():
            cls._heatmap_own[:] = heatmap[:]
        with cls._has_ball_own.get_lock():
            cls._has_ball_own.value = has_ball
        with cls._see_ball_own.get_lock():
            cls._see_ball_own.value = see_ball
        with cls._ball_angle_dist_own.get_lock():
            cls._ball_angle_dist_own[:] = ball_angle_dist[:]
        with cls._goalie_own.get_lock():
            cls._goalie_own.value = goalie
        with cls._locked_own.get_lock():
            cls._locked_own.value = locked

    @classmethod
    def data_received(cls, data):
        # parse data and store in shared variables
        latest_data = data.strip().split("\n")[-1]
        try:
            parsed_data = json.loads(latest_data)
            LoggerModule.log_debug(parsed_data)

            with cls._has_ball.get_lock():
                cls._has_ball.value = parsed_data["has_ball"]
            with cls._see_ball.get_lock():
                cls._see_ball.value = parsed_data["see_ball"]
            with cls._goalie.get_lock():
                cls._goalie.value = parsed_data["goalie"]
            with cls._locked.get_lock():
                cls._locked.value = parsed_data["locked"]
            with cls._ball_angle_dist.get_lock():
                cls._ball_angle_dist[:] = parsed_data["ball_angle_dist"][:]
            with cls._heatmap.get_lock():
                cls._heatmap[:] = parsed_data["heatmap"][:]

        except JSONDecodeError as e:
            LoggerModule.log_warning(f"Invalid json received: {e}")
            return
        except KeyError as e:
            LoggerModule.log_warning(f"Data not found in received json: {e}")
            return

    @classmethod
    def on_run(cls, _stop_flag):
        if con.ROBOT_INDEX == con.BTH_SERVER_INDEX:
            LoggerModule.log_info("Set as bluetooth server")
            device = RobotServer(cls.data_received)
        else:
            LoggerModule.log_info("Set as bluetooth client")
            device = RobotClient(cls.data_received)

        while not _stop_flag.value:
            cls._send_timer.reset()
            connected = device.check_connection()
            with cls._connected.get_lock():
                cls._connected.value = connected
            data = {}
            with cls._has_ball_own.get_lock():
                data["has_ball"] = cls._has_ball_own.value
            with cls._see_ball_own.get_lock():
                data["see_ball"] = cls._see_ball_own.value
            with cls._goalie_own.get_lock():
                data["goalie"] = cls._goalie_own.value
            with cls._locked_own.get_lock():
                data["locked"] = cls._locked_own.value
            with cls._ball_angle_dist_own.get_lock():
                data["ball_angle_dist"] = cls._ball_angle_dist_own[:]
            with cls._heatmap_own.get_lock():
                data["heatmap"] = cls._heatmap_own[:]
            device.send(json.dumps(data)+"\n")

            to_sleep = con.BTH_MESSAGE_INTERVAL - cls._send_timer.get()
            if to_sleep > 0:
                time.sleep(to_sleep/1000)
        device.stop()
