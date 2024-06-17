from soccer_robot.module.module import Module
from soccer_robot.logger.logger_module import LoggerModule
from soccer_robot.interface.camera_module.front_camera_module import FrontCameraModule
from soccer_robot.interface.camera_module.mirror_camera_module import MirrorCameraModule
from soccer_robot.interface.undercarriage_module import UndercarriageModule
from soccer_robot.interface.lidar.lidar_module import LidarModule
import soccer_robot.constants as con

import numpy
import gpiozero

import enum
import asyncio
import websockets
import time
import struct
import subprocess
import multiprocessing

class PacketID(enum.Enum):
    INIT = 0
    ERROR = 1
    DATA = 2
    BALL_CALIBRATION = 3
    BLUE_GOAL_CALIBRATION = 4
    YELLOW_GOAL_CALIBRATION = 5
    LINE_SENSOR_CALIBRATION = 6
    MOTORS_CALIBRATION = 7

# Class used for creating byte packets from robot data
class PacketBuilder():
    def __init__(self) -> None:
        self._body: bytearray = bytearray()
        self._packetID: int = -1
        self._packet_size: int = -1

    def new_packet(self, packetID: PacketID):
        self._body.clear()

        self._packetID = packetID.value
        self._packet_size = 5

    def add_int(self, value: int, size: int = 4) -> None:
        self._body += value.to_bytes(size, byteorder = "little", signed = True)
        self._packet_size += size

    def add_float(self, value: float, size: int = 4) -> None:
        self._body += struct.pack("f", value)
        self._packet_size += size

    def add_bytes(self, bytes: bytes):
        self._body += bytes
        self._packet_size += len(bytes)

    def get_bytes(self) -> bytes:
        header: bytearray = bytearray()

        header += self._packetID.to_bytes(1, "little")
        header += self._packet_size.to_bytes(4, "little")

        return header + self._body


class VisualizationModule(Module):
    module_name = "VisualizationModule"

    @classmethod
    def init(cls, ball_calibration: tuple, blue_goal_calibration: tuple, yellow_goal_calibration: tuple, line_calibration: int, motors_calibration: tuple) -> None:
        super().init()

        cls._ball_calibration = multiprocessing.Array("i", ball_calibration)
        cls._blue_goal_calibration = multiprocessing.Array("i", blue_goal_calibration)
        cls._yellow_goal_calibration = multiprocessing.Array("i", yellow_goal_calibration)

        cls._mirror_ball_calibration = multiprocessing.Array("i", ball_calibration)
        cls._mirror_blue_goal_calibration = multiprocessing.Array("i", blue_goal_calibration)
        cls._mirror_yellow_goal_calibration = multiprocessing.Array("i", yellow_goal_calibration)

        cls._line_calibration = multiprocessing.Value("i", line_calibration)
        cls._motors_calibration = multiprocessing.Array("i", motors_calibration)

        cls._position_heatmap = multiprocessing.Array("f", [0] * 9)
        # Every new calibration received from client is put to this queue to be handled by other modules
        cls._calibration_queue = multiprocessing.Queue(10)

    @classmethod
    def on_start(cls) -> None:
        stop_future = asyncio.Future()
        clients = set()

        packet_builder: PacketBuilder = PacketBuilder()

        async def update():
            while True:
                start_time = time.time()

                if False:
                    for ws in clients.copy():
                        await ws.close()

                    stop_future.set_result(0)

                """
                PACKET STRUCTURE:
                    1 - ID
                    2 - 5 - lenght
                    !!! TODO: FINISH PACKET STRUCTURE !!!

                """

                # PACKET CONSTRUCTION
                packet_builder.new_packet(PacketID.DATA)

                # System Information
                cpu_temp: int = int(gpiozero.CPUTemperature().temperature)
                packet_builder.add_int(cpu_temp, 4)
                # TODO: CPU usage
                packet_builder.add_int(0, 4)
                # TODO: Number of process
                packet_builder.add_int(0, 4)
                # TODO: Encodes per second
                packet_builder.add_int(0, 4)

                # Line Sensors and motor values
                # TODO: Color sensors 1 to 16
                if UndercarriageModule.is_inited():
                    for i in UndercarriageModule.get_color_sensor_values():
                        packet_builder.add_int(i, 4)
                    for i in UndercarriageModule.get_motor_values():
                        packet_builder.add_int(i, 4)
                else:
                    for i in range(16 + 4):
                        packet_builder.add_int(-1, 4)

                # Compass
                # TODO: Compass value
                packet_builder.add_int(0, 4)

                # Heatmap
                with cls._position_heatmap.get_lock():
                    for i in range (9):
                        packet_builder.add_float(cls._position_heatmap[i], 4)

                # Front Camera
                ball_bounding_box = FrontCameraModule.get_ball_position()

                packet_builder.add_float(ball_bounding_box[0], 4)
                packet_builder.add_float(ball_bounding_box[1], 4)
                packet_builder.add_float(ball_bounding_box[2], 4)
                packet_builder.add_float(ball_bounding_box[3], 4)

                goal_bounding_box = FrontCameraModule.get_goal_position()

                packet_builder.add_float(goal_bounding_box[0], 4)
                packet_builder.add_float(goal_bounding_box[1], 4)
                packet_builder.add_float(goal_bounding_box[2], 4)
                packet_builder.add_float(goal_bounding_box[3], 4)

                if FrontCameraModule.is_new_frame():
                    frame_buffer, frame_buffer_size, frame_buffer_lock = FrontCameraModule.get_frame_buffer()

                    with frame_buffer_lock:
                        frame = numpy.ndarray(shape = (frame_buffer_size, ), dtype = "uint8", buffer = frame_buffer.buf)
                        packet_builder.add_int(frame_buffer_size, 4)
                        packet_builder.add_bytes(frame.tobytes())

                else:
                    packet_builder.add_int(0, 4)


                # Mirror Camera
                ball_bounding_box = MirrorCameraModule.get_ball_position()

                packet_builder.add_float(ball_bounding_box[0], 4)
                packet_builder.add_float(ball_bounding_box[1], 4)
                packet_builder.add_float(ball_bounding_box[2], 4)
                packet_builder.add_float(ball_bounding_box[3], 4)


                goal_bounding_box = MirrorCameraModule.get_goal_position()

                packet_builder.add_float(goal_bounding_box[0], 4)
                packet_builder.add_float(goal_bounding_box[1], 4)
                packet_builder.add_float(goal_bounding_box[2], 4)
                packet_builder.add_float(goal_bounding_box[3], 4)


                if MirrorCameraModule.is_new_frame():
                    frame_buffer, frame_buffer_size, frame_buffer_lock = MirrorCameraModule.get_frame_buffer()

                    with frame_buffer_lock:
                        frame = numpy.ndarray(shape = (frame_buffer_size, ), dtype = "uint8", buffer = frame_buffer.buf)
                        packet_builder.add_int(frame_buffer_size, 4)
                        packet_builder.add_bytes(frame.tobytes())
                else:
                    packet_builder.add_int(0, 4)




                lidar_buffer, lidar_buffer_size, lidar_buffer_lock = LidarModule.get_frame_buffer()

                with lidar_buffer_lock:
                    lidar_buffer_data = numpy.ndarray((1800), float, buffer = lidar_buffer.buf)

                    packet_builder.add_int(lidar_buffer_size)

                    for i in range(lidar_buffer_size):
                        packet_builder.add_float(float(lidar_buffer_data[i * 3]))
                        packet_builder.add_int(int(lidar_buffer_data[i * 3 + 1]))
                        packet_builder.add_int(int(lidar_buffer_data[i * 3 + 2]))


                    packet_builder.add_float(float(lidar_buffer_data[lidar_buffer_size]), 4)
                    packet_builder.add_float(float(lidar_buffer_data[lidar_buffer_size + 1]), 4)

                    nlines = int(lidar_buffer_data[lidar_buffer_size + 2])
                    packet_builder.add_int(nlines)

                    for i in range(nlines * 2):
                        if (lidar_buffer_size + 3 + i) < len(lidar_buffer_data): #FIXME: This is a temporary fix
                            packet_builder.add_int(int(lidar_buffer_data[lidar_buffer_size + 3 + i]), 4)

                    nPoints = 1800
                    if (lidar_buffer_size + 3 + nlines * 2) < len(lidar_buffer_data): #FIXME: This is a temporary fix
                        nPoints = int(lidar_buffer_data[lidar_buffer_size + 3 + nlines * 2])
                    packet_builder.add_int(nPoints)

                    for i in range(nPoints * 2):
                        if (lidar_buffer_size + 3 + nlines * 2 + 1 + i) < len(lidar_buffer_data): #FIXME: This is a temporary fix
                            packet_builder.add_float(float(lidar_buffer_data[lidar_buffer_size + 3 + nlines * 2 + 1 + i]), 4)


                for ws in clients.copy():
                    try:
                        await ws.send(packet_builder.get_bytes())
                    except websockets.exceptions.ConnectionClosedOK:
                        LoggerModule.log_warning("Failed to send the message, client was already shutting down")

                # END OF PACKET CONSTRUCTION

                for ws in clients.copy():
                    try:
                        await ws.send(packet_builder.get_bytes())
                    except websockets.exceptions.ConnectionClosedOK:
                        LoggerModule.log_warning("Failed to send the message, client was already shutting down")


                delta_time = time.time() - start_time

                if (delta_time > con.VISUALIZER_MESSAGE_INTERVAL):
                    LoggerModule.log_warning("Visualizer server can't keep up with 'MESSAGE_INTERVAL', consider increasing it ({} ms behind)".format((delta_time - con.VISUALIZER_MESSAGE_INTERVAL) * 1000))

                await asyncio.sleep(max(0, con.VISUALIZER_MESSAGE_INTERVAL - delta_time))

        asyncio.get_event_loop().create_task(update())

        async def connection_handler(websocket, path):
            LoggerModule.log_info("Client connected to the visualizer server ({}:{})".format(websocket.remote_address[0], websocket.remote_address[1]))
            clients.add(websocket)

            packet_builder: PacketBuilder = PacketBuilder()
            packet_builder.new_packet(PacketID.INIT)
            packet_builder.add_int(cls._line_calibration.value, 4)

            for i in range(3):
                packet_builder.add_int(cls._motors_calibration[i], 4)

            packet_builder.add_float(con.FRONT_CAMERA.vertical_crop, 4)

            try:
                await websocket.send(packet_builder.get_bytes())
            except websockets.exceptions.ConnectionClosedOK:
                LoggerModule.log_warning("Failed to send the message, client was already shutting down")


            try:
                async for msg in websocket:
                    object_type = int.from_bytes(msg[0:1], byteorder = "little")
                    message_lenght = int.from_bytes(msg[1:5], byteorder = "little")

                    if (object_type == PacketID.BALL_CALIBRATION.value):
                        target_camera = msg[5]
                        calibration = [x for x in msg[6:12]]

                        if target_camera == 0:
                            with cls._ball_calibration.get_lock():
                                cls._ball_calibration[0:6] = calibration
                        else:
                            with cls._mirror_ball_calibration.get_lock():
                                cls._mirror_ball_calibration[0:6] = calibration

                        cls._calibration_queue.put((0 + 3 * target_camera, calibration))
                        LoggerModule.log_info("Calibration data from client received: {}, {}, {}, {}".format(target_camera, calibration, object_type, message_lenght))

                    elif (object_type == PacketID.BLUE_GOAL_CALIBRATION.value):
                        target_camera = msg[5]
                        calibration = [x for x in msg[6:12]]

                        if target_camera == 0:
                            with cls._blue_goal_calibration.get_lock():
                                cls._blue_goal_calibration[0:6] = calibration
                        else:
                            with cls._mirror_blue_goal_calibration.get_lock():
                                cls._mirror_blue_goal_calibration[0:6] = calibration

                        cls._calibration_queue.put((1 + 3 * target_camera, calibration))
                        LoggerModule.log_info("Calibration data from client received: {}, {}, {}, {}".format(target_camera, calibration, object_type, message_lenght))

                    elif (object_type == PacketID.YELLOW_GOAL_CALIBRATION.value):
                        target_camera = msg[5]
                        calibration = [x for x in msg[6:12]]

                        if target_camera == 0:
                            with cls._yellow_goal_calibration.get_lock():
                                cls._yellow_goal_calibration[0:6] = calibration
                        else:
                            with cls._mirror_yellow_goal_calibration.get_lock():
                                cls._mirror_yellow_goal_calibration[0:6] = calibration


                        cls._calibration_queue.put((2 + 3 * target_camera, calibration))
                        LoggerModule.log_info("Calibration data from client received: {}, {}, {}, {}".format(target_camera, calibration, object_type, message_lenght))

                    elif (object_type == PacketID.LINE_SENSOR_CALIBRATION.value):
                        line_calibration = int.from_bytes(msg[5:9], byteorder = "little")

                        with cls._line_calibration.get_lock():
                            cls._line_calibration.value = line_calibration

                        cls._calibration_queue.put((6, line_calibration))
                        await websocket.send(msg)
                        LoggerModule.log_info("Calibration data from client received: {}, {}, {}".format(line_calibration, object_type, message_lenght))

                    elif (object_type == PacketID.MOTORS_CALIBRATION.value):
                        with cls._line_calibration.get_lock():
                            cls._motors_calibration[0] = int.from_bytes(msg[5:9], byteorder = "little")
                            cls._motors_calibration[1] = int.from_bytes(msg[9:13], byteorder = "little")
                            cls._motors_calibration[2] = int.from_bytes(msg[13:17], byteorder = "little")

                        cls._calibration_queue.put((7, cls._motors_calibration[0:3]))
                        #await websocket.send(msg)
                        LoggerModule.log_info("Calibration data from client received: {}, {}, {}".format(cls._motors_calibration[0:3], object_type, message_lenght))


            except websockets.ConnectionClosedError:
                    LoggerModule.log_warning("Connection unexpectedly closed ({}:{})".format(websocket.remote_address[0], websocket.remote_address[1]))

            finally:
                LoggerModule.log_info("Client disconnected from the visualizer server ({}:{})".format(websocket.remote_address[0], websocket.remote_address[1]))
                clients.remove(websocket)

        ip_address_raw = subprocess.check_output(['hostname', '-I']).decode('ascii').strip().split()

        if ip_address_raw:
            ip_address = ip_address_raw[0]

            start_server = websockets.serve(connection_handler, ip_address, con.VISUALIZER_PORT)
            LoggerModule.log_info("Visualizer server is running on {}:{}".format(ip_address, con.VISUALIZER_PORT))

            asyncio.get_event_loop().run_until_complete(start_server)
            asyncio.get_event_loop().run_until_complete(stop_future)

        else:
            LoggerModule.log_error("No IP adress to run visualizer server")

    @classmethod
    def is_new_calibration(cls) -> bool:
        return (cls._calibration_queue.qsize() != 0)

    @classmethod
    def get_new_calibration(cls) -> tuple:
        return (cls._calibration_queue.get())

    @classmethod
    def set_position_heatmap(cls, heatmap) -> None:
        with cls._position_heatmap.get_lock():
            cls._position_heatmap[0:9] = heatmap
