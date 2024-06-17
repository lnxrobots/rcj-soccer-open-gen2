from soccer_robot.module.module import Module
from soccer_robot.constants import LOGGER_LOGS_PATH, LOGGER_MAX_QUEUE_SIZE, LOGGER_FILE_LOGLEVEL, LOGGER_CONSOLE_LOGLEVEL

import multiprocessing
import logging
import os
import time
import datetime
import inspect

class LoggerModule(Module):
    module_name = "LoggerModule"

    @classmethod
    def init(cls) -> None:
        super().init()
        cls._message_queue = multiprocessing.Queue(LOGGER_MAX_QUEUE_SIZE)

        cls._native_logger_messages = logging.getLogger("MessageLogger")

    @classmethod
    def stop(cls) -> None:
        super().stop()

    @classmethod
    def on_start(cls):
        # Prepare message logger
        cls._native_logger_messages.setLevel(logging.DEBUG)

        file_handler = logging.FileHandler(cls._prepare_log_file())
        file_handler.setLevel(LOGGER_FILE_LOGLEVEL)

        stream_handler = logging.StreamHandler()
        stream_handler.setLevel(LOGGER_CONSOLE_LOGLEVEL)

        cls._native_logger_messages.addHandler(file_handler)
        cls._native_logger_messages.addHandler(stream_handler)

    @classmethod
    def on_update(cls) -> None:
        while cls._message_queue.qsize() > 0:
            message = cls._message_queue.get()

            formatted_time = time.strftime("%H:%M:%S.", time.localtime(message[3])) + "{:.3f}".format(message[3] % 1, 3)[2:]
            formatted_message = "[{}] [{}/{}]: {}".format(formatted_time, message[2], logging._levelToName[message[0]], message[1])

            from soccer_robot.visualization.visualizer_module import VisualizationModule

            #if VisualizationModule.is_inited():
                #VisualizationModule.add_log(formatted_message)

            cls._native_logger_messages.log(message[0], formatted_message)

    @classmethod
    def log_debug(cls, message):
        filename = os.path.basename(inspect.stack()[1].filename)
        cls._message_queue.put((logging.DEBUG, message, filename, time.time()))

    @classmethod
    def log_info(cls, message):
        filename = os.path.basename(inspect.stack()[1].filename)
        cls._message_queue.put((logging.INFO, message, filename, time.time()))

    @classmethod
    def log_warning(cls, message):
        filename = os.path.basename(inspect.stack()[1].filename)
        cls._message_queue.put((logging.WARNING, message, filename, time.time()))

    @classmethod
    def log_error(cls, message):
        filename = os.path.basename(inspect.stack()[1].filename)
        cls._message_queue.put((logging.ERROR, message, filename, time.time()))

    @classmethod
    def log_critical(cls, message):
        filename = os.path.basename(inspect.stack()[1].filename)
        cls._message_queue.put((logging.CRITICAL, message, filename, time.time()))

    @classmethod
    def _prepare_log_file(cls, name = "", symlink = True) -> str:
        if not os.path.exists(LOGGER_LOGS_PATH):
            raise FileNotFoundError(LOGGER_LOGS_PATH)

        logs_path = LOGGER_LOGS_PATH + "/logs"

        if not os.path.exists(logs_path):
            os.mkdir(logs_path)

        log_file = logs_path + "/" + datetime.datetime.now().strftime("%Y-%m-%d-%H_%M_%S") + name + ".log"

        open(log_file, "a").close()

        if (symlink):
            if os.path.exists(logs_path + "/latest.log"):
                os.remove(logs_path + "/latest.log")

            os.symlink(os.path.abspath(log_file), logs_path + "/latest.log")

        return log_file
