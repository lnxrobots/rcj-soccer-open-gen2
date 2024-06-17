# Here you can find example Modules for reference purposes
from soccer_robot.logger.logger_module import LoggerModule

import time
import multiprocessing
import os

from soccer_robot.module.module import *

# Modula class template
class TemplateModule(Module):
    # Set name of your module for debugging purposes
    module_name = "TemplateModule"

    @classmethod
    def init(cls) -> None:
        # It is called when the process is initialized
        # You can define your custom shared variables here
        super().init()
        cls._example_double = multiprocessing.Value("d", 8.0)

    @classmethod
    def on_run(cls, stop_flag) -> None:
        # This function is called in the other process
        while not stop_flag.value:
            cls.get_example_double.value += 1

    @classmethod
    def on_stop(cls) -> None:
        # It is called when the process stops (still on process thread)
        LoggerModule.log_info("{} stopped".format(cls.module_name))

    @classmethod
    def get_example_double(cls) -> float:
        # You should also define getter for each of your custom variable
        with cls._example_double.get_lock():
            val = cls._example_double.value
        return val

# Example module usage
class FileReaderModule(Module):
    module_name = "FileReaderModule"

    @classmethod
    def init(cls) -> None:
        super().init()
        cls._number_in_file = multiprocessing.Value("d", -1.0)

        cls._file = open("exampleFile.txt", "w")
        cls._file.write("64")
        cls._file.close()

    @classmethod
    def on_run(cls, _stop_flag):
        file = open("exampleFile.txt", "r")

        while not _stop_flag.value:
            file.seek(0)
            file_string = file.read()
            if file_string:
                cls._number_in_file.value = float(file_string)
            time.sleep(0.5)

        file.close()

    @classmethod
    def on_stop(cls) -> None:
        os.remove("exampleFile.txt")

    @classmethod
    def get_number_in_file(cls):
        with cls._number_in_file.get_lock():
            val = cls._number_in_file.value
        return val

class ConsoleLoggerModule(Module):
    module_name = "ConsoleLoggerModule"

    @classmethod
    def on_run(cls, _stop_flag):
        while not _stop_flag.value:
            LoggerModule.log_info(FileReaderModule.get_number_in_file())
            time.sleep(1)
