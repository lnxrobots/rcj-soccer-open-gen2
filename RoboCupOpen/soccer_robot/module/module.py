import multiprocessing
import multiprocessing.managers

import sys
import signal
import traceback
import time

class Module:
    def __new__(cls) -> 'Module':
        raise RuntimeError("Module: '{}' should not be instantiated, use class methods to control this module!".format(cls.module_name))

    module_name = "UndefinedModuleName"
    _inited = False

    # This method is called to init the Module
    # You may override it ('see example_modules.py'), mainly for adding your own shared variables
    # This method is called on the main thread (process)
    @classmethod
    def init(cls) -> None:
        if cls._inited:
            raise RuntimeError("Module: '{}' is already inited!".format(cls.module_name))

        cls._stop_flag = multiprocessing.Value("b", False)
        cls._failed = multiprocessing.Value("b", False)
        cls._native_process = multiprocessing.Process(target = cls._process_target_function, args = (cls._stop_flag,))
        cls._update_frequency = multiprocessing.Value("i", 60)
        cls._inited = True

    # Called when the module is started, should be overriden to add your own functionality
    # This method is called in the separate module thread (process)
    @classmethod
    def on_start(cls) -> None:
        pass

    # Called repeatedly based on update frequency, should be overriden to add your own functionality
    # This method is called in the separate module thread (process)
    @classmethod
    def on_update(cls) -> None:
        pass

    # Called when the module stops or when the terminate() is called
    # This method is called in the separate module thread (process)
    @classmethod
    def on_stop(cls) -> None:
        pass

    # Starts separate process
    @classmethod
    def launch(cls) -> None:
        cls._stop_flag.value = False
        cls._active = True
        cls._native_process.start()

    # "Nice" stop
    @classmethod
    def stop(cls) -> None:
        cls._stop_flag.value = True

    # "Force" stop
    @classmethod
    def terminate(cls) -> None:
        cls._native_process.terminate()

    @classmethod
    def get_pid(cls) -> int:
        return cls._native_process.pid

    @classmethod
    def is_inited(cls) -> bool:
        return cls._inited

    @classmethod
    def has_failed(cls) -> bool:
        return cls._failed.value

    @classmethod
    def is_active(cls) -> bool:
        return cls._native_process.is_alive()

    @classmethod
    def set_update_freqeuncy(cls, update_frequency: float) -> None:
        with cls._update_frequency.get_lock():
            cls._update_frequency.value = update_frequency

    @classmethod
    def _process_target_function(cls, stop_flag: multiprocessing.Value) -> None:
        signal.signal(signal.SIGTERM, cls._sigterm_signal)

        from soccer_robot.logger.logger_module import LoggerModule

        try:
            cls.on_start()

            # Since we do not want modules to take 100% of the core, but we want it to be updated fixed number of times
            # per second, a system has to be implemented to adaptively sleep the process
            sleepTime = 0
            sleepStartTime = time.perf_counter()

            while not stop_flag.value:
                taskStartTime = time.perf_counter()
                sleepDelay = taskStartTime - (sleepStartTime + sleepTime)

                cls.on_update()

                if cls._update_frequency.value == -1:
                    continue

                elapsedTime = time.perf_counter() - taskStartTime
                sleepTime = 1.0 / cls._update_frequency.value - elapsedTime

                if sleepTime < 0:
                    #LoggerModule.log_warning("{} cannot keep up with its update frequency ({})".format(cls.module_name, cls._update_frequency.value))
                    sleepTime = 0
                else:
                    sleepTime -= sleepDelay

                sleepStartTime = time.perf_counter()

                time.sleep(max(sleepTime, 0))

        except Exception as e:
            cls._failed.value = True

            LoggerModule.log_critical("{} has failed".format(cls.module_name))
            LoggerModule.log_critical(traceback.format_exc())

        cls.on_stop()
        sys.exit()

    @classmethod
    def _sigterm_signal(cls, sig, frame):
        cls.on_stop()
        sys.exit()
