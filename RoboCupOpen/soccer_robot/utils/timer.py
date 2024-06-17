import time

class Timer():
    def __init__(self) -> None:
        self._start_timepoint = time.time()

    def reset(self) -> None:
        self._start_timepoint = time.time()

    def get(self) -> float:
        return (time.time() - self._start_timepoint) * 1000
