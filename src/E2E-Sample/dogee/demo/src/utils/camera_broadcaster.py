from ctypes import c_bool
from datetime import datetime
from multiprocessing import shared_memory, Value

import cv2
import numpy as np

from src.utils.logger import logger_instance as log


class CameraBroadcaster:
    def __init__(self, system_info):
        self.height = system_info.height
        self.width = system_info.width
        self.fps = system_info.fps
        self.stop_sign = Value(c_bool, False)
        self.frame = shared_memory.SharedMemory(create=True, size=np.zeros(shape=(self.height, self.width, 3),
                                                                           dtype=np.uint8).nbytes)
        self.memory_name = self.frame.name

    def run(self):
        cap = cv2.VideoCapture()
        cap.open(0, apiPreference=cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        sender = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.frame.buf)

        try:
            while True:
                if self.stop_sign.value:
                    self.frame.close()
                    self.frame.unlink()
                    break
                start = datetime.now()
                ret, frame = cap.read()
                end1 = datetime.now()
                sender[:] = frame[:]
                end2 = datetime.now()
                log.debug(f'{self.memory_name}  read time: {end1 - start}, copy time: {end2 - end1}')
        except (KeyboardInterrupt, SystemExit):
            log.info('Cam broadcaster closing')
            self.frame.close()
            self.frame.unlink()
