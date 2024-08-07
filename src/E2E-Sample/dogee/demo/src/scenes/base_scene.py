#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from abc import ABC, abstractmethod
from ctypes import c_bool
from multiprocessing import shared_memory, Value

from src.utils import Controller


class BaseScene(ABC):
    def __init__(self, memory_name, system_info, msg_queue):
        self.pause_sign = Value(c_bool, False)
        self.stop_sign = Value(c_bool, False)
        self.ctrl = Controller()
        self.msg_queue = msg_queue
        self.broadcaster = shared_memory.SharedMemory(name=memory_name)
        self.system_info = system_info
        self.height = self.system_info.height
        self.width = self.system_info.width
        self.fps = self.system_info.fps

    @abstractmethod
    def init_state(self):
        pass

    @abstractmethod
    def loop(self):
        pass
