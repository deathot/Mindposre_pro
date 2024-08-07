#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import time

import numpy as np

from src.actions import SetServo, Stop, Start, TurnLeft, TurnRight, Advance, TurnAround
from src.models import LFNet
from src.scenes.base_scene import BaseScene
from src.utils import log


class LF(BaseScene):
    def __init__(self, memory_name, camera_info, msg_queue):
        super().__init__(memory_name, camera_info, msg_queue)
        self.net = None
        self.forward_spd = 22

    def init_state(self):
        log.info(f'start init {self.__class__.__name__}')
        lfnet_path = os.path.join(os.getcwd(), 'weights', 'lfnet.om')
        if not os.path.exists(lfnet_path):
            log.error(f'Cannot find the offline inference model(.om) file needed for {self.__class__.__name__}  scene.')
            return True
        self.net = LFNet(lfnet_path)
        log.info(f'{self.__class__.__name__} model init succ.')
        self.ctrl.execute(SetServo(servo=[90, 65]))
        return False

    def loop(self):
        ret = self.init_state()
        if ret:
            log.error(f'{self.__class__.__name__} init failed.')
            return
        frame = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.broadcaster.buf)
        log.info(f'{self.__class__.__name__} loop start')
        self.ctrl.execute(Start())
        try:
            while True:
                if self.stop_sign.value:
                    break
                if self.pause_sign.value:
                    continue
                start = time.time()
                img_bgr = frame.copy()
                curr_steering_val = float(self.net.infer(img_bgr)[0])
                log.info(f'lfnet: {curr_steering_val}')

                if 0 < curr_steering_val <= 65:
                    self.ctrl.execute(Advance(speed=self.forward_spd))
                if 65 < curr_steering_val <= 75:
                    self.ctrl.execute(TurnLeft(degree=0.7))
                elif 75 < curr_steering_val <= 80:
                    self.ctrl.execute(TurnLeft(degree=0.6))
                elif 80 < curr_steering_val <= 85:
                    self.ctrl.execute(TurnLeft(degree=0.5))
                elif 85 < curr_steering_val <= 88:
                    self.ctrl.execute(TurnLeft(degree=0.4))
                elif 88 < curr_steering_val <= 92:
                    self.ctrl.execute(Advance(speed=self.forward_spd))
                elif 92 < curr_steering_val <= 95:
                    self.ctrl.execute(TurnRight(degree=0.1))
                elif 95 < curr_steering_val <= 100:
                    self.ctrl.execute(TurnRight(degree=0.2))
                elif 100 < curr_steering_val <= 102:
                    self.ctrl.execute(TurnRight(degree=0.3))
                elif 102 < curr_steering_val <= 104:
                    self.ctrl.execute(TurnRight(degree=0.3))
                elif 104 < curr_steering_val <= 108:
                    self.ctrl.execute(TurnRight(degree=0.3))
                elif 108 < curr_steering_val <= 112:
                    self.ctrl.execute(TurnRight(degree=0.4))
                elif 112 < curr_steering_val <= 115:
                    self.ctrl.execute(TurnRight(degree=0.5))
                elif 115 < curr_steering_val <= 120:
                    self.ctrl.execute(TurnRight(degree=0.6))
                elif 120 < curr_steering_val <= 135:
                    self.ctrl.execute(TurnRight(degree=0.7))


                log.info(f'infer cost {time.time() - start}')
        except KeyboardInterrupt:
            self.ctrl.execute(Stop())
