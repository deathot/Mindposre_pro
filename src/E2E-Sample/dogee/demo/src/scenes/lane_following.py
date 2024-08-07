#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import time

import numpy as np

from src.actions import SetServo, Stop
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
        # 设置舵机水平角度90度，垂直角度10度
        self.ctrl.execute(SetServo(servo=[90, 10]))
        return False

    def loop(self):
        ret = self.init_state()
        if ret:
            log.error(f'{self.__class__.__name__} init failed.')
            return
        frame = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.broadcaster.buf)
        log.info(f'{self.__class__.__name__} loop start')
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

                log.info(f'infer cost {time.time() - start}')
        except KeyboardInterrupt:
            self.ctrl.execute(Stop())
