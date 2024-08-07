import datetime
import os

import cv2
import numpy as np

from src.actions import MoveForward, SetServo, Stop, MoveBack, BaseAction
from src.scenes.base_scene import BaseScene
from src.utils import log


class Manual(BaseScene):
    def __init__(self, memory_name, camera_info, msg_queue):
        super().__init__(memory_name, camera_info, msg_queue)
        self.speed = 100
        self.save_dir = os.path.join(os.getcwd(), 'capture')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir, exist_ok=True)

    def init_state(self):
        self.ctrl.execute(SetServo(servo=[91, 10]))

    def loop(self):
        ret = self.init_state()
        if ret:
            log.error(f'{self.__class__.__name__} init failed.')
            return
        frame = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.broadcaster.buf)
        log.info(f'{self.__class__.__name__} loop start')
        last_action = SetServo(servo=[91, 10])

        while True:
            try:
                if not self.msg_queue.empty():
                    key = self.msg_queue.get()
                else:
                    continue
            except KeyboardInterrupt:
                self.ctrl.execute(Stop())
                break

            # degree为z轴的角度数，正数为左转，负数为右转
            degree = 0
            if key == 'up':
                self.speed = min(self.speed + 1, 100)
            elif key == 'down':
                self.speed = max(self.speed - 1, 100)
            elif key == 'right':
                last_action = SetServo(servo=[91, 10])
            elif key == 'left':
                last_action = SetServo(servo=[91, 90])
            elif key == 'w':
                last_action = MoveForward(x=self.speed)
            elif key == 'a':
                last_action = MoveForward()
                degree = 40
            elif key == 's':
                last_action = MoveBack(x=self.speed)
            elif key == 'd':
                last_action = MoveForward()
                degree = -40
            elif key == 'space':
                last_action = Stop()
            elif key == 'esc':
                self.ctrl.execute(Stop())
                break
            elif key == 'c':
                save_img = frame.copy()
                cv2.imwrite(os.path.join(self.save_dir, f'{datetime.datetime.now()}.jpg'), save_img)
                log.info(f'image saved.')
            else:
                continue

            if isinstance(last_action, BaseAction):
                last_action.update_z_speed = False
                last_action.update_x_speed = False
                last_action.z_speed = degree
                last_action.speed_setting = last_action.generate_speed_setting(self.speed, degree)
            self.ctrl.execute(last_action)
