#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import atexit
import os
import pickle
import shutil
import time

import serial
from filelock import FileLock

from src.actions.base_action import BaseAction
from src.utils.common_utils import SingleTonType
from src.utils.logger import logger_instance as log
from src.utils.init_utils import get_port
from src.utils.constant import ESP32_NAME,STM32_NAME


class Controller(metaclass=SingleTonType):
    """
    由单例和文件锁保证的全局唯一的控制器
    支持执行动作序列及单个动作

    """

    # 保证所有进程内的控制器内的参数保持一致
    def _set_gloabl_unique(self):
        with self._lock:
            if not os.path.exists(self._ser_path):
                self.reset()
                self._save()
            else:
                try:
                    self._update()
                except EOFError:
                    self.reset()
                    self._save()
            self._inc_count()

    def __init__(self) -> None:
        # 获取当前用户的home目录
        home = os.path.expanduser('~')

        # 新建临时文件夹
        self._temp_path = os.path.join(home, 'temp', 'controller')
        if not os.path.exists(self._temp_path):
            os.makedirs(self._temp_path, exist_ok=True)
        # 设置序列化文件保存路径，文件锁路径，全局引用计数路径
        self._ser_path = os.path.join(self._temp_path, 'controller.pickle')
        self._lock_path = os.path.join(self._temp_path, 'controller.lock')
        self._count_path = os.path.join(self._temp_path, 'controller.count')

        # 创建文件锁
        self._lock = FileLock(self._lock_path)

        # esp32串口信息
        esp32_port = get_port(ESP32_NAME)
        self._esp32_ser = serial.Serial(esp32_port, 115200)
        self._esp32_check_val = -12345

        # stm32串口信息
        stm32_port = get_port(STM32_NAME)
        self._stm32_ser = serial.Serial(stm32_port, 115200)
        self._stm32_state_head = str.encode('\x7B')
        self._stm32_state_tail = str.encode('\x7D')

        # 需要保存的信息，分别为最后一次动作成功下发的时间，当前的线速度与角速度，当前的舵机角度
        self.last_modify_time = 0
        self.x_speed = 0
        self.z_speed = 0
        self.servo_angle = [0, 0]

        # 设置并更新序列化信息
        self._set_gloabl_unique()

        # 设置调用__del__方法前调用的去初始化方法
        atexit.register(self._deinit)

    @staticmethod
    def calculate_check_val(state):
        check_num = 0
        for val in state:
            check_num ^= val
        return check_num

    # 重置电机与舵机状态
    def reset(self):
        self.servo_angle = [90, 10]
        esp32_state = self.servo_angle
        stm32_state = [0] * 4
        self.send_to_device(esp32_state, stm32_state)

    # 执行动作序列
    def execute(self, action):
        # 获取文件锁
        with self._lock:
            # 从序列化文件更新当前控制器的相关参数
            self._update()

            # 获取输入action的相关attr
            attr_dict = action.__dict__
            force = attr_dict.get('force', None)
            send_time = attr_dict.get('send_time', None)
            action_seq = attr_dict.get('action_seq', None)
            update_controller_speed = attr_dict.get('update_controller_speed', True)
            log.info(f'update_controller_speed: {update_controller_speed}')
            log.info(f'self.z_speed: {self.z_speed}')
            log.info(f'self.x_speed: {self.x_speed}')

            # 判断是否丢弃过时的动作
            # 如果动作序列带有force（强制执行）属性将无视动作下发时间
            # 如果动作序列带有下发时间，且动作序列下发时间早于最后一次动作的执行时间，当前动作序列将被丢弃
            if force is None or not force:
                if send_time is not None and isinstance(send_time, float) and self.last_modify_time > send_time:
                    return -1

            # 判断输入动作是否为动作序列或单个简单动作
            if action_seq is None and isinstance(action, BaseAction):
                action_seq = [action]

            # 执行动作序列
            for action in action_seq:
                # 调用动作的__call__方法
                state = action(self.x_speed, self.z_speed, self.servo_angle)
                # 如果当前动作不需要下发指令至ESP32或STM32，跳过后续步骤
                if state is None:
                    continue
                esp32_state = state[-2:]
                stm32_state = state[:-2]

                # 下发指令并获取到修改的时间戳
                cost_time, modify_time = self.send_to_device(esp32_state, stm32_state)
                log.info(f'action {action.__class__.__name__} execute cost {cost_time} s')

                # 更新控制器相关信息
                self.last_modify_time = modify_time
                if update_controller_speed:
                    if action.z_speed != -1:
                        self.z_speed = action.z_speed
                    if action.x_speed != -1:
                        self.x_speed = action.x_speed
                if action.servo_angle != [-1, -1]:
                    self.servo_angle = action.servo_angle

            # 保存控制器的相关信息
            self._save()
            return 0

    def esp32_encoder(self, state):
        state = [0] * 4 + state + [self._esp32_check_val]
        msg = b''.join([num.to_bytes(2, byteorder='little', signed=True) for num in state])
        return msg

    def stm32_encoder(self, state):
        msg = self._stm32_state_head + b''.join([num.to_bytes(2, byteorder='big', signed=True) for num in state])
        msg += self.calculate_check_val(msg).to_bytes(1, byteorder='big', signed=True)
        msg += self._stm32_state_tail
        return msg

    def send_to_device(self, esp32_state, stm32_state):
        start = time.time()
        # 下发编码好的信息至stm32
        stm32_msg = self.stm32_encoder(stm32_state)
        try:
            self._stm32_ser.write(stm32_msg)
        except Exception:
            stm32_port = None
            while stm32_port is None:
                stm32_port = get_port(STM32_NAME)
            self._stm32_ser = serial.Serial(stm32_port, 115200)
            self._stm32_ser.write(stm32_msg)

        log.debug(f'stm32 state: {stm32_state}')

        # 下发编码好的14bytes信息至esp32
        esp32_msg = self.esp32_encoder(esp32_state)
        try:
            self._esp32_ser.write(esp32_msg)
        except Exception:
            esp32_port = None
            while esp32_port is None:
                esp32_port = get_port(ESP32_NAME)
            self._esp32_ser = serial.Serial(esp32_port, 115200)
            self._esp32_ser.write(esp32_msg)
        log.debug(f'esp32 state: {esp32_state}')

        end = time.time()
        log.debug(f'action execute cost: {end - start}s')
        return end-start, time.time()

    def recv_from_device(self):
        return self._esp32_ser.readline()

    # 从序列化文件中获取参数并更新当前控制器的相关参数
    def _update(self):
        with open(self._ser_path, 'rb') as f:
            attr_dict = pickle.load(f)
            for k, v in attr_dict.items():
                setattr(self, k, v)

    # 保存当前控制器的相关参数至序列化文件
    def _save(self):
        with open(self._ser_path, 'wb') as f:
            pickle.dump(self.get_public_var(), f)

    # 增加全局引用计数
    def _inc_count(self):
        count = 1
        if os.path.exists(self._count_path):
            with open(self._count_path, 'r') as f:
                count = int(f.readline()) + 1

        with open(self._count_path, 'w') as f:
            f.write(str(count))

    # 释放资源前减少全局引用计数
    def _dec_count(self):
        count = 0
        if os.path.exists(self._count_path):
            with open(self._count_path, 'r') as f:
                count = int(f.readline()) - 1

            if count > 0:
                with open(self._count_path, 'w') as f:
                    f.write(str(count))
                return False
            else:
                return True
        else:
            raise RuntimeError('Cannot find the processes count file.')

    # 过滤出需要保存的参数
    def get_public_var(self):
        dic = self.__dict__
        public_var = {key: value for key, value in dic.items() if not key.startswith('_')}
        return public_var

    # 在__del__方法被调用前执行的去初始化方法
    def _deinit(self):
        with self._lock:
            finalize = self._dec_count()
        # 引用记数归零则删除临时文件路径,并发送指令保证电机停转
        if finalize:
            self.reset()
            shutil.rmtree(self._temp_path)
