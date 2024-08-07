#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
from abc import ABC, abstractmethod


class BaseAction(ABC):
    """
    基础动作的基类，所有基本动作均继承于该类
    """

    def __init__(self, *args, **kwds) -> None:
        """
        基础动作类的初始化方法，通过args与kwds控制输入参数
        :param args:
        :param kwds:
        """
        # 抽象的速度信息
        # x为线速度，z为角速度
        self.x_speed = kwds.get('x', -1)
        self.z_speed = kwds.get('z', -1)

        # 电机角度
        self.servo_angle = kwds.get('servo', [-1, -1])

        # 确定是否需要在运行时根据前动作更新电机角度及电机速度
        self.update_x_speed = False
        self.update_z_speed = False
        self.update_servo = False

        if self.x_speed == -1:
            self.update_x_speed = True

        if self.z_speed == -1:
            self.update_z_speed = True

        if self.servo_angle[0] == -1 and self.servo_angle[1] == -1:
            self.update_servo = True

        # 初步生成两个速度信息和两个pose信息
        self.speed_setting = self.generate_speed_setting(self.x_speed,self.z_speed)



    @staticmethod
    @abstractmethod
    def generate_speed_setting(x_speed, z_speed):
        """
        抽象类，需要根据具体情况进行设置
        :param x_speed: x方向的线速度
        :param z_speed: z方向的角速度
        :return:
        """
        pass

    def __call__(self, x_speed, z_speed, servo_angle):
        """
        call魔法函数，两个输入参数由控制器输入
        当init方法设置了相关信息，则忽略控制器输入的参数
        当init方法没有设置相关信息，相关信息的将由控制器输入的参数进行更新

        :param servo_angle: 舵机的角度
        :param x_speed: x方向的线速度
        :param z_speed: z方向的角速度
        """
        if self.update_servo:
            self.servo_angle = servo_angle
        if self.update_x_speed:
            self.x_speed = x_speed
        if self.update_z_speed:
            self.z_speed = z_speed

        self.speed_setting = self.generate_speed_setting(self.x_speed, self.z_speed)

        return self.speed_setting + self.servo_angle


class MoveForward(BaseAction):
    """
    Dogee前进，x_speed为线速度，z_speed为角速度
    """

    @staticmethod
    def generate_speed_setting(x_speed, z_speed):
        return [x_speed, z_speed, 0, 0]


class MoveBack(BaseAction):
    """
    Dogee后退，x_speed为线速度，z_speed为角速度
    """

    @staticmethod
    def generate_speed_setting(x_speed, z_speed):
        return [-x_speed, z_speed, 0, 0]


class SetServo(BaseAction):
    """
    舵机转动
    """

    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self.x_speed = 0
        self.z_speed = 0

    @staticmethod
    def generate_speed_setting(x_speed, z_speed):
        return [0, 0, 0, 0]


class Stop(BaseAction):
    """
    Dogee停止
    """
    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self.x_speed = 0
        self.z_speed = 0

    @staticmethod
    def generate_speed_setting(x_speed, z_speed):
        return [0, 0, 0, 0]


class Sleep(BaseAction):
    """
    Sleep(1)等同于time.sleep(1)
    可加入至动作序列进行使用
    """

    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self.sleep_time = args[0]

    @staticmethod
    def generate_speed_setting(speed, degree=0):
        return []

    def __call__(self, x_speed, z_speed, servo_angle):
        time.sleep(self.sleep_time)
        return None
