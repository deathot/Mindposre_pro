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
        self.speed = kwds.get('speed', -1)
        # 电机角度
        self.servo_angle = kwds.get('servo', [-1, -1])

        # 根据电机的实际情况修改下发到电机的速度
        self.motor_rating = [1.55, 1.3, 1, 1]

        # 确定是否需要在运行时根据前动作更新电机角度及电机速度
        self.update_speed = False
        self.update_servo = False

        if self.speed == -1:
            self.update_speed = True

        if self.servo_angle[0] == -1 and self.servo_angle[1] == -1:
            self.update_servo = True

        # 由速度生成方法将抽象的总体速度计算为4个电机的速度并输出为list
        self.speed_setting = self.generate_speed_setting(self.speed)
        self.fix_speed()

    def fix_speed(self):
        self.speed_setting = [int(speed * ratio) for speed, ratio in zip(self.speed_setting, self.motor_rating)]

    @staticmethod
    @abstractmethod
    def generate_speed_setting(speed, degree=0):
        """
        生成4个电机的速度，并输出为列表
        抽象类，需要根据具体情况进行设置
        :param speed: 抽象的速度。 当前动作初始化时设置 或 控制器根据前一动作速度进行设置
        :param degree: 如需转弯，速度计算需要的角度信息
        :return:
        """
        pass

    def __call__(self, speed, servo_angle):
        """
        call魔法函数，两个输入参数由控制器输入
        当init方法设置了相关信息，则忽略控制器输入的参数
        当init方法没有设置相关信息，相关信息的将由控制器输入的参数进行更新

        :param speed: 抽象速度
        :param servo_angle: 舵机的角度
        :return: 长度为6的列表，前4位为4个电机的速度，后2位为舵机的两个角度
        """
        if self.update_servo:
            self.servo_angle = servo_angle
        if self.update_speed:
            degree = 0
            if hasattr(self, 'degree'):
                degree = self.degree
            self.speed_setting = self.generate_speed_setting(speed, degree)
            self.fix_speed()

        return self.speed_setting + self.servo_angle


class Advance(BaseAction):
    """
    小车前进
    """

    @staticmethod
    def generate_speed_setting(speed, degree=0):
        return [-speed, -speed, speed, speed]


class BackUp(BaseAction):
    """
    小车后退
    """

    @staticmethod
    def generate_speed_setting(speed, degree=0):
        return [speed, speed, -speed, -speed]


class CustomAction(BaseAction):
    """
    自定义动作
    """

    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self.speed_setting = kwds.get('motor_setting', [0, 0, 0, 0])
        self.update_controller_speed = False
        self.update_speed = False

    @staticmethod
    def generate_speed_setting(speed, degree=0):
        return [0, 0, 0, 0]


class Stop(BaseAction):
    """
    小车停止
    """

    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self.speed = 0

    @staticmethod
    def generate_speed_setting(speed, degree=0):
        return [0, 0, 0, 0]


class TurnLeft(BaseAction):
    """
    小车左转
    """

    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self.degree = kwds.get('degree', 0)
        self.speed_setting = self.generate_speed_setting(speed=self.speed, degree=self.degree)
        self.fix_speed()

    @staticmethod
    def generate_speed_setting(speed, degree=0):
        return [-speed, -speed, int(speed * (1 + degree)), int(speed * (1 + degree))]


class TurnRight(BaseAction):
    """
    小车右转
    """

    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self.degree = kwds.get('degree', 0)
        self.speed_setting = self.generate_speed_setting(speed=self.speed, degree=self.degree)
        self.fix_speed()

    @staticmethod
    def generate_speed_setting(speed, degree=0):
        return [-int(speed * (1 + degree)), -int(speed * (1 + degree)), speed, speed]


class ShiftLeft(BaseAction):
    """
    向左平移
    """

    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self.motor_rating = [1.5, 1.3, 1.25, 1.25]
        self.fix_speed()

    @staticmethod
    def generate_speed_setting(speed, degree=0):
        return [speed, -speed, -speed, speed]


class ShiftRight(BaseAction):
    """
    向右平移
    """

    @staticmethod
    def generate_speed_setting(speed, degree=0):
        return [-speed, speed, speed, -speed]


class LeftOblique(BaseAction):
    """
    斜向左前方
    """

    @staticmethod
    def generate_speed_setting(speed, degree=0):
        return [0, -speed, 0, speed]


class RightOblique(BaseAction):
    """
    斜向右前方
    """

    @staticmethod
    def generate_speed_setting(speed, degree=0):
        return [-speed, 0, speed, 0]


class SpinClockwise(BaseAction):
    """
    顺时针旋转
    """

    @staticmethod
    def generate_speed_setting(speed, degree=0):
        return [-speed] * 4


class SpinAntiClockwise(BaseAction):
    """
    逆时针旋转
    """

    @staticmethod
    def generate_speed_setting(speed, degree=0):
        return [speed] * 4


class SetServo(BaseAction):
    """
    舵机转动
    """

    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self.speed = 0

    @staticmethod
    def generate_speed_setting(speed, degree=0):
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

    def __call__(self, speed, servo_angle):
        time.sleep(self.sleep_time)
        return None
