#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from abc import ABC

from src.actions.base_action import Advance, Sleep, SpinAntiClockwise, Stop, SpinClockwise, CustomAction, ShiftLeft, \
    TurnRight


class ComplexAction(ABC):
    def __init__(self, ):
        # 当前动作是否强制执行，默认为强制执行
        self.force = True
        # 当前动作序列是否更新控制器记录的速度，默认为不更新
        self.update_controller_speed = False
        pass


class TurnLeftInPlace(ComplexAction):
    def __init__(self):
        super().__init__()
        self.action_seq = [
            Advance(speed=30),
            Sleep(0.8),
            SpinAntiClockwise(speed=40),
            Sleep(0.5),
            Stop()
        ]


class TurnRightInPlace(ComplexAction):
    def __init__(self):
        super().__init__()
        self.action_seq = [
            Advance(speed=30),
            Sleep(0.55),
            SpinClockwise(speed=40),
            Sleep(0.48),
            Stop()
        ]


class TurnAround(ComplexAction):
    def __init__(self):
        super().__init__()
        self.action_seq = [
            Stop(),
            Sleep(0.1),
            Advance(speed=30),
            Sleep(1.55),
            Stop(),
            Sleep(0.3),
            SpinAntiClockwise(speed=50),
            Sleep(0.4),
            Advance(speed=30),
            Sleep(0.9),
            SpinAntiClockwise(speed=50),
            Sleep(0.45),
            Stop(),

        ]


class Start(ComplexAction):
    def __init__(self):
        super().__init__()
        self.update_controller_speed = True
        self.action_seq = [
            Advance(speed=35),
            Sleep(0.2),
            Advance(speed=25)
        ]


class Parking(ComplexAction):
    def __init__(self):
        super().__init__()
        self.action_seq = [
            Stop(),
            Sleep(1),

            CustomAction(motor_setting=[-85, 65, 60, -55]),
            Sleep(0.75),
            Stop(),

            Sleep(2),
            CustomAction(motor_setting=[65, -58, -55, 55]),
            Sleep(1),
            Stop()
        ]
