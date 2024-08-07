#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np

DEVICE_ID = 0

SUCCESS = 0
FAILED = 1

ACL_MEM_MALLOC_HUGE_FIRST = 0
ACL_MEM_MALLOC_NORMAL_ONLY = 2
ACL_MEMCPY_DEVICE_TO_DEVICE = 3

LOG_NAME = 'logs'
LOG_TYPE = 'Dogee'

GB = 1024 * 1024 * 1024

ACL_FLOAT = 0
ACL_FLOAT16 = 1
ACL_INT8 = 2
ACL_INT32 = 3
ACL_UINT8 = 4
ACL_INT16 = 6
ACL_UINT16 = 7
ACL_UINT32 = 8
ACL_INT64 = 9
ACL_UINT64 = 10
ACL_DOUBLE = 11
ACL_BOOL = 12

FIND_PORT_CODE = '''
#!/bin/bash
for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && exit
        eval "$(udevadm info -q property --export -p $syspath)"
        [[ -z "$ID_SERIAL" ]] && exit
        echo "/dev/$devname - $ID_SERIAL"
    )
done
'''

STM32_NAME = 'Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001'
ESP32_NAME = '1a86_USB_Serial'

STATE_HEAD = '\x7B'
STATE_TAIL = '\x7D'

# 状态观测矩阵
STATE_OBSERVATION_MATRIX = np.eye(6)

# 状态转移矩阵，上一时刻的状态转移到当前时刻
STATE_TRANSITION_MATRIX = np.array([[1, 0, 0, 0, 1, 0],
                                    [0, 1, 0, 0, 0, 1],
                                    [0, 0, 1, 0, 0, 0],
                                    [0, 0, 0, 1, 0, 0],
                                    [0, 0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 0, 1]])

# 过程噪声协方差矩阵Q，p(w)~N(0,Q)，噪声来自真实世界中的不确定性,
# 在跟踪任务当中，过程噪声来自于目标移动的不确定性（突然加速、减速、转弯等）
PROCESS_NOISE_COVARIANCE_MATRIX = np.eye(6) * 0.1

# 观测噪声协方差矩阵R，p(v)~N(0,R)
# 观测噪声来自于检测框丢失、重叠等
OBSERVATION_NOISE_COVARIANCE_MATRIX = np.eye(6) * 0.1
