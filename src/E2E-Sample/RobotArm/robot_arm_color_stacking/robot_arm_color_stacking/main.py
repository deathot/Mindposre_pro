#!/usr/bin/env python3
# coding: utf-8

import Arm_Lib
import cv2 as cv
import threading
from time import sleep
from .utils.dofbot_config import Arm_Calibration, read_XYT
from .utils.stacking_target import stacking_GetTarget
from pathlib import Path
import os
import numpy as np


def main(args=None):
    # 创建获取目标实例
    target = stacking_GetTarget()
    # 创建相机标定实例
    calibration = Arm_Calibration()
    # 初始化一些参数
    num=0
    dp = []
    xy=[90,135]
    msg = {}
    threshold = 140
    color_list = {}
    color_hsv  = {"red"   : ((0, 43, 46), (10, 255, 255)),
                "green" : ((35, 43, 46), (77, 255, 255)),
                "blue"  : ((100, 43, 46), (124, 255, 255)),
                "yellow": ((26, 43, 46), (34, 255, 255))}

    DP_PRINT = False # 后续作为ROS参数
    
    FILE = Path(__file__).resolve()
    lib_root = FILE.parents[0]
    lib_site_pkg = os.path.dirname(lib_root)
    lib_python = os.path.dirname(lib_site_pkg)
    lib_path = os.path.dirname(lib_python)
    shared_path = os.path.join(os.path.dirname(lib_path), "share")
    share_root = os.path.join(shared_path, "robot_arm_color_stacking")
    cfg_folder = os.path.join(share_root, 'config')
    dp_cfg_path = os.path.join(cfg_folder, 'dp.bin')

    # XYT参数路径
    XYT_path = os.path.join(cfg_folder, 'XYT_config.txt') # revise

    try: 
        xy, thresh = read_XYT(XYT_path)
    except Exception: 
        print("No XYT_config file!!!")
        
    print("Read xy is", xy)
        
    warm_up_count = 0
    last_num = 0
    last_count = 0
        
    # 创建机械臂驱动实例
    arm = Arm_Lib.Arm_Device()
    joints_0 = [xy[0], xy[1], 0, 0, 90, 30] 
    joints_1 = [xy[0], xy[1], 50, 50, 90, 30]

    for _ in range(1):
        print("Start Reset Robot Arm Position, Please Wait..")
        arm.Arm_serial_servo_write6_array(joints_1, 1000)
        sleep(2)
        arm.Arm_serial_servo_write6_array(joints_0, 1000)
        sleep(2)
    print("Finish Robot Arm Position Reset!")


    # 打开摄像头
    capture = cv.VideoCapture(0)
    # 当摄像头正常打开的情况下循环执行
    while capture.isOpened():
        # 读取相机的每一帧
        ret, img = capture.read()
        print("read image from camera successfully:", ret)
        # 统一图像大小
        img = cv.resize(img, (640, 480))
        
        dp = np.fromfile(dp_cfg_path, dtype=np.int32)
        if DP_PRINT:
            print("dp dtype:", dp.dtype)
            print(dp.shape)
            print(dp)
        dp = dp.reshape(4,2)
        
        img = calibration.Perspective_transform(dp, img)
        
        img, msg = target.select_color(img, color_hsv, color_list)
        
        print("Model is warming up at stage:", warm_up_count)
        if warm_up_count != 0 and last_num == warm_up_count:
            last_count += 1
            if last_count > 5:
                warm_up_count = 0
                last_count = 0
        last_num = warm_up_count
        

        if len(msg) != 0:
            warm_up_count += 1
            if warm_up_count > 25: # REVISE
                target.target_run(msg, xy)
                warm_up_count = 0
        
            
    cv.destroyAllWindows()
    capture.release()
    
if __name__ == "__main__":
    main()