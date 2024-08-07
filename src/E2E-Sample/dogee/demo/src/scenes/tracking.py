#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from multiprocessing import Process, Queue

import numpy as np
from src.actions import SetServo, Stop, MoveForward
from src.models import YoloV5
from src.models.yolov7 import YoloV7
from src.scenes.base_scene import BaseScene
from src.utils import log
from src.utils.cv_utils import cal_iou, xyxy_to_xywh, xywh_to_xyxy, cal_inter_small
from src.utils.constant import STATE_OBSERVATION_MATRIX, STATE_TRANSITION_MATRIX, PROCESS_NOISE_COVARIANCE_MATRIX, \
    OBSERVATION_NOISE_COVARIANCE_MATRIX


class Tracking(BaseScene):
    def __init__(self, memory_name, camera_info, msg_queue):
        super().__init__(memory_name, camera_info, msg_queue)
        self.in_queue1 = Queue(1)
        self.in_queue2 = Queue(1)
        self.out_queue1 = Queue(1)
        self.out_queue2 = Queue(1)

    def init_state(self):
        log.info(f'start init {self.__class__.__name__}')
        yolov5_model_path = os.path.join(os.getcwd(), 'weights', 'yolo.om')
        yolov7_model_path = os.path.join(os.getcwd(), 'weights', 'hand_det.om')

        if not os.path.exists(yolov5_model_path) or not os.path.exists(yolov7_model_path):
            log.error(f'Cannot find the offline inference model(.om) file needed for {self.__class__.__name__}  scene.')
            return True
        lock_process = Process(target=YoloV7(yolov7_model_path).infer, args=(self.in_queue2, self.out_queue2))
        tracking_process = Process(target=YoloV5(yolov5_model_path).infer, args=(self.in_queue1, self.out_queue1))
        lock_process.start()
        tracking_process.start()
        log.info(f'{self.__class__.__name__} model init succ.')
        self.ctrl.execute(SetServo(servo=[91, 60]))
        return False

    def get_start_bbox(self, tracking_bboxes, lock_bboxes):
        """
        判断当前是否存在锁定目标，判断条件为：人体的检测框与手势的检测框的相交面积 / 手势检测框的面积 大于0.9 且手势为锁定手势
        @param tracking_bboxes: 所有的人体检测框
        @param lock_bboxes: 所有的手势检测框
        @return: 
        """
        fives = []
        for i, (x1, y1, x2, y2, cls, conf) in enumerate(lock_bboxes):
            if cls != 6:
                continue
            fives.append([x1, y1, x2, y2])

        for i, (x1, y1, x2, y2, cls, conf) in enumerate(tracking_bboxes):
            human_box = [x1, y1, x2, y2]
            for i, (xx1, yy1, xx2, yy2) in enumerate(fives):
                hand_box = [xx1, yy1, xx2, yy2]
                iou = cal_inter_small(hand_box, human_box)
                if iou > 0.9:
                    return [x1, y1, x2, y2]
        return None

    def get_count(self, target_box, lock_bboxes):
        """
        判断当前锁定目标是否解锁，判断条件为：人体的检测框与手势的检测框的相交面积 / 手势检测框的面积 大于0.9 且手势为解锁手势
        @param target_box:当前锁定目标的检测框
        @param lock_bboxes:所有的手势检测框
        @return:
        """
        for i, (x1, y1, x2, y2, cls, conf) in enumerate(lock_bboxes):
            if cls != 8:
                continue
            hand_box = [x1, y1, x2, y2]
            iou = cal_inter_small(hand_box, target_box)
            if iou > 0.9:
                return 1
        return 0

    def loop(self):
        # 初始化循环与模型
        ret = self.init_state()
        if ret:
            log.error(f'{self.__class__.__name__} init failed.')
            return
        # 初始化舵机角度
        self.ctrl.execute(SetServo(servo=[91, 60]))
        # 创建np数组，绑定共享内存
        frame = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.broadcaster.buf)
        log.info(f'{self.__class__.__name__} loop start')
        locked = False
        locked_count = 0
        unlocked_count = 0
        lost_count = 0

        iou_threshold = 0.3  # 匹配时的阈值

        x_posterior = None
        x_speed = 70
        z_speed = 40
        last_action = SetServo(servo=[91, 60])
        while True:
            action = Stop()
            if self.stop_sign.value:
                break
            if self.pause_sign.value:
                continue
            # 获取图像
            img_bgr = frame.copy()
            # 图像通过消息队列传递给模型
            self.in_queue1.put(img_bgr)
            self.in_queue2.put(img_bgr)

            # 获取模型的输出
            tracking_bbox = self.out_queue1.get()
            lock_bbox = self.out_queue2.get()

            if not locked:
                # 获取锁定目标的检测框
                start_bbox = self.get_start_bbox(tracking_bbox, lock_bbox)
                # 当前没有找到锁定目标
                if start_bbox is None:
                    locked_count = 0
                    continue
                else:
                    locked_count += 1
                    log.info(
                        f'Gesture recognition successful, locked count incremented by one. locked count:{locked_count}')

                if locked_count > 4:
                    locked = True
                    locked_count = 0
                    action = SetServo(servo=[91, 60])
                    log.info(f'locked')

                    # 将检测框转换为中心点坐标和宽高，初始化状态
                    initial_box_state = xyxy_to_xywh(start_bbox)
                    initial_state = np.array(
                        [[initial_box_state[0], initial_box_state[1], initial_box_state[2], initial_box_state[3],
                          0, 0]]).T  # [中心x,中心y,宽w,高h,dx,dy]

            if locked:
                log.info(f'======================================================')
                # 初始化卡尔曼滤波器
                if x_posterior is None:
                    x_posterior = np.array(initial_state)
                    p_posterior = np.array(np.eye(6))
                    z = np.array(initial_state)
                max_iou = iou_threshold
                max_iou_matched = False

                # 使用最大IOU来寻找观测值
                for i, (x1, y1, x2, y2, cls, conf) in enumerate(tracking_bbox):
                    xyxy = [x1, y1, x2, y2]
                    iou = cal_iou(xyxy, xywh_to_xyxy(x_posterior[0:4]))
                    if iou > max_iou:
                        target_box = xyxy
                        max_iou = iou
                        max_iou_matched = True
                if max_iou_matched:
                    log.info(f'-----------------------------------------------------')
                    # 如果找到了最大IOU BOX,则认为该框为观测值
                    xywh = xyxy_to_xywh(target_box)
                    x1, y1, x2, y2 = target_box
                    x, y, w, h = xywh
                    log.info(f'x1:{x1},y1:{y1},x2:{x2},y2:{y2}')
                    log.info(f'x:{x},y:{y},w:{w},h:{h}')
                    log.info(f'-----------------------------------------------------')

                    # 运动控制
                    while True:
                        if h >= 700 and 450 < x < 750:
                            action = Stop(servo=[91, 60])
                            break
                        if x <= 450:
                            action = MoveForward(x=x_speed, z=-z_speed, servo=[91, 60])
                        elif x >= 750:
                            action = MoveForward(x=x_speed, z=z_speed, servo=[91, 60])
                        else:
                            action = MoveForward(x=x_speed, z=0, servo=[91, 60])
                        break

                    # 计算dx,dy
                    dx = xywh[0] - x_posterior[0]
                    dy = xywh[1] - x_posterior[1]

                    z[0:4] = np.array([xywh]).T
                    z[4::] = np.array([dx, dy])
                    lost_count = 0
                else:
                    lost_count += 1

                if max_iou_matched:
                    # 进行先验估计
                    x_prior = np.dot(STATE_TRANSITION_MATRIX, x_posterior)

                    # 计算状态估计协方差矩阵P
                    p_prior = np.dot(np.dot(STATE_TRANSITION_MATRIX, p_posterior),
                                     STATE_TRANSITION_MATRIX.T) + PROCESS_NOISE_COVARIANCE_MATRIX
                    # 计算卡尔曼增益
                    k1 = np.dot(p_prior, STATE_OBSERVATION_MATRIX.T)
                    k2 = np.dot(np.dot(STATE_OBSERVATION_MATRIX, p_prior),
                                STATE_OBSERVATION_MATRIX.T) + OBSERVATION_NOISE_COVARIANCE_MATRIX
                    kalman_gain = np.dot(k1, np.linalg.inv(k2))
                    # 后验估计
                    x_posterior_1 = z - np.dot(STATE_OBSERVATION_MATRIX, x_prior)
                    x_posterior = x_prior + np.dot(kalman_gain, x_posterior_1)

                    # 更新状态估计协方差矩阵P
                    P_posterior_1 = np.eye(6) - np.dot(kalman_gain, STATE_OBSERVATION_MATRIX)
                    p_posterior = np.dot(P_posterior_1, p_prior)
                else:
                    # 如果IOU匹配失败，此时失去观测值，那么直接使用上一次的最优估计作为先验估计
                    # 此时直接迭代，不使用卡尔曼滤波
                    x_posterior = np.dot(STATE_TRANSITION_MATRIX, x_posterior)
                    action = Stop()

            if lost_count > 20:
                action = Stop()
                locked = False
                x_posterior = None
                log.info(f'target lost')

            if locked:
                log.info(f'target_box: {target_box},lock_bbox: {lock_bbox}')

                if self.get_count(target_box, lock_bbox):
                    unlocked_count += 1
                    log.info(f'Gesture recognition successful, unlocked count incremented by one. '
                             f'unlocked_count:{unlocked_count}')
                else:
                    unlocked_count = 0

                if unlocked_count > 5:
                    log.info(f'unlock')
                    locked = False
                    action = Stop()
                    x_posterior = None
                    
            # 执行动作，如果动作和上一次动作相同，则不执行
            if action.speed_setting == last_action.speed_setting:
                pass
            else:
                log.info('*' * 20)
                log.info(
                    f'action: {action}, class_x:{action.x_speed}, class_z:{action.z_speed}, '
                    f'speed:{action.speed_setting}, servo:{action.servo_angle}')
                log.info(
                    f'last_action: {last_action}, class_x:{last_action.x_speed}, class_z:{last_action.z_speed}, '
                    f'speed:{last_action.speed_setting}, servo:{last_action.servo_angle}')
                log.info('*' * 20)
                self.ctrl.execute(action)
                last_action = action
