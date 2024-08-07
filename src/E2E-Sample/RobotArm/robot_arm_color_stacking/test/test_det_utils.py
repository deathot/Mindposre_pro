import pytest
import cv2
import numpy as np
from robot_arm_color_stacking.utils.det_utils import letterbox
from pathlib import Path
import os

class TestDETUtils:
    
    def setup_method(self):
        self.cfg = {
            'conf_thres': 0.7,
            'iou_thres': 0.7,
            'input_shape': [640, 640],
        }
        
        FILE = Path(__file__).resolve()
        pkg_root_dir = os.path.dirname(FILE.parents[0])
        model_dir = os.path.join(pkg_root_dir, "robot_arm_color_stacking", "model")
        model_path = os.path.join(model_dir, "yolov5s_bs1.om")
        test_dir = os.path.join(pkg_root_dir, "robot_arm_color_stacking", "test_imgs")
        self.label_path = os.path.join(model_dir, "coco_names.txt")
        self.img_bgr = cv2.imread(os.path.join(test_dir, "2.jpg"))
        
    # 测试letterbox函数的输出类型
    def test_letter_box(self):
        img, ratio, pad_size = letterbox(self.img_bgr, 640)
        assert isinstance(img, np.ndarray)
        assert isinstance(ratio, tuple)
        assert isinstance(pad_size, tuple)