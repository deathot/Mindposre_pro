import numpy as np
import cv2
import torch
from robot_arm_color_stacking.utils.stacking_target import stacking_GetTarget
from pathlib import Path
import os

class TestStackingGrap:
    
    def setup_method(self):
        FILE = Path(__file__).resolve()
        lib_root = os.path.dirname(FILE.parents[0]) 
        test_dir = os.path.join(lib_root, "robot_arm_color_stacking", "test_imgs")
        self.img_bgr = cv2.imread(os.path.join(test_dir, "2.jpg"))
        self.st_obj = stacking_GetTarget(test_mode=True)
    
    # 测试小index时，select_color函数的输出类型
    def test_select_color_low_garbage_index(self):
        _, msg = self.st_obj.select_color(self.img_bgr, 1, 1)
        assert isinstance(msg, dict)
        
    # 测试大index时，select_color函数的输出类型
    def test_select_color_high_garbage_index(self):
        _, msg = self.st_obj.select_color(self.img_bgr, 1, 1, 5) # {'Fish_bone0': (0.04875, 0.25267), 'Syringe1': (-0.03575, 0.27107)}
        assert isinstance(msg, dict)
        
    # 测试target_run函数运行时，机械臂的抓取情况
    def test_target_run(self): # 需要后台开启ros server
        msg = {'Syringe2': (0.04875, 0.25267), 'Syringe1': (-0.03575, 0.27107)}
        ret = self.st_obj.target_run(msg)
        assert ret is None