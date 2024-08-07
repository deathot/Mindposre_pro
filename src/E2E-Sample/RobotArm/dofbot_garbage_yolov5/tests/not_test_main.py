import pytest
import Arm_Lib
import cv2 as cv
from time import sleep
from dofbot_garbage_yolov5.utils.dofbot_config import Arm_Calibration, read_XYT
from dofbot_garbage_yolov5.utils.garbage_identify import garbage_identify
from pathlib import Path
import os
import numpy as np
from unittest.mock import MagicMock, patch

def test_main(monkeypatch):
    # Mock the garbage_identify class
    garbage_identify_mock = MagicMock(spec=garbage_identify)
    garbage_identify_mock.garbage_run.return_value = (np.zeros((480, 640, 3), dtype=np.uint8), {})
    garbage_identify_mock.garbage_grap.return_value = None
    monkeypatch.setattr('main.garbage_identify', garbage_identify_mock)

    # Mock the Arm_Calibration class
    Arm_Calibration_mock = MagicMock(spec=Arm_Calibration)
    Arm_Calibration_mock.Perspective_transform.return_value = np.zeros((480, 640, 3), dtype=np.uint8)
    monkeypatch.setattr('main.Arm_Calibration', Arm_Calibration_mock)

    # Mock the read_XYT function
    read_XYT_mock = MagicMock(spec=read_XYT)
    read_XYT_mock.return_value = ([90, 135], 0)
    monkeypatch.setattr('main.read_XYT', read_XYT_mock)

    # Mock the Arm_Device class
    Arm_Device_mock = MagicMock(spec=Arm_Lib.Arm_Device)
    Arm_Device_mock.Arm_serial_servo_write6_array.return_value = None
    monkeypatch.setattr('main.Arm_Device', Arm_Device_mock)

    # Mock the cv.VideoCapture class
    VideoCapture_mock = MagicMock(spec=cv.VideoCapture)
    VideoCapture_mock.return_value.isOpened.return_value = True
    VideoCapture_mock.return_value.read.return_value = (True, np.zeros((480, 640, 3), dtype=np.uint8))
    monkeypatch.setattr('main.cv.VideoCapture', VideoCapture_mock)

    # Call the main function
    main()

    # Check that the garbage_identify class was called
    assert garbage_identify_mock.garbage_run.call_count > 0
    assert garbage_identify_mock.garbage_grap.call_count > 0

    # Check that the Arm_Calibration class was called
    assert Arm_Calibration_mock.Perspective_transform.call_count > 0

    # Check that the read_XYT function was called
    assert read_XYT_mock.call_count > 0

    # Check that the Arm_Device class was called
    assert Arm_Device_mock.Arm_serial_servo_write6_array.call_count > 0

    # Check that the cv.VideoCapture class was called
    assert VideoCapture_mock.call_count > 0
    assert VideoCapture_mock.return_value.isOpened.call_count > 0
    assert VideoCapture_mock.return_value.read.call_count > 0