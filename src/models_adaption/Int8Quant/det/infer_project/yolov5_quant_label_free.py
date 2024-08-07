from modelslim.onnx.post_training_quant import QuantConfig, run_quantize
import cv2
import numpy as np
import os
import argparse

from modelslim import set_logger_level
set_logger_level("info") #根据实际情况配置

FILE = os.path.realpath(__file__)
infer_folder = os.path.dirname(FILE)
input_model_path = os.path.join(infer_folder, "yolov5s.onnx")
output_model_path = os.path.join(os.path.join(infer_folder, "output"), "yolov5s_quant.onnx")

def parse_args():
    parser = argparse.ArgumentParser("Quantize Yolov5.")
    parser.add_argument("--img_path", required=True, type=str, help="Image for data calibration")
    return parser.parse_args()

def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=False, scaleFill=False, scaleup=True):
    # Resize image to a 32-pixel-multiple rectangle https://github.com/ultralytics/yolov3/issues/232
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, 64), np.mod(dh, 64)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return img, ratio, (dw, dh)

def custom_read_data():

    img_bgr = cv2.imread(img_path)
    img_padding, scale_ratio, pad_size = letterbox(img_bgr, new_shape=(640, 640))  # padding resize bgr
    
    img = []

    img.append(img_padding)
    img = np.stack(img, axis=0)
    img = img[..., ::-1].transpose(0, 3, 1, 2)  # BGR tp RGB
    image_np = np.array(img, dtype=np.float32)
    image_np_expanded = image_np / 255.0
    img = np.ascontiguousarray(image_np_expanded).astype(np.float16)    
    
    calib_data = []
    calib_data.append(img)
    return calib_data

args = parse_args()
img_path = args.img_path
calib_data = custom_read_data()
quant_config = QuantConfig(calib_data=calib_data, amp_num=5)
run_quantize(input_model_path, output_model_path, quant_config)
