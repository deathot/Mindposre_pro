from modelslim.onnx.post_training_quant import QuantConfig, run_quantize
import cv2
import numpy as np
# import torch
from albumentations.augmentations import transforms
import os
import argparse

from modelslim import set_logger_level
set_logger_level("info") #根据实际情况配置

FILE = os.path.realpath(__file__)
infer_folder = os.path.dirname(FILE)
input_model_path = os.path.join(infer_folder, "model.onnx")
output_model_path = os.path.join(infer_folder, "model_quant.onnx")

def parse_args():
    parser = argparse.ArgumentParser("Quantize Unet++.")
    parser.add_argument("--img_path", required=True, type=str, help="Image for data calibration")
    return parser.parse_args()

def custom_read_data():

    img_bgr = cv2.imread(img_path)
    
    image = cv2.resize(img_bgr, (96, 96))
    nor = transforms.Normalize()
    image = nor.apply(image)
    image = image.astype('float32') / 255
    image = image.transpose(2, 0, 1)
    
    image = np.expand_dims(image, axis=0)
    
    calib_data = []
    calib_data.append(image)
    return calib_data


args = parse_args()
img_path = args.img_path

calib_data = custom_read_data()

quant_config = QuantConfig(calib_data=calib_data, amp_num=5)

run_quantize(input_model_path, output_model_path, quant_config)
