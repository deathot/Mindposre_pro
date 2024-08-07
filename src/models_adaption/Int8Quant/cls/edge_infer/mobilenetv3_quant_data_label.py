from modelslim.onnx.post_training_quant import QuantConfig, run_quantize
from torchvision import transforms
from PIL import Image
import os
import numpy as np

from modelslim import set_logger_level
set_logger_level("info")        #根据实际情况配置

FILE = os.path.realpath(__file__)
infer_folder = os.path.dirname(FILE)
img_folder = os.path.join(os.path.join(infer_folder, "data"), "dog")
input_model_path = os.path.join(infer_folder, "MobileNetV3_large.onnx")
output_model_path = os.path.join(infer_folder, "MobileNetV3_large_quant.onnx")

def custom_read_data():
    calib_data = []
    
    img_list = []
    for path in os.listdir(img_folder):
        pic_path = os.path.join(img_folder, path)
        input = Image.open(pic_path)
        preprocess = transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        input_tesnor = preprocess(input)
        img_list.append(input_tesnor)
    calib_data.append(np.stack(img_list))
    print(calib_data[0].shape)
    return calib_data

calib_data = custom_read_data()  

quant_config = QuantConfig(calib_data=calib_data, amp_num=5)

run_quantize(input_model_path, output_model_path, quant_config)