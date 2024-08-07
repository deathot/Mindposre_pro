# -*- coding: utf-8 -*-
import torch
from geffnet.mobilenetv3 import mobilenetv3_large_100

import argparse
import os
import json

from torchvision import transforms
from PIL import Image


def parse_args():
    parser = argparse.ArgumentParser("Model infer.")
    parser.add_argument("--weights", required=True, type=str, help="Input train output '.pt' abs file path.",
                        default=r"./MobileNetV3.pth")
    parser.add_argument("--data", required=True, type=str, help="Infer data path with images.",
                        default=r"./data")
    parser.add_argument("--output_path", required=True, type=str, help="cls_output.txt path.",
                        default=r"./")
    parser.add_argument("--label_path", required=True, type=str, help="label json file path.")

    parser.set_defaults()
    return parser.parse_args()


def main(weights, data, label_path, output_path):
    # check
    if not os.path.isfile(weights):
        return 'ERROR: not found weights file.'
    if not os.path.isdir(data):
        return 'ERROR: Input data is not a valid direction.'
    if os.path.isfile(f'{output_path}/cls_output.txt'):
        return 'ERROR: output file already exist.'
    if not os.path.isfile(label_path):
        return "ERROR: not found label_path 'class_indices.json'."
    if os.path.exists(output_path):
        return 'ERROR: output_path already exists.'
    os.makedirs(output_path)
    label_dict = {}
    with open(label_path, 'r') as f:
        label_dict = json.load(f)
    num_classes = len(label_dict)
    net = mobilenetv3_large_100(pretrained=False, num_classes=num_classes)
    pre_weights = torch.load(weights)
    net.load_state_dict(pre_weights)
    net.eval()
    with open(f'{output_path}/cls_output.txt', 'w') as f:
        for pic in os.listdir(data):
            pic_path = os.path.join(data, pic)
            pic_name = pic.split('.')[0]

            # preprocess
            input = Image.open(pic_path)
            preprocess = transforms.Compose([
                transforms.Resize(256),
                transforms.CenterCrop(224),
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
            ])
            input_tesnor = preprocess(input)
            input_batch = input_tesnor.unsqueeze(0)
            with torch.no_grad():
                output = net(input_batch)

            torch.nn.functional.softmax(output[0], dim=0)
            _, preds = torch.max(output, 1)

            infer_result = label_dict.get(str(preds.numpy().tolist()[0]))

            print(f"image name :{pic}, infer result: {infer_result}")
            f.write(f"image name :{pic}, infer result: {infer_result}\n")
        return 'INFO: Infer success!'


if __name__ == '__main__':
    args = parse_args()
    try:
        ret = main(args.weights, args.data, args.label_path, args.output_path)
        print(ret)
    except Exception as e:
        print(e)
