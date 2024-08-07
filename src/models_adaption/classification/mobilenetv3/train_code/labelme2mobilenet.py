# -*- coding: utf-8 -*-
import os
from shutil import copy, rmtree
import random
import psutil
from tqdm import tqdm
import argparse
import shutil
import json
from glob import glob
import sys
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))))
sys.path.append(BASE_DIR)
from src.models_adaption.config.config import ModelConfig


def create_dir(input_path, temp_path, class_path):
    f = open(class_path, "r", encoding="utf-8")
    flags = f.read().splitlines()
    for flag in flags:
        flag = flag.strip('\n')
        os.makedirs(os.path.join(temp_path, flag), exist_ok=True)
    f.close()
    files = glob(f"{input_path}/*.json")
    for file in files:
        json_file = json.load(open(file, "r", encoding="utf-8"))
        labels = json_file["flags"]
        img = json_file['imagePath']
        if not os.path.exists(os.path.join(input_path, img)):
            raise Exception("There are pictures do not exist, please check the dataset.")
        for item in labels:
            if labels[item]:
                shutil.copy(os.path.join(input_path, img),
                            os.path.join(temp_path, item))
                break


def split_data(output_path, temp_path, split_rate, split_test):
    '''
    split_rate  : 测试集划分比例
    init_dataset: 未划分前的数据集路径
    new_dataset : 划分后的数据集路径
    '''

    def makedir(path):
        if os.path.exists(path):
            rmtree(path)
        os.makedirs(path)

    init_dataset = temp_path
    random.seed(0)

    classes_name = [name for name in os.listdir(init_dataset)]

    makedir(output_path)
    training_set = os.path.join(output_path, "train")
    val_set = os.path.join(output_path, "val")
    makedir(training_set)
    makedir(val_set)
    test_set = ''
    if split_test:
        test_set = os.path.join(output_path, "test")
        makedir(test_set)

    for cla in classes_name:
        makedir(os.path.join(training_set, cla))
        makedir(os.path.join(val_set, cla))
        if split_test:
            makedir(os.path.join(test_set, cla))

    for cla in classes_name:
        class_path = os.path.join(init_dataset, cla)
        img_set = os.listdir(class_path)
        num = len(img_set)
        if split_test:
            train_val_sample = random.sample(img_set, int(num * (1 - 0.1)))
            train_sample = random.sample(train_val_sample, int(len(train_val_sample) * (1 - split_rate)))
            with tqdm(total=num, desc=f'Class : ' + cla, mininterval=0.3) as pbar:
                for _, img in enumerate(img_set):
                    if img in train_sample:
                        init_img = os.path.join(class_path, img)
                        new_img = os.path.join(training_set, cla)
                        copy(init_img, new_img)
                    elif img in train_val_sample:
                        init_img = os.path.join(class_path, img)
                        new_img = os.path.join(val_set, cla)
                        copy(init_img, new_img)
                    else:
                        init_img = os.path.join(class_path, img)
                        new_img = os.path.join(test_set, cla)
                        copy(init_img, new_img)
                    pbar.update(1)
        else:
            train_sample = random.sample(img_set, k=int(num * (1 - split_rate)))
            with tqdm(total=num, desc=f'Class : ' + cla, mininterval=0.3) as pbar:
                for _, img in enumerate(img_set):
                    if img in train_sample:
                        init_img = os.path.join(class_path, img)
                        new_img = os.path.join(training_set, cla)
                        copy(init_img, new_img)
                    else:
                        init_img = os.path.join(class_path, img)
                        new_img = os.path.join(val_set, cla)
                        copy(init_img, new_img)
                    pbar.update(1)
    rmtree(temp_path)


# 首先确保当前文件夹下的所有图片统一后缀，如.jpg，如果为其他后缀，将suffix改为对应的后缀，如.png
def ChangeToMobilenet(input_path, class_path, output_path, split_ratio, split_test):
    # 校验
    if psutil.cpu_percent(interval=1) > 72:
        raise Exception("The CPU usage is too high. Please try again later.")
    if not os.path.exists(input_path):
        return "ERROR: not found input_path."
    if os.path.exists(output_path):
        return "ERROR: output_path already exits."
    if split_ratio <= 0 or split_ratio >= 1:
        return 'ERROR: split_ratio out of range (0,1).'
    temp_path = f"{os.path.dirname(os.path.abspath(output_path))}/temp"
    externs = ['png', 'jpg', 'JPEG', 'BMP', 'bmp', 'webp']
    files = list()
    for extern in externs:
        path = os.path.join(input_path, '*.' + extern)
        files.extend(glob(path))
    create_dir(input_path, temp_path, class_path)
    if len(files) < 20:
        raise Exception("The dataset needs at least 20 pictures")
    if len(files) == 0:
        raise Exception("Invalid dataset path.")
    split_data(output_path, temp_path, split_ratio, split_test)
    print("process_value=5")
    return "INFO:generate mobilenet dataset success!"


def parse_args():
    parser = argparse.ArgumentParser("Transfer labelme dataset into mobilenet format.")
    parser.add_argument("--input_path", required=True, type=str, help="Input labelme dataset abs path.",
                        default="")
    parser.add_argument("--class_txt", required=True, type=str, help="Input class txt abs path.",
                        default="")
    parser.add_argument("--output_path", required=True, type=str, help="Output mobilenet format path.",
                        default="./output")
    parser.add_argument("--split_ratio", required=False, type=float, default=0.2,
                        help="Dataset split ratio, both used in val and test dataset.")
    parser.add_argument("--split_test", type=str, default="False",
                        help="Choose whether to generate test dataset, true or false.")

    parser.set_defaults()
    return parser.parse_args()


if __name__ == "__main__":
    # args = parse_args()
    # args.split_test = bool(args.split_test == "True")
    yaml_file = '../../../config/config.yaml'
    config = ModelConfig(yaml_file)
    yaml_data = config.get_yaml_data().get('classification')
    input_path = yaml_data['labelme_dataset_path']
    class_txt = yaml_data['class_txt_path']
    trans_output_path = os.path.join(yaml_data['output_path'], 'trans_output')
    split_ratio = yaml_data['split_ratio']
    split_test = yaml_data['split_test']
    config.set_para('classification', 'trans_output', trans_output_path)
    config.yaml_dump(yaml_file)
    try:
        ret = ChangeToMobilenet(input_path, class_txt, trans_output_path, split_ratio, split_test)
        print(ret)
    except Exception as e:
        # delete temp
        if os.path.isdir(f"{os.path.dirname(os.path.abspath(trans_output_path))}/temp"):
            rmtree(f"{os.path.dirname(os.path.abspath(trans_output_path))}/temp")
        # delete output
        print(f"ERROR:{e}")
