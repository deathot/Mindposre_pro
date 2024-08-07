# -*- coding: utf-8 -*-
import os
import argparse
import shutil
import json
import numpy as np
from glob import glob
import cv2
import psutil
import yaml
from sklearn.model_selection import train_test_split
import sys
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))))
sys.path.append(BASE_DIR)
from src.models_adaption.config.config import ModelConfig
from yolov5_dataset_2_coco import yolov5_2_coco_annotation

IMG_SUFFIX_TRANS = ".jpg"


def change_image_format(input_path, output_path):
    """
    统一当前文件夹下所有图像的格式，如'.jpg'
    :param input_path:当前文件路径
    :return:
    """
    externs = ['png', 'jpg', 'JPEG', 'BMP', 'bmp', 'webp']
    files = list()
    for extern in externs:
        path = os.path.join(input_path, '*.' + extern)
        files.extend(glob(path))
    json_dirs = glob(f'{input_path}/*.json')
    for _file in json_dirs:
        json_file = json.load(open(_file, "r", encoding="utf-8"))
        imagename = json_file['imagePath']
        if not os.path.exists(os.path.join(input_path, imagename)):
            raise Exception("There are pictures do not exist, please check the dataset.")
    index = 0
    if len(files) < 20:
        raise Exception("The dataset needs at least 20 pictures")
    if len(files) == 0:
        raise Exception("Invalid dataset path.")
    for file in files:
        name = ''.join(file.split('.')[:-1])
        json_file = os.path.join(input_path, name + '.json')
        if not os.path.exists(json_file):
            continue
        index = index + 1
        new_json_file = "%08d.json" % index
        new_json_path = os.path.join(output_path, new_json_file)
        shutil.copy(json_file, new_json_path)
        new_img_file = "%08d.jpg" % index
        image = cv2.imread(file)
        new_file_path = os.path.join(output_path, new_img_file)
        cv2.imwrite(new_file_path, image)


def get_all_class(file_list, input_path):
    """
    从json文件中获取当前数据的所有类别
    :param file_list:当前路径下的所有文件名
    :param input_path:当前文件路径
    :return:
    """
    classes = list()
    for filename in file_list:
        json_path = os.path.join(input_path, filename + '.json')
        json_file = json.load(open(json_path, "r", encoding="utf-8"))
        for item in json_file["shapes"]:
            label_class = item['label']
            if label_class not in classes:
                classes.append(label_class)
    classes.sort()
    return classes


def split_dataset(input_path, split_ratio, split_test):
    """
    将文件分为训练集，测试集和验证集
    :param split_ratio: 分割测试集或验证集的比例
    :param split_test: 是否使用测试集，默认为False
    :param input_path:当前文件路径
    :return:
    """
    files = glob(os.path.join(input_path, '*.json'))
    files = [os.path.basename(filename).split(".json")[0] for filename in files]

    test_files = None
    if split_test:
        trainval_files, test_files = train_test_split(files, test_size=0.1, random_state=55)
        print(f'Test set size: {len(test_files)}')
    else:
        trainval_files = files
    train_files, val_files = train_test_split(trainval_files, test_size=split_ratio, random_state=55)
    print(f'Training set size: {len(train_files)}')
    print(f'Validation set size: {len(val_files)}')
    return train_files, val_files, test_files, files


def create_save_dir(output_path, split_test):
    """
    按照训练时的图像和标注路径创建文件夹
    :param output_path:输出文件路径
    :param split_test: 是否有测试集
    :return:
    """
    # 生成训练集
    train_image = os.path.join(output_path, 'train', 'images')
    if not os.path.exists(train_image):
        os.makedirs(train_image)
    train_label = os.path.join(output_path, 'train', 'labels')
    if not os.path.exists(train_label):
        os.makedirs(train_label)
    # 生成验证集
    val_image = os.path.join(output_path, 'valid', 'images')
    if not os.path.exists(val_image):
        os.makedirs(val_image)
    val_label = os.path.join(output_path, 'valid', 'labels')
    if not os.path.exists(val_label):
        os.makedirs(val_label)
    # 生成测试集
    if split_test:
        test_image = os.path.join(output_path, 'test', 'images')
        if not os.path.exists(test_image):
            os.makedirs(test_image)
        test_label = os.path.join(output_path, 'test', 'labels')
        if not os.path.exists(test_label):
            os.makedirs(test_label)
        return train_image, train_label, val_image, val_label, test_image, test_label
    else:
        return train_image, train_label, val_image, val_label, '', ''


def convert(size, box):
    dw = 1. / (size[0])
    dh = 1. / (size[1])
    x = (box[0] + box[1]) / 2.0 - 1
    y = (box[2] + box[3]) / 2.0 - 1
    w = box[1] - box[0]
    h = box[3] - box[2]
    x = x * dw
    w = w * dw
    y = y * dh
    h = h * dh
    return x, y, w, h


def push_into_file(file, images, labels, input_path, output_path):
    """
    最终生成在当前文件夹下的所有文件按image和label分别存在到训练集/验证集/测试集路径的文件夹下
    :param file: 文件名列表
    :param images: 存放images的路径
    :param labels: 存放labels的路径
    :param input_path: 输入文件路径
    :param output_path: 输出文件路径
    :return:
    """

    for filename in file:
        image_file = os.path.join(input_path, filename + IMG_SUFFIX_TRANS)
        label_file = os.path.join(output_path, f'{filename}.txt')
        if not os.path.exists(os.path.join(images, filename + IMG_SUFFIX_TRANS)):
            try:
                shutil.copy(image_file, images)
            except OSError:
                pass
        if not os.path.exists(os.path.join(labels, f'{filename}.txt')):
            try:
                shutil.move(label_file, labels)
            except OSError:
                pass


def json2txt(classes, input_path, output_path, files):
    """
    将json文件转化为txt文件,存到输出文件夹
    :param classes: 类别名
    :param input_path:当前文件路径
    :param output_path:输出文件路径
    :param files:文件名
    :return:
    """
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    for json_file in files:
        json_filename = os.path.join(input_path, json_file + ".json")
        imagePath = os.path.join(input_path, json_file + IMG_SUFFIX_TRANS)
        out_file = open(f"{output_path}/{json_file}.txt", 'w')
        json_file_ = json.load(open(json_filename, "r", encoding="utf-8"))
        if os.path.exists(imagePath):
            height, width, channels = cv2.imread(imagePath).shape
            for multi in json_file_["shapes"]:
                points = np.array(multi["points"])
                xmin = min(points[:, 0]) if min(points[:, 0]) > 0 else 0
                xmax = max(points[:, 0]) if max(points[:, 0]) > 0 else 0
                ymin = min(points[:, 1]) if min(points[:, 1]) > 0 else 0
                ymax = max(points[:, 1]) if max(points[:, 1]) > 0 else 0
                label = multi["label"]
                if xmax <= xmin:
                    pass
                elif ymax <= ymin:
                    pass
                else:
                    cls_id = classes.index(label)
                    b = (float(xmin), float(xmax), float(ymin), float(ymax))
                    bb = convert((width, height), b)
                    out_file.write(str(cls_id) + " " + " ".join([str(a) for a in bb]) + '\n')


def create_yaml(classes, output_path, split_test):
    nc = len(classes)
    key = list(range(nc))
    classes_dict = dict(zip(key, classes))

    if not split_test:
        desired_caps = {
            'train': os.path.join(os.path.abspath(output_path), 'train', 'images'),
            'val': os.path.join(os.path.abspath(output_path), 'valid', 'images'),
            'nc': nc,
            'names': classes_dict
        }
    else:
        desired_caps = {
            'train': os.path.join(os.path.abspath(output_path), 'train', 'images'),
            'val': os.path.join(os.path.abspath(output_path), 'valid', 'images'),
            'test': os.path.join(os.path.abspath(output_path), 'test', 'images'),
            'nc': nc,
            'names': classes_dict
        }
    yamlpath = os.path.join(output_path, 'data.yaml')

    # 写入到yaml文件
    with open(yamlpath, "w+", encoding="utf-8") as f:
        for key, val in desired_caps.items():
            yaml.dump({key: val}, f, default_flow_style=False)

    return yamlpath, nc


# 首先确保当前文件夹下的所有图片统一后缀，如.jpg，如果为其他后缀，将suffix改为对应的后缀，如.png
def ChangeToYolo5(input_path, output_path, split_ratio, split_test):
    """
    生成最终标准格式的文件
    :param split_ratio: 分割测试集或验证集的比例
    :param input_path:当前文件路径
    :param output_path:输出文件路径
    :param split_test: 是否使用测试集
    :return:
    """
    if psutil.cpu_percent(interval=1) > 72:
        raise Exception("The CPU usage is too high. Please try again later.")
    if not os.path.exists(input_path):
        raise Exception('ERROR:input_path not exist.')
    if split_ratio <= 0 or split_ratio >= 1:
        raise Exception('ERROR:split_ratio out of range (0,1).')
    labelme_dataset_path = os.path.join(output_path, 'labelme')
    if not os.path.exists(labelme_dataset_path):
        os.makedirs(labelme_dataset_path)
    change_image_format(input_path, labelme_dataset_path)

    image_path = os.path.join(labelme_dataset_path, '*.jpg')
    json_path = os.path.join(labelme_dataset_path, '*.json')
    img_dirs = glob(image_path)
    json_dirs = glob(json_path)
    if len(img_dirs) != len(json_dirs):
        raise Exception('ERROR:Unsupported picture type.')
    train_files, val_files, test_file, files = split_dataset(labelme_dataset_path, split_ratio, split_test)
    classes = get_all_class(files, labelme_dataset_path)
    json2txt(classes, labelme_dataset_path, output_path, files)
    yaml_file_path, nc = create_yaml(classes, output_path, split_test)
    train_image, train_label, val_image, val_label, test_image, test_label = create_save_dir(output_path, split_test)
    push_into_file(train_files, train_image, train_label, labelme_dataset_path, output_path)
    push_into_file(val_files, val_image, val_label, labelme_dataset_path, output_path)
    if split_test:
        push_into_file(test_file, test_image, test_label, labelme_dataset_path, output_path)
        coco_format_annotation_file = os.path.join(output_path, 'test', 'test.json')
        yolo_dataset_yaml_path = os.path.join(output_path, 'data.yaml')
        yolo_annotation_path = os.path.join(output_path, 'test', 'labels')
        yolo_img_path = os.path.join(output_path, 'test', 'images')
        yolov5_2_coco_annotation(coco_format_annotation_file, yolo_dataset_yaml_path,
                                 yolo_img_path, yolo_annotation_path)
    print(f"process_value=5")
    print('INFO:create dataset success!')
    return yaml_file_path, nc


def parse_args():
    parser = argparse.ArgumentParser("Transfer labelme dataset into yolov5 format.")
    parser.add_argument("--input_path", required=True, type=str, help="Input labelme dataset abs path.",
                        default=r"./input_images")
    parser.add_argument("--output_path", required=False, type=str, help="Output yolov5 format path.", default="./output")
    parser.add_argument("--split_ratio", required=False, type=str, default="0.3",
                        help="Dataset split ratio, both used in val and test dataset.")
    parser.add_argument("--split_test", type=str, default="False",
                        help="Choose whether to generate test dataset, true or false.")

    parser.set_defaults()
    return parser.parse_args()


if __name__ == "__main__":
    # args = parse_args()
    # args.split_ratio = float(args.split_ratio)
    # args.split_test = bool(args.split_test == "True")
    yaml_file = '../../../../config/config.yaml'
    config = ModelConfig(yaml_file)
    yaml_data = config.get_yaml_data().get('detection')
    input_path = yaml_data['labelme_dataset_path']
    trans_output_path = os.path.join(yaml_data['output_path'], 'trans_output')
    split_ratio = yaml_data['split_ratio']
    split_test = yaml_data['split_test']
    try:
        yolo_dataset_yaml_path, class_num = ChangeToYolo5(input_path, trans_output_path, split_ratio, split_test)
        config.set_para('detection', 'yolo_dataset_yaml_path', yolo_dataset_yaml_path)
        config.set_para('detection', 'class_num', class_num)
        config.set_para('detection', 'trans_output', trans_output_path)
        config.yaml_dump(yaml_file)
    except Exception as e:
        print(f"ERROR:{e}")
