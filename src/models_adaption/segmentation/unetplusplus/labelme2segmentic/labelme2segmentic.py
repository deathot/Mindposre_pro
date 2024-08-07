import os
import sys
import json
import shutil
import argparse
from glob import glob

import psutil

BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))))
sys.path.append(BASE_DIR)

import numpy as np
import cv2
from sklearn.model_selection import train_test_split

from src.models_adaption.config.config import ModelConfig
from labelme2voc import convert_label_to_mask


img_size = 96


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--labelme_anno_path', default='C:/Users/TDTECH/Desktop/tmp/camera_data_2class', type=str)
    parser.add_argument('--out_dir', default='C:/Users/TDTECH/Desktop/tmp/trans_out', type=str)
    parser.add_argument('--split_ratio', default=0.2, type=float)
    parser.add_argument('--split_test', default=True, type=bool)
    args = parser.parse_args()
    return args


def check_dir(dir):
    if not os.path.exists(dir):
        os.makedirs(dir, exist_ok=True)


def create_json_for_each_class(in_dir, out_dir, label_dict):
    ''' 将labelme的多分类标注, 分离为单分类标注, 在 out_dir 中建立各个分类的文件夹'''

    # 得到 img_names 和 json_names 列表
    img_names = []
    json_files = glob(f"{in_dir}/*.json")
    for _file in json_files:
        json_file = json.load(open(_file, "r", encoding="utf-8"))
        img_name = json_file['imagePath']
        img_names.append(img_name)
    json_names = [i.split('.')[0]+'.json' for i in img_names]

    for (i, class_name) in label_dict.items():
        # 构造输出文件夹
        save_dir = os.path.join(out_dir, str(i)+'_'+class_name)
        check_dir(save_dir)

        # 复制图片
        for img_name in img_names:
            shutil.copy(os.path.join(in_dir, img_name), save_dir)

        # 得到本类json
        for json_name in json_names:
            # 读入原来json
            json_file = json.load(
                open(os.path.join(in_dir, json_name), "r", encoding="utf-8"))

            # 删除非本类 json
            idx_to_delete = []
            for j, one_seg_area in enumerate(json_file['shapes']):
                if one_seg_area['label'] != class_name:
                    idx_to_delete.append(j)
            for idx in reversed(idx_to_delete):
                json_file['shapes'].pop(idx)

            # 保存到本类json文件
            new_json = json.dumps(json_file)
            f = open(os.path.join(save_dir, json_name), 'w')
            f.write(new_json)
            f.close()


def get_mask_dirs(mask_dir, num_class):
    re = [os.path.join(mask_dir, str(i)) for i in range(num_class)]
    for i in re:
        check_dir(i)
    return re


def get_data_subdirectory(root_dir, tr_or_val, num_class):
    # 在 root_dir 下，建立一个名为 tr_or_val 的文件夹，其中包含 images 和 masks 文件夹，其目录层级如下：
    # |-- root_dir
    #     |-- tr_or_val
    #     |   |-- images
    #     |   |   |-- file1667306504837.png
    #     |   |   |-- ...
    #     |   |-- masks
    #     |       |-- 0
    #     |           |-- file1667306504837.png
    #     |           |-- ...
    #     |       |-- ...
    #     |           |-- file1667306504837.png
    #     |           |-- ...
    #     |       |-- num_class-1
    #     |           |-- file1667306504837.png
    #     |           |-- ...

    sub_dir = os.path.join(root_dir, tr_or_val)
    sub_img_dir = os.path.join(sub_dir, 'images')
    check_dir(sub_dir)
    check_dir(sub_img_dir)
    sub_msk_dirs = get_mask_dirs(os.path.join(sub_dir, 'masks'), num_class)
    return sub_img_dir, sub_msk_dirs


def main(args):
    if psutil.cpu_percent(interval=1) > 72:
        raise Exception("The CPU usage is too high. Please try again later.")
    # get files
    img_paths = []
    img_names = []
    extens = []
    labels = []  # 用于查看有几个类别
    json_files = glob(f"{args.labelme_anno_path}/*.json")
    if len(json_files) < 20:
        raise Exception("The dataset needs at least 20 pictures")
    if len(json_files) == 0:
        raise Exception("Invalid dataset path.")

    for _file in json_files:
        json_file = json.load(open(_file, "r", encoding="utf-8"))

        imagename = json_file['imagePath']
        if not os.path.exists(os.path.join(args.labelme_anno_path, imagename)):
            raise Exception("There are pictures do not exist, please check the dataset.")
        for single_label in json_file['shapes']:
            labels.append(single_label['label'])

        img_paths.append(os.path.join(args.labelme_anno_path, imagename))
        img_names.append(imagename)
        extens.append(os.path.splitext(imagename)[1])

    if len(set(extens)) > 1:
        print("数据集图片只能有一种类型，例如只有 'jpg'，只有'png' 等")
        exit(-1)

    extend = list(set(extens))[0]
    classes = list(set(labels))
    num_class = len(classes)

    label_dict = {}  # example: {0: 'cat', 1: 'dog'}
    for i, c in enumerate(classes):
        label_dict[i] = c

    # split total to tr, val and test
    trval_img_names = img_names
    test_img_names = None
    if args.split_test:
        trval_img_names, test_img_names = train_test_split(trval_img_names, test_size=0.1, random_state=41)
    tr_img_names, val_img_names = train_test_split(trval_img_names, test_size=args.split_ratio, random_state=41)
    tr_ids = [i.split('.')[0] for i in tr_img_names]
    val_ids = [i.split('.')[0] for i in val_img_names]
    if args.split_test:
        test_ids = [i.split('.')[0] for i in test_img_names]

    # get tr val test dirs
    tr_img_dir, tr_msk_dirs = get_data_subdirectory(
        args.out_dir, 'train', num_class)
    val_img_dir, val_msk_dirs = get_data_subdirectory(
        args.out_dir, 'val', num_class)
    if args.split_test:
        test_img_dir, test_msk_dirs = get_data_subdirectory(
            args.out_dir, 'test', num_class)

    # copy 图片到 train val test 的 images
    for i, img_path in enumerate(img_paths):
        if img_names[i] in tr_img_names:
            shutil.copy(img_path, tr_img_dir)
        elif img_names[i] in val_img_names:
            shutil.copy(img_path, val_img_dir)
        if args.split_test:
            if img_names[i] in test_img_names:
                shutil.copy(img_path, test_img_dir)

    # 构造输出文件夹
    separated_dir = os.path.join(args.out_dir, 'separated_dataset')
    check_dir(separated_dir)
    create_json_for_each_class(
        args.labelme_anno_path, separated_dir, label_dict)

    for i in range(num_class):
        class_name = label_dict[i]

        # 得到单分类标注文件夹
        dataset_dir = os.path.join(separated_dir, str(i)+'_'+class_name)

        # save files temporaly
        seg_tmp_dir = os.path.join(args.out_dir, 'seg_tmp')
        if os.path.exists(seg_tmp_dir):
            shutil.rmtree(seg_tmp_dir)

        # 构造本类 label txt
        label_txt_i = os.path.join(dataset_dir, 'label.txt')
        f = open(label_txt_i, mode='w')
        f.writelines(['__ignore__\n', '_background_\n'])
        f.write(class_name)
        f.close()

        # convert Polygon annotation to semantic segmentation annotation
        # save to seg_tmp_dir/SegmentationClass/*.npy
        convert_label_to_mask(dataset_dir, seg_tmp_dir, label_txt_i, extend)

        # copy 本类 masks
        for imagepath in img_paths:
            # copy images
            file_id = os.path.basename(imagepath).split('.')[0]
            mask = np.load(os.path.join(
                seg_tmp_dir, 'SegmentationClass', file_id+'.npy'))
            mask = cv2.resize(mask.astype('float'), (img_size, img_size))

            if file_id in tr_ids:
                cv2.imwrite(os.path.join(
                    tr_msk_dirs[i], file_id+'.png'), (mask * 255).astype('uint8'))
            elif file_id in val_ids:
                cv2.imwrite(os.path.join(
                    val_msk_dirs[i], file_id+'.png'), (mask * 255).astype('uint8'))
            if args.split_test:
                if file_id in test_ids:
                    cv2.imwrite(os.path.join(
                        test_msk_dirs[i], file_id+'.png'), (mask * 255).astype('uint8'))

        if os.path.exists(seg_tmp_dir):
            shutil.rmtree(seg_tmp_dir)

    if os.path.exists(separated_dir):
        shutil.rmtree(separated_dir)

    print("process_value=5")
    return 'INFO:create dataset success!'


if __name__ == '__main__':
    args = parse_args()
    yaml_file = '../../../config/config.yaml'
    config = ModelConfig(yaml_file)
    yaml_data = config.get_yaml_data().get('segmentation')
    args.labelme_anno_path = yaml_data['labelme_dataset_path']
    trans_output_path = os.path.join(yaml_data['output_path'], 'trans_output')
    args.split_ratio = yaml_data['split_ratio']
    args.out_dir = trans_output_path
    config.set_para('segmentation', 'trans_output', trans_output_path)
    config.yaml_dump(yaml_file)

    try:
        ret = main(args)
        print(ret)
    except Exception as e:
        print(f"ERROR:{e}")
