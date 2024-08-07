# -*- coding: utf-8 -*-
import sys
import os
import glob
import cv2
import argparse
import numpy as np
import json
# ---below---imgaug module
import imgaug as ia
import imgaug.augmenters as iaa
from imgaug.augmentables import Keypoint, KeypointsOnImage

from labelme import utils

'''
ticks:
1) picture type : jpg;
2) while augumenting, mask not to go out image shape;
3) maybe some error because data type not correct.
'''


def mkdir(path):
    isExists = os.path.exists(path)
    if not isExists:
        os.mkdir(path)
        print('====================')
        print('creat path : ', path)
        print('====================')
    return 0


def check_json_file(path):
    for i in path:
        json_path = i[:-3] + 'json'
        if not os.path.exists(json_path):
            print('error')
            print(json_path, ' not exist !!!')
            sys.exit(1)


def read_jsonfile(path):
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def save_jsonfile(object, save_path):
    json.dump(object, open(save_path, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)


def get_points_from_json(json_file):
    point_list = []
    shapes = json_file['shapes']
    for i in range(len(shapes)):
        for j in range(len(shapes[i]["points"])):
            point_list.append(shapes[i]["points"][j])
    return point_list


def write_points_to_json(json_file, aug_points):
    k = 0
    new_json = json_file
    shapes = new_json['shapes']
    for i in range(len(shapes)):
        for j in range(len(shapes[i]["points"])):
            new_point = [aug_points.keypoints[k].x, aug_points.keypoints[k].y]
            new_json['shapes'][i]["points"][j] = new_point
            k = k + 1
    return new_json


# -----------------------------Sequential-augument choose here-----
# ia.seed(1)

# Define our augmentation pipeline.
sometimes = lambda aug: iaa.Sometimes(0.3, aug)
seq = iaa.Sequential([
    # weather
    iaa.Sometimes(0.3, iaa.FastSnowyLandscape(lightness_threshold=40, lightness_multiplier=2)),
    iaa.Sometimes(0.3, iaa.Clouds()),
    iaa.Sometimes(0.3, iaa.Snowflakes(flake_size=(0.2, 0.7), speed=(0.007, 0.03))),
    iaa.Add((-10, 10)),
    iaa.Fliplr(0.5),
    iaa.Sometimes(0.3, iaa.GaussianBlur(sigma=(0, 1.0))),
    iaa.Sometimes(0.3, iaa.Multiply((0.8, 1.2))),
    iaa.Sometimes(0.3, iaa.Affine(rotate=(-3, 3))),  # rotate by -3 to 3 degrees (affects segmaps)
    iaa.Sometimes(0.3, iaa.CropAndPad(percent=(-0.05, 0.05), pad_mode=ia.ALL, pad_cval=(0, 255))),
], random_order=True)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--in_dir', type=str, required=True, help='input path.')
    parser.add_argument('--out_dir', type=str, required=True, help='output path.')
    parser.add_argument('--aug_times', type=int, default=5, required=True, help='each images aug times.')

    parser.set_defaults()
    return parser.parse_args()


def main(opt):
    # TO-DO-BELOW
    if os.path.exists(opt.out_dir):
        return "ERROR: output path already exists."
    if isinstance(opt.aug_times, int) and opt.aug_times < 2:
        return "ERROR: aug_times should bigger than 1."
    aug_times = opt.aug_times
    in_dir = opt.in_dir
    out_dir = opt.out_dir
    # ---check-------------
    mkdir(out_dir)
    imgs_dir_list = glob.glob(os.path.join(in_dir, '*.jpg'))
    check_json_file(imgs_dir_list)

    # for : image
    for idx_jpg_path in imgs_dir_list:
        idx_json_path = idx_jpg_path[:-3] + 'json'
        # get image file
        idx_img = cv2.imdecode(np.fromfile(idx_jpg_path, dtype=np.uint8), 1)
        # get json file
        idx_json = read_jsonfile(idx_json_path)
        # get point_list from json file
        points_list = get_points_from_json(idx_json)
        # convert to Keypoint(imgaug mode)
        kps = KeypointsOnImage([Keypoint(x=p[0], y=p[1]) for p in points_list], shape=idx_img.shape)

        # Augument Keypoints and images
        for idx_aug in range(aug_times):
            image_aug, kps_aug = seq(image=idx_img, keypoints=kps)
            image_aug.astype(np.uint8)
            # write aug_points in json file
            idx_new_json = write_points_to_json(idx_json, kps_aug)
            idx_new_json["imagePath"] = idx_jpg_path.split(os.sep)[-1][:-4] + '_' + str(idx_aug) + '.jpg'
            idx_new_json["imageData"] = str(utils.img_arr_to_b64(image_aug), encoding='utf-8')
            for i in idx_new_json['shapes']:
                if not i.get('points'):
                    continue
                i['points'] = np.array(i.get('points')).astype(float).tolist()

            # save
            new_img_path = os.path.join(out_dir, idx_jpg_path.split(os.sep)[-1][:-4] + '_' + str(idx_aug) + '.jpg')
            cv2.imwrite(new_img_path, image_aug)
            new_json_path = new_img_path[:-3] + 'json'
            save_jsonfile(idx_new_json, new_json_path)
    return f"INFO: data aug success, output path: {opt.out_dir}."


if __name__ == '__main__':
    opt = parse_args()
    try:
        ret = main(opt)
        print(ret)
    except Exception as e:
        print(f"ERROR:{e}")
        raise e
