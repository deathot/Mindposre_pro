#!/usr/bin/env python

from __future__ import print_function

import glob
import os
import os.path as osp
import shutil

import numpy as np
import labelme
import cv2


def convert_label_to_mask(input_dir, output_dir, label_file, ext):

    if osp.exists(output_dir):
        print("{} already exists, delete it".format(output_dir))
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)
    os.makedirs(osp.join(output_dir, "SegmentationClass"))

    print("Creating dataset:", output_dir)

    class_names = []
    class_name_to_id = {}
    for i, line in enumerate(open(label_file).readlines()):
        class_id = i - 1  # starts with -1
        class_name = line.strip()
        class_name_to_id[class_name] = class_id
        if class_id == -1:
            assert class_name == "__ignore__"
            continue
        elif class_id == 0:
            assert class_name == "_background_"
        class_names.append(class_name)
    class_names = tuple(class_names)
    print("class_names:", class_names)
    out_class_names_file = osp.join(output_dir, "class_names.txt")
    with open(out_class_names_file, "w") as f:
        f.writelines("\n".join(class_names))
    print("Saved class_names:", out_class_names_file)

    for filename in glob.glob(osp.join(input_dir, "*.json")):
        print("Generating dataset from:", filename)

        label_file = labelme.LabelFile(filename=filename)

        base = osp.splitext(osp.basename(filename))[0]
        out_lbl_file = osp.join(
            output_dir, "SegmentationClass", base + ".npy"
        )

        img = cv2.imread(filename[:-5]+ext)

        lbl, _ = labelme.utils.shapes_to_label(
            img_shape=img.shape,
            shapes=label_file.shapes,
            label_name_to_value=class_name_to_id,
        )

        np.save(out_lbl_file, lbl)
