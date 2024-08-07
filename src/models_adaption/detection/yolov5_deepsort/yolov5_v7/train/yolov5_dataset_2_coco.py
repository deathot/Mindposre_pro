import os
import json
import yaml
import argparse
from pathlib import Path
from PIL import Image


def yolov5_2_coco_annotation(coco_format_json_path, yolo_data_yaml_file_path, yolo_img_path, yolo_annotation_path):
    with open(yolo_data_yaml_file_path, errors='ignore') as f:
        yaml_data = yaml.safe_load(f)
    names = yaml_data['names']
    index = 0
    categories = []
    for v in names.values():
        index = index + 1
        label = v
        categories.append({'id': index, 'name': label, 'supercategory': 'None'})

    write_json_context = dict()  # 写入.json文件的大字典
    write_json_context['info'] = {'description': '', 'url': '', 'version': '', 'year': 2022, 'contributor': '',
                                  'date_created': '2022-01-01'}
    write_json_context['licenses'] = [{'id': 1, 'name': None, 'url': None}]
    write_json_context['categories'] = categories
    write_json_context['images'] = []
    write_json_context['annotations'] = []

    # 接下来的代码主要添加'images'和'annotations'的key值
    image_file_list = os.listdir(yolo_img_path)  # 遍历该文件夹下的所有文件，并将所有文件名添加到列表中
    for i, image_file in enumerate(image_file_list):
        image_path = os.path.join(yolo_img_path, image_file)  # 获取图片的绝对路径
        image = Image.open(image_path)  # 读取图片，然后获取图片的宽和高
        W, H = image.size

        img_context = dict()  # 使用一个字典存储该图片信息
        # img_name=os.path.basename(imagePath)       #返回path最后的文件名。如果path以/或\结尾，那么就会返回空值
        img_context['file_name'] = image_file
        img_context['height'] = H
        img_context['width'] = W
        img_context['date_captured'] = '2022-01-01'
        img_context['id'] = int(Path(image_file).stem)  # 该图片的id, t
        img_context['license'] = 1
        img_context['color_url'] = ''
        img_context['flickr_url'] = ''
        write_json_context['images'].append(img_context)  # 将该图片信息添加到'image'列表中

        txt_file = Path(image_file).stem + '.txt'  # 获取该图片获取的txt文件
        with open(os.path.join(yolo_annotation_path, txt_file), 'r') as fr:
            lines = fr.readlines()  # 读取txt文件的每一行数据，lines2是一个列表，包含了一个图片的所有标注信息
        for j, line in enumerate(lines):
            bbox_dict = {}  # 将每一个bounding box信息存储在该字典中

            class_id, x, y, w, h = line.strip().split(' ')
            class_id, x, y, w, h = int(class_id), float(x), float(y), float(w), float(h)

            xmin = (x - w / 2) * W  # 坐标转换
            ymin = (y - h / 2) * H
            xmax = (x + w / 2) * W
            ymax = (y + h / 2) * H
            w = w * W
            h = h * H

            bbox_dict['id'] = i * 10000 + j  # bounding box的坐标信息
            bbox_dict['image_id'] = int(Path(image_file).stem)
            bbox_dict['category_id'] = class_id + 1  # 注意目标类别要加一
            bbox_dict['iscrowd'] = 0
            height, width = abs(ymax - ymin), abs(xmax - xmin)
            bbox_dict['area'] = height * width
            bbox_dict['bbox'] = [xmin, ymin, w, h]
            bbox_dict['segmentation'] = [[xmin, ymin, xmax, ymin, xmax, ymax, xmin, ymax]]
            write_json_context['annotations'].append(bbox_dict)

    with open(coco_format_json_path, 'w') as fw:  # 将字典信息写入.json文件中
        json.dump(write_json_context, fw, indent=2)

def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--coco_format_json_path', type=str, default='coco.json', help='coco format json file path')
    parser.add_argument('--yolo_data_yaml_file_path', type=str, default='data.yaml', help='yolo data yaml file path')
    parser.add_argument('--yolo_img_path', type=str, default='test/images', help='yolo dataset img dir path')
    parser.add_argument('--yolo_annotation_path', type=str, default='test/labels',
                        help='yolo dataset annotation dir path')

    opt = parser.parse_args()
    return opt


if __name__ == '__main__':
    opt = parse_opt()
    yolov5_2_coco_annotation(opt.coco_format_json_path, opt.yolo_data_yaml_file_path,
                             opt.yolo_img_path, opt.yolo_annotation_path)
