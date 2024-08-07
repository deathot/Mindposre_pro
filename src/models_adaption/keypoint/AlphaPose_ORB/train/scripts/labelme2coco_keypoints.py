import base64
import io
import os
import cv2
import psutil
import yaml
import json
import shutil
import argparse
import numpy as np
import PIL.Image
from tqdm import tqdm
from glob import glob
from sklearn.model_selection import train_test_split
from shapely.geometry import Polygon
import sys
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))))
sys.path.append(BASE_DIR)
from src.models_adaption.config.config import ModelConfig

IMG_SUFFIX_TRANS = ".jpg"


def change_image_format(input_path):
    """
    统一当前文件夹下所有图像的格式，如'.jpg'
    :param input_path:当前文件路径
    :return:
    """
    externs = ['png', 'jpg', 'JPEG', 'BMP', 'bmp', 'webp']
    files = list()
    for extern in externs:
        files.extend(glob(input_path + "\\*." + extern))
    for file in files:
        name = ''.join(file.split('.')[:-1])
        file_suffix = file.split('.')[-1]
        if file_suffix != IMG_SUFFIX_TRANS.split('.')[-1]:
            new_name = name + IMG_SUFFIX_TRANS
            image = cv2.imread(file)
            cv2.imwrite(new_name, image)
            os.remove(file)

labelsStr2Int = {"nose" : 1,
                 "left_eye" : 2,
                 "right_eye" : 3, 
                 "left_ear" : 4,
                 "right_ear" : 5,
                 "left_shoulder" : 6,
                 "right_shoulder" : 7,
                 "left_elbow" : 8,
                 "right_elbow" : 9,
                 "left_wrist" : 10,
                 "right_wrist" : 11,
                 "left_hip" : 12,
                 "right_hip" : 13,
                 "left_knee" : 14,
                 "right_knee" : 15,
                 "left_ankle" : 16,
                 "right_ankle" : 17}

class Labelme2coco():
    def __init__(self, args):
        self.classname_to_id = {args.class_name: 1}
        self.images = []
        self.annotations = []
        self.categories = []
        self.img_id = 0
        self.ann_id = 0

    def save_coco_json(self, instance, save_path):
        json.dump(instance, open(save_path, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)

    def read_jsonfile(self, path):
        with open(path, "r", encoding='utf-8') as f:
            return json.load(f)

    def _get_box(self, points):
        min_x = min_y = np.inf
        max_x = max_y = 0
        for x, y in points:
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            max_x = max(max_x, x)
            max_y = max(max_y, y)
        return [min_x, min_y, max_x - min_x, max_y - min_y]

    
    def _img_data_to_pil(self, img_data):
        f = io.BytesIO()
        f.write(img_data)
        img_pil = PIL.Image.open(f)
        return img_pil

    def _img_data_to_arr(self, img_data):
        img_pil = self._img_data_to_pil(img_data)
        img_arr = np.array(img_pil)
        return img_arr


    def _img_b64_to_arr(self, img_b64):
        img_data = base64.b64decode(img_b64)
        img_arr = self._img_data_to_arr(img_data)
        return img_arr
    
    def _image(self, obj, path, coco_url):
        image = {}

        img_x = self._img_b64_to_arr(obj['imageData'])
        image['height'], image['width'] = img_x.shape[:-1]

        self.img_id = int(os.path.basename(path).split(".json")[0])
        image['id'] = self.img_id
        image['file_name'] = os.path.basename(path).replace(".json", ".jpg")
        image['coco_url'] = os.path.join(coco_url, image['file_name'])

        return image


    def _init_categories(self):
        for name, id in self.classname_to_id.items():
            category = {}
            category['supercategory'] = name
            category['id'] = id
            category['name'] = name
            category['keypoint'] = ['nose',
                                    'left_eye',
                                    'right_eye',
                                    'left_ear',
                                    'right_ear',
                                    'left_shoulder',
                                    'right_shoulder',
                                    'left_elbow',
                                    'right_elbow',
                                    'left_wrist',
                                    'right_wrist',
                                    'left_hip',
                                    'right_hip',
                                    'left_knee',
                                    'right_knee',
                                    'left_ankle',
                                    'right_ankle']
            category['skeleton'] = [[16, 14],
                                    [14, 12],
                                    [17, 15],
                                    [15, 13],
                                    [12, 13],
                                    [6, 12],
                                    [7, 13],
                                    [6, 7],
                                    [6, 8],
                                    [7, 9],
                                    [8, 10],
                                    [9, 11],
                                    [2, 3],
                                    [1, 2],
                                    [1, 3],
                                    [2, 4],
                                    [3, 5],
                                    [4, 6],
                                    [5, 7]]
            self.categories.append(category)

    def to_coco(self, json_path_list, coco_url):
        self._init_categories()
        instance = {}
        instance['info'] = {'description': 'Pose Estimation Dataset', 'version': 1.0, 'year': 2022}
        instance['license'] = ['Attribution License']
        instance['images'] = self.images

        for json_path in tqdm(json_path_list):
            obj = self.read_jsonfile(json_path)
            self.images.append(self._image(obj, json_path, coco_url))
            shapes = obj['shapes']

            # check number of people in the image
            num_person = 0
            isIndividual = False
            for shape in shapes:
                if shape['group_id'] == None:
                    isIndividual = True
                    continue
                if shape['group_id'] > num_person:
                    num_person = shape['group_id']
            print("Start annotate for img: ", json_path, "There are", num_person + 1, "people in total")
            
            for person in range(num_person + 1):
                print("Person", person + 1, "...")
                # start with person = 0, create annotation dict for each person
                annotation = {}
                person_annotation = []
                keypoints = [None] * (args.join_num + 1)
                for shape in shapes:
                    if shape['shape_type'] == 'point':
                        # iterate through keypoints, add to dict if belongs to person
                        if shape['group_id'] != person and isIndividual == False:
                            continue
                        # get the body part this keypoint represents
                        part_index = labelsStr2Int[shape['label']]
                        # store the keypoint data to keypoints[] at its respective index
                        keypoints[part_index] = shape['points']
                        
                    elif shape['shape_type'] == 'polygon':
                        bbox = shape['points']
                        annotation['segmentation'] = [np.asarray(bbox).flatten().tolist()]
                        annotation['bbox'] = self._get_box(bbox)
                        #annotation['area'] = annotation['bbox'][2] * annotation['bbox'][3]
                        poly = Polygon(bbox)
                        area_ = round(poly.area, 6)
                        annotation['area'] = area_
                
                # edit the keypoint data to fit COCO annotation format
                num_keypoints = 0
                for keypoint_i in range(1, args.join_num + 1):
                    # store keypoint for person in annotation
                    if keypoints[keypoint_i] == None:
                        person_annotation.extend([0, 0, 0])
                    else:
                        person_annotation.extend([keypoints[keypoint_i][0][0], keypoints[keypoint_i][0][1], 2])
                        num_keypoints += 1       
                
                annotation['id'] = self.ann_id
                annotation['image_id'] = self.img_id
                annotation['category_id'] = 1
                #annotation['area'] = 1.0
                annotation['iscrowd'] = 0
                annotation['num_keypoints'] = num_keypoints
                annotation['keypoints'] = person_annotation
                # add person annotation to image annotation
                # print("Annotated data: ", annotation)
                self.annotations.append(annotation)
                self.ann_id += 1
            
            # next image
            self.img_id += 1
        instance['annotations'] = self.annotations
        instance['categories'] = self.categories
        return instance


def ChangeToKeypoints(args):
    if psutil.cpu_percent(interval=1) > 72:
        return "ERROR:The CPU usage is too high. Please try again later."
    if not os.path.exists(args.input_path):
        return 'ERROR:input_path not exist.'
    if args.split_ratio <= 0 or args.split_ratio >= 1:
        return 'ERROR:split_ratio out of range (0,1).'
    change_image_format(args.input_path)
    json_dirs = glob(f'{args.input_path}/*.json')
    for _file in json_dirs:
        json_file = json.load(open(_file, "r", encoding="utf-8"))
        imagename = json_file['imagePath']
        if not os.path.exists(os.path.join(args.input_path, imagename)):
            raise Exception("There are pictures do not exist, please check the dataset.")
    if len(json_dirs) < 20:
        raise Exception("The dataset needs at least 20 pictures")
    labelme_path = args.input_path
    saved_coco_path = args.output_path
    if not os.path.exists("%s/annotations/"%saved_coco_path):
        os.makedirs("%s/annotations/"%saved_coco_path)
    if not os.path.exists("%s/train2017/"%saved_coco_path):
        os.makedirs("%s/train2017"%saved_coco_path)
    if not os.path.exists("%s/val2017/"%saved_coco_path):
        os.makedirs("%s/val2017"%saved_coco_path)
    if not os.path.exists("%s/test/"%saved_coco_path):
        os.makedirs("%s/test"%saved_coco_path)

    json_list_path = glob(labelme_path + "/*.json")
    trainval_path, test_path = train_test_split(json_list_path, test_size=0.1)
    train_path, val_path = train_test_split(trainval_path, test_size=args.split_ratio)
    print('{} for training'.format(len(train_path)),
          '\n{} for testing'.format(len(val_path)))
    print('Start transform please wait ...')

    train_url = "./train2017/"
    l2c_train = Labelme2coco(args)
    train_keypoints = l2c_train.to_coco(train_path, train_url)

    l2c_train.save_coco_json(train_keypoints, '%s/annotations/person_keypoints_train2017.json' % saved_coco_path)
    for file in train_path:
        shutil.copy(file.replace("json", "jpg"), "%s/train2017/" % saved_coco_path)
    for file in val_path:
        shutil.copy(file.replace("json", "jpg"), "%s/val2017/" % saved_coco_path)
    for file in test_path:
        shutil.copy(file.replace("json", "jpg"), "%s/test/" % saved_coco_path)

    val_url = "./val2017/"
    l2c_val = Labelme2coco(args)
    val_instance = l2c_val.to_coco(val_path, val_url)
    l2c_val.save_coco_json(val_instance, '%s/annotations/person_keypoints_val2017.json' % saved_coco_path)
    
    with open(args.cfg) as f:
        doc = yaml.load(f, Loader=yaml.FullLoader)
        tree = yaml.compose(f)
        print(tree)
    doc['DATASET']['TRAIN']['ROOT'] = args.output_path
    doc['DATASET']['TEST']['ROOT'] = args.output_path
    doc['DATASET']['VAL']['ROOT'] = args.output_path
    yamlpath = os.path.join(args.output_path, "data" + ".yaml")
    with open(yamlpath, 'w') as f:
        yaml.dump(doc, f, default_flow_style=False)
    print("process_value=5")
    return 'INFO:create dataset success!'
    
def parse_args():
    parser = argparse.ArgumentParser("Transfer labelme dataset into alphapose format.")
    parser.add_argument("--cfg", "--c", help="config name", type=str,
                        default="./configs/coco/resnet/256x192_res50_lr1e-3_1x.yaml")
    parser.add_argument("--class_name", "--n", help="class name", type=str, default="person")
    parser.add_argument("--join_num", "--j", help="number of join", type=int, default=17)
    parser.add_argument("--input_path", "--i", help="json file path (labelme)", type=str, default='')
    parser.add_argument("--output_path", "--o", help="output file path (coco format)", type=str, default='')
    parser.add_argument("--split_ratio", "--r", help="train and test split ratio", type=float, default=0.2)
    parser.set_defaults()
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    # args.split_ratio = float(args.split_ratio)
    yaml_file = '../../../config/config.yaml'
    config = ModelConfig(yaml_file)
    yaml_data = config.get_yaml_data().get('keypoint')
    args.input_path = yaml_data['labelme_dataset_path']
    trans_output_path = os.path.join(yaml_data['output_path'], 'trans_output')
    args.split_ratio = yaml_data['split_ratio']
    args.output_path = trans_output_path
    config.set_para('keypoint', 'trans_output', trans_output_path)
    config.yaml_dump(yaml_file)
    try:
        ret = ChangeToKeypoints(args)
        print(ret)
    except Exception as e:
        print(f"ERROR:{e}")