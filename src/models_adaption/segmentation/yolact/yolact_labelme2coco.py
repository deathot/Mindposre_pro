# -*- coding:utf-8 -*-
from labelme import utils
import PIL.Image
import os
import json
import argparse
import shutil
import numpy as np
from glob import glob
from sklearn.model_selection import train_test_split
 
IMG_SUFFIX_TRANS = ".jpg"


class MyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return super(MyEncoder, self).default(obj)
 
 
class labelme2coco(object):
    def __init__(self, labelme_json=[], save_json_path='./tran.json'):

        self.labelme_json = labelme_json
        self.save_json_path = save_json_path
        self.images = []
        self.categories = []
        self.annotations = []
        # self.data_coco = {}
        self.label = []
        self.annID = 1
        self.height = 0
        self.width = 0
 
        self.save_json()
 
    def data_transfer(self):
 
        for num, json_file in enumerate(self.labelme_json):
            with open(json_file, 'r') as fp:
                data = json.load(fp)  # 加载json文件
                self.images.append(self.image(data, num))
                for shapes in data['shapes']:
                    label = shapes['label']
                    if label not in self.label:
                        self.categories.append(self.categorie(label))
                        self.label.append(label)

                    points = shapes['points']  # 这里的point是用rectangle标注得到的，只有两个点，需要转成四个点
                    # points.append([points[0][0], points[1][1]])
                    # points.append([points[1][0], points[0][1]])
                    self.annotations.append(self.annotation(points, label, num))
                    self.annID += 1
 
    def image(self, data, num):
        image = {}
        print(data["imagePath"])
        img = utils.img_b64_to_arr(data['imageData'])  # 解析原图片数据
        # img=io.imread(data['imagePath']) # 通过图片路径打开图片
        # img = cv2.imread(data['imagePath'], 0)
        height, width = img.shape[:2]
        img = None
        image['height'] = height
        image['width'] = width
        image['id'] = num + 1
        image['file_name'] = data['imagePath'].split('/')[-1]
 
        self.height = height
        self.width = width
 
        return image
 
    def categorie(self, label):
        categorie = {}
        categorie['supercategory'] = 'Cancer'
        categorie['id'] = len(self.label) + 1  # 0 默认为背景
        categorie['name'] = label
        return categorie
 
    def annotation(self, points, label, num):
        annotation = {}
        annotation['segmentation'] = [list(np.asarray(points).flatten())]
        annotation['iscrowd'] = 0
        annotation['image_id'] = num + 1
        # annotation['bbox'] = str(self.getbbox(points)) # 使用list保存json文件时报错（不知道为什么）
        # list(map(int,a[1:-1].split(','))) a=annotation['bbox'] 使用该方式转成list
        annotation['bbox'] = list(map(float, self.getbbox(points)))
        annotation['area'] = annotation['bbox'][2] * annotation['bbox'][3]
        # annotation['category_id'] = self.getcatid(label)
        annotation['category_id'] = self.getcatid(label)  # 注意，源代码默认为1
        annotation['id'] = self.annID
        return annotation
 
    def getcatid(self, label):
        for categorie in self.categories:
            if label == categorie['name']:
                return categorie['id']
        return 1
 
    def getbbox(self, points):
        # img = np.zeros([self.height,self.width],np.uint8)
        # cv2.polylines(img, [np.asarray(points)], True, 1, lineType=cv2.LINE_AA)  # 画边界线
        # cv2.fillPoly(img, [np.asarray(points)], 1)  # 画多边形 内部像素值为1
        polygons = points
 
        mask = self.polygons_to_mask([self.height, self.width], polygons)
        return self.mask2box(mask)
 
    def mask2box(self, mask):
        '''从mask反算出其边框
        mask：[h,w]  0、1组成的图片
        1对应对象，只需计算1对应的行列号（左上角行列号，右下角行列号，就可以算出其边框）
        '''
        # np.where(mask==1)
        index = np.argwhere(mask == 1)
        rows = index[:, 0]
        clos = index[:, 1]
        # 解析左上角行列号
        left_top_r = np.min(rows)  # y
        left_top_c = np.min(clos)  # x
 
        # 解析右下角行列号
        right_bottom_r = np.max(rows)
        right_bottom_c = np.max(clos)
 

        return [left_top_c, left_top_r, right_bottom_c - left_top_c, right_bottom_r - left_top_r]  # [x1,y1,w,h] 对应COCO的bbox格式
 
    def polygons_to_mask(self, img_shape, polygons):
        mask = np.zeros(img_shape, dtype=np.uint8)
        mask = PIL.Image.fromarray(mask)
        xy = list(map(tuple, polygons))
        PIL.ImageDraw.Draw(mask).polygon(xy=xy, outline=1, fill=1)
        mask = np.array(mask, dtype=bool)
        return mask
 
    def data2coco(self):
        data_coco = {}
        data_coco['images'] = self.images
        data_coco['categories'] = self.categories
        data_coco['annotations'] = self.annotations
        return data_coco
 
    def save_json(self):
        self.data_transfer()
        self.data_coco = self.data2coco()
        # 保存json文件
        json.dump(self.data_coco, open(self.save_json_path, 'w'), indent=4, cls=MyEncoder)  # indent=4 更加美观显示
 

def split_dataset(input_path, split_ratio, split_test):
    """
    将文件分为训练集，测试集和验证集
    :param split_ratio: 分割测试集或验证集的比例
    :param split_test: 是否使用测试集，默认为False
    :param input_path:当前文件路径
    :return:
    """
    files = glob(input_path + "\\*.json")
    files = [i.replace("\\", "/").split("/")[-1].split(".json")[0] for i in files]

    test_files = None
    if split_test:
        trainval_files, test_files = train_test_split(files, test_size=split_ratio, random_state=55)
    else:
        trainval_files = files
    train_files, val_files = train_test_split(trainval_files, test_size=split_ratio, random_state=55)

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
    train_label = os.path.join(output_path, 'train')
    if not os.path.exists(train_label):
        os.makedirs(train_label)
    # 生成验证集
    val_image = os.path.join(output_path, 'valid', 'images')
    if not os.path.exists(val_image):
        os.makedirs(val_image)
    val_label = os.path.join(output_path, 'valid')
    if not os.path.exists(val_label):
        os.makedirs(val_label)
    # 生成测试集
    if split_test:
        test_image = os.path.join(output_path, 'test', 'images')
        if not os.path.exists(test_image):
            os.makedirs(test_image)
        test_label = os.path.join(output_path, 'test')
        if not os.path.exists(test_label):
            os.makedirs(test_label)
        return train_image, train_label, val_image, val_label, test_image, test_label
    else:
        return train_image, train_label, val_image, val_label


def push_into_file(file, images, input_path):
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
        if not os.path.exists(os.path.join(images, filename + IMG_SUFFIX_TRANS)):
            try:
                shutil.copy(image_file, images)
            except OSError:
                pass



def Change2Coco(input_path, output_path, split_ratio, split_test):
    """
    生成最终标准格式的文件
    :param split_ratio: 分割测试集或验证集的比例
    :param input_path:当前文件路径
    :param output_path:输出文件路径
    :param split_test: 是否使用测试集
    :return:
    """
    train_files,val_files,test_files,__ = split_dataset(input_path,split_ratio,split_test)

    train_jsons = [os.path.join(input_path,train_file+'.json') for train_file in train_files]
    val_jsons = [os.path.join(input_path,val_file+'.json') for val_file in val_files]
    if split_test:
        test_jsons = [os.path.join(input_path,test_file+'.json') for test_file in test_files]
        train_image, train_label, val_image, val_label, test_image, test_label = create_save_dir(output_path, split_test)
    else:
        train_image, train_label, val_image, val_label = create_save_dir(output_path, split_test)

    labelme2coco(train_jsons, os.path.join(train_label,'train_annotation.json'))
    labelme2coco(val_jsons, os.path.join(val_label,'val_annotation.json'))
    if split_test:
        labelme2coco(test_jsons,os.path.join(test_label,'test_annotation.json'))

    push_into_file(train_files, train_image, input_path)
    push_into_file(val_files, val_image, input_path)
    if split_test:
        push_into_file(test_files, test_image, input_path)
    print('create dataset done')





if __name__ == "__main__":
    input_path="./sample_data"
    output_path = "./yolact_master/data/coco_format_data"
    split_ratio = 0.3
    Change2Coco(input_path,output_path,split_ratio,False) # 因为现在没有用测试集,所以默认false



