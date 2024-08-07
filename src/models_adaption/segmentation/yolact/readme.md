# Yolact 模型

## 一、数据转换

### 1、labelme 标注

启动labelme,选择图片文件夹,开始标注检测区域及其类别
labelme输出可放在 "./sample_data" 文件夹下,包括图片和标注信息 json

标注示例:
![输入图片说明](https://foruda.gitee.com/images/1661824378437951094/58cff6d3_11359706.png "示例图片1.png")

### 2、数据集转换脚本

运行 'yolact_labelme2coco.py'

开放参数: 
```
input_path:  labelme 标注的图片文件夹，默认："./sample_data"
split_ratio: 训练集和验证集分割比例，默认：0.3
```

输出文件夹:
```
训练数据:"train_code/data/coco_format_data/train/images"
训练标签:"train_code/data/coco_format_data/train/train_annotation.json"
验证数据:"train_code/data/coco_format_data/valid/images"
验证标签:"train_code/data/coco_format_data/valid/val_annotation.json"
(因为与后面模型加载数据路径相关联,暂未开放接口)
```
![输入图片说明](https://foruda.gitee.com/images/1661824538012097976/67d1a964_11359706.png "示例图片2.png")

## 二、PC端迁移学习训练

### 1、准备预训练模型

进入链接 https://github.com/dbolya/yolact , 点击下图所示方框, 下载预训练模型 yolact_base_54_800000.pth ,将其放入 "train_code/weights" 文件夹下

![输入图片说明](https://foruda.gitee.com/images/1661824551391340635/857f08f6_11359706.jpeg "模型链接.jpg")


### 2、进入 train_code 目录,运行 train_transfer.py 文件

开放参数:
```
pretrained_weights: 预训练模型路径(开源模型),默认:'./weights/yolact_base_54_800000.pth'
class_names:        给定数据集的类别,元组类型,默认:('person','bear','car')
max_iter:           最大循环iteration数量,epoch=max_iter/(len(dadtaset)//batch_size), 默认:100(为了打通流程设置较小,为20)
save_mdl_name:      输出模型名字,在此名字基础上会添加 '_epoch数_iter数',默认: 'yolact_base_transfer'
```
默认输出模型文件:
```
./weights/yolact_base_transfer_6_20.pth
```

## 三、PC 端评估

**进入 train_code 目录,运行 eval.py 文件**

1.查看验证集的 mAP 结果, 开放参数:
```
trained_model: 已训练好的模型,默认:'weights/yolact_base_transfer_6_20.pth'
class_names:   给定数据集的类别,元组类型,注意需要和训练时所输入的 class_names 相同,默认:('person','bear','car')
```

2.查看单张图片分割结果,开放参数:
```
image_path: 传入图片路径,默认: None 
trained_model: 已训练好的模型,默认:'weights/yolact_base_transfer_6_20.pth'
class_names:   给定数据集的类别,元组类型,注意需要和训练时所输入的 class_names 相同,默认:('person','bear','car')
score_threshold: 只查看在此得分以上的 bbox,默认:0.15 
top_k: 只查看得分前 top_k 的 bbox,默认: 2
```
其中 image_path 为必须传入的参数, 如: 'data/coco_format_data/train/images/000000078823.jpg'
若为默认的 None,则为查看验证集 mAP 结果,不显示图像分割结果


## 四、模型转换

### 1、PC 端 pth 转 onnx

**进入train_code目录,运行 pth2onnx.py 文件**
开放参数:
```
weight_path:     已训练好的模型(.pth格式), 默认:"weights/yolact_base_transfer_6_20.pth"
onnx_model_path: 输出的模型(.onnx格式), 默认:"weights/yolact_base_transfer_6_20.onnx"
```
此时,"weights/"文件夹下包含:

![输入图片说明](https://foruda.gitee.com/images/1661824577991223838/0e074b99_11359706.png "示例图片3.png")

### 2、310/200DK 上 onnx 转 om
将onnx模型传到200DK上,进入所在文件夹,并执行命令:
```
atc --framework=5 --model=ONNX_PATH --input_format=NCHW --input_shape="input.1:1,3,550,550" --output=OM_NAME --log=debug --soc_version=Ascend310 --out_nodes="Transpose_298:0;Concat_391:0;Softmax_393:0;Concat_389:0"
```
可改参数:
```
ONNX_PATH: onnx 的文件路径,如: yolact_base_transfer_6_20.onnx
OM_NAME:   om模型的文件名,如: yolact_base_transfer_6_20
```
输出文件名:
```
yolact_base_transfer_6_20.om
```

## 四、模型推理(310/200DK)
代码正在进行中





