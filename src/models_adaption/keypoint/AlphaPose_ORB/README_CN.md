# 目录

<!-- TOC -->

- [目录](#目录)
- [数据集格式转换](#数据集格式转换)
  - [Labelme标注](#Labelme标注)
  - [执行转换脚本](#执行转换脚本)
- [模型训练](#模型训练)
  - [准备数据集](#准备数据集)
  - [执行训练](#执行训练)
  - [模型评估](#模型评估)
  - [模型推理](#模型推理)
  - [姿态跟踪](#姿态跟踪)
- [模型转换](#模型转换)
  - [导出onnx模型](#导出onnx模型)
  - [转换om模型](#转换om模型)
- [模型推理](#模型推理)

<!-- /TOC -->

## 数据集格式转换

用户自行准备的数据集，使用Labelme进行标注，并转换为模型训练可加载的coco关键点格式。

### Labelme标注

需要标注信息：polygon格式的分割区域和points格式的关键点。

标注分割区域：标签名称为person，填写group_id。

标注关键点：标签名称可选择【nose,  left_eye,  right_eye,  left_ear,  right_ear,  left_shoulder,  right_shoulder,  left_elbow,  right_elbow,  left_wrist,  right_wrist, left_hip,  right_hip,  left_knee,  right_knee, left_ankle,  right_ankle】，填写和person对应的group_id。

标注示例：
![输入图片说明](https://foruda.gitee.com/images/1661862811898295719/44e3f6d6_9461593.png "标注示例.PNG")

### 执行转换脚本

```
# 命令行：
python ./scripts/labelme2coco_keypoints.py [--input] [--output] [--ratio]
# 示例：
python ./scripts/labelme2coco_keypoints.py --input=./data/input --output=./data/
```

GUI支持用户修改input、output、ratio。

参数说明：

| 参数     | 说明                |
| -------- | ------------------- |
| --input  | 输入路径            |
| --output | 输出路径            |
| --ratio  | 训练/验证集划分比例 |

```
# 执行结果示例：
16 for training 
4 for testing
Start transform please wait ...
  0%|                                                             | 0/16 [00:00<?, ?it/s]
Start annotate for img:  ./data/input\000000239537.json There are 1 people in total
Person 1 ...
...
 19%|███████████████████████                             |  3/16 [00:00<00:00, 22.24it/s]
Start annotate for img:  ./data/input\000000177489.json There are 2 people in total
Person 1 ...
Person 2 ...
...
100%█████████████████████████████████████████████████████| 16/16 [00:00<00:00, 39.63it/s]
```

执行完成后，在output指定目录下生成转换后的coco关键点格式数据集，如下：

```
data
├── coco
│   ├── annotations
│   │   ├── person_keypoints_train2017.json
│   │   ├── person_keypoints_val2017.json
│   ├── train2017
│   │   ├── 000000162415.jpg
│   │   ├── ......
│   ├── val2017
│   │   ├── 000000020333.jpg
│   │   ├── ......
```



## 模型训练

### 准备数据集

完成步骤一数据集格式转换，得到coco关键点格式数据集，在模型代码根目录下创建data文件夹，将转换完成的数据集放在该文件夹下。

### 执行训练

（1）准备预训练模型文件

```
a. 手动下载检测器模型yolov3-spp.weights，并放在"detector/yolo/data"目录中（训练的验证过程需要加载）
b. (可选)下载预训练模型(如resnet50-0676ba61.pth)放在/root/.cache/torch/checkpoints/目录下。如果执行训练时下载不成功，可以根据提示的下载链接手动下载放在该目录下，再次执行训练。
```

（2）执行训练

```
# 命令行：
python ./scripts/train.py [--cfg][--batch_size][--end_epoch][--pretrained][--try_load]

# 直接训练（重新训练），示例：
python ./scripts/train.py --cfg=./configs/coco/resnet/256x192_res50_lr1e-3_1x.yaml --batch_size=2 --end_epoch=2

# 断点续训（加载预训练模型，继续训练），示例：
python ./scripts/train.py --cfg=./configs/coco/resnet/256x192_res50_lr1e-3_1x.yaml --batch_size=2 --end_epoch=2 --pretrained=./fast_res50_256x192.pth

# 移学习训练 （加载预训练模型，换数据集训练），示例：
python ./scripts/train.py --cfg=./configs/coco/resnet/256x192_res50_lr1e-3_1x.yaml --batch_size=2 --end_epoch=2 --try_load=./fast_res50_256x192.pth
```

参数说明：

| 参数         | 是否必需 | 说明                                                         |
| ------------ | -------- | ------------------------------------------------------------ |
| --cfg        | 是       | 训练配置文件（预置了数据集、模型、检测器参数，可根据实际情况修改） |
| --batch_size | 是       | 每次训练加载的图片数                                         |
| --end_epoch  | 是       | 训练的迭代次数                                               |
| --pretrained | 否       | 断点续训加载的预训练模型                                     |
| --try_load   | 否       | 迁移学习加载的预训练模型                                     |

```
# 执行结果示例：
******************************
Namespace(batch_size=2, ..., try_load='./fast_res50_256x192.pth', work_dir='./exp/default-256x192_res50_lr1e-3_1x.yaml/', world_size=4)
******************************
{'DATASET': {'TRAIN': ..., 'FILE_NAME': '256x192_res50_lr1e-3_1x.yaml'}
******************************
Loading model from ./fast_res50_256x192.pth...
############# Starting Epoch 0 | LR: 0.001 #############
Train-0 epoch | loss:0.00040714 | acc:0.8779
############# Starting Epoch 1 | LR: 0.001 #############
Train-1 epoch | loss:0.00024426 | acc:0.9516
##### Epoch 1 | gt mAP: 0.7069306930693069 | rcnn mAP: 0.7069306930693069 #####
```

执行完成后，在代码根目录下生成exp文件夹，如下：

```
AlphaPose-master
├── exp
│   ├── default-256x192_res50_lr1e-3_1x.yaml
│   │   ├── final_DPG.pth
│   │   ├── model_1.pth
│   │   ├── test_gt_kpt.json
│   │   ├── test_kpt.json
│   │   ├── training.log
│   ├── json
│   │   ├── test_det_yolo.json
```



### 模型评估

```
# 命令行：
python ./scripts/validate.py [--cfg] [--checkpoint]
# 示例：
python ./scripts/validate.py --cfg=./configs/coco/resnet/256x192_res50_lr1e-3_1x.yaml --checkpoint=./fast_res50_256x192.pth
```

参数说明：

| 参数         | 是否必需 | 说明     |
| ------------ | -------- | -------- |
| --cfg        | 是       | 配置文件 |
| --checkpoint | 是       | 模型文件 |

```
执行结果示例：
Loading model from ./fast_res50_256x192.pth...
loading annotations into memory...
Done (t=0.01s)
creating index...
index created!
100%|██████████████████████████████████████████████████████| 1/1 [00:10<00:00, 10.66s/
Detection results exist, will use it
100%|██████████████████████████████████████████████████████| 1/1 [00:10<00:00, 10.88s/it] 
##### gt box: 0.5658415841584158 mAP | det box: 0.4173267326732673 mAP #####
```



### 模型推理

```
# 命令行：
--checkpoint [--detector] [--cfg] [--checkpoint] [--indir] [--outdir]
# 示例
python scripts/demo_inference.py --detector=yolox-x(可选) --cfg=./configs/coco/resnet/256x192_res50_lr1e-3_1x.yaml --checkpoint=./fast_res50_256x192.pth --indir=./data/images/ --outdir=./data/result/
```

| 参数         | 是否必需 | 说明                                                  |
| ------------ | -------- | ----------------------------------------------------- |
| --detector   | 否       | 检测器，可根据实际情况设置，如使用yolox-x作为detector |
| --cfg        | 是       | 配置文件                                              |
| --checkpoint | 是       | 模型文件                                              |
| --indir      | 是       | 推理输入路径                                          |
| --outdir     | 是       | 推理输出                                              |

执行完成后，在outdir目录下生成alphapose-result.json文件（关键点信息）、vis文件夹（推理可视化结果）

```
推理结果示例：
Loading YOLOX-X model..
Loading pose model from pretrained_models/fast_res50_256x192.pth...
100%███████████████████████████████████████████████████████| 3/3 [00:47<00:00, 15.93s/it]
===========================> Finish Model Running.
===========================> Rendering remaining images in the queue...
===========================> If this step takes too long, you can enable the --vis_fast flag to use fast rendering (real-time).
Results have been written to json.
```



### 姿态跟踪

```
(1)准备模型文件：
   (a)'detector/yolox/data/yolox_x.pth'(可选)
   (b)'pretrained_models/fast_res50_256x192.pth'
   (c)准备视频编解码器：openh264-1.8.0-win64.dll

(2)视频跟踪，执行一步完成：
# 命令行：
python scripts/demo_inference.py [--detector] [--cfg] [--checkpoint] [--video] [--outdir] [--save_video] [--pose_flow] [--vis_fast]
# 示例：
python scripts/demo_inference.py --detector=yolox-x(可选) --cfg=./configs/coco/resnet/256x192_res50_lr1e-3_1x.yaml --checkpoint=./pretrained_models/fast_res50_256x192.pth --video=./data/video/sample.mp4 --outdir=./data/video/result/ --save_video --pose_flow --vis_fast

(3)图片序列跟踪，需要执行两步：
(a) python scripts/demo_inference.py --detector=yolox-x(可选) --cfg=./configs/coco/resnet/256x192_res50_lr1e-3_1x.yaml --checkpoint=./pretrained_models/fast_res50_256x192.pth --indir=./data/images/input/ --outdir=./data/images/result/
(b) python ./trackers_files/PoseFlow/tracker-general.py --imgdir=./data/images/input --in_json=./data/images/result/alphapose-results.json --out_json=./data/images/result/alphapose-results-forvis-tracked.json --visdir=./data/images/result/vis
```

参数说明：

| 参数             | 是否必需 | 说明                                                  |
| ---------------- | -------- | ----------------------------------------------------- |
| --detector       | 否       | 检测器，可根据实际情况设置，如使用yolox-x作为detector |
| --cfg            | 是       | 配置文件                                              |
| --checkpoint     | 是       | 模型文件                                              |
| --video/--imgdir | 是       | 输入视频/图片序列路径                                 |
| --outdir         | 是       | 跟踪结果路径                                          |
| --save_video     | 否       | 保存视频跟踪结果                                      |
| --pose_flow      | 否       | 使用PoseFlow跟踪视频中的人体姿态                      |
| --vis_fast       | 否       | 使用快速渲染                                          |

执行完成后在outdir指定的目录下生成跟踪结果文件：

```
# 视频跟踪结果，如下示例：
Loading YOLOX-X model..
Loading pose model from pretrained_models/fast_res50_256x192.pth...
Start pose tracking...

  0%|                                                | 0/178 [00:00<?, ?it/s] 
        OpenH264 Video Codec provided by Cisco Systems, Inc.

100%█████████████████████████████████████████████████| 178/178 [1:08:09<00:00, 22.98s/it] 
===========================> Finish Model Running.
Results have been written to json.ring remaining 0 images in the queue...
```

视频跟踪结果文件，如下：

```
AlphaPose-master/data/video
├── result
│   ├── poseflow
│   │   ├── matching
│   │   │   ├── 0_1_orb.txt
│   │   │   ├── 1_2_orb.txt
│   │   │   ├── ......
│   ├── AlphaPose_sample.mp4
│   ├── alphapose-results.json
```



## 模型转换

### 导出onnx模型

```
# 命令行：
python pthtar2onnx.py [--cfg] [--input_pth] [--output_onnx]
# 示例：
python pthtar2onnx.py --cfg=./configs/coco/resnet/256x192_res50_lr1e-3_1x.yaml --input_pth=./fast_res50_256x192.pth --output_onnx=./alphapose_fastpose_cpu.onnx
```

GUI支持用户传入cfg、input、output参数

### 转换om模型

准备好导出的onnx模型文件。

```
# 命令行：
atc --framework=5 \
    --input_format=NCHW \
    --model=fast_res50_256x192_bs1.onnx \
    --output=fast_res50_256x192_bs1_aipp_rgb \
    --input_shape="input:1,3,256,192" \
    --output_type=FP32 \
    --soc_version=Ascend310
```



## 模型推理

1、参考infer文件的README.md进行推理

2、运行图片推理示例：

```
bash run.sh image
如果有core dumped，注释MxpiAlphaposePreProcess.cpp的218、219行。
```

![输入图片说明](https://foruda.gitee.com/images/1661826039904555181/224e40b2_9461593.png "图片推理结果.PNG")