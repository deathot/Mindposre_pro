[toc]

## 1. 数据标注

启动labelme,选择图片文件夹,开始标注检测区域及其类别
labelme输出可放在 "./sample_data" 文件夹下,包括图片和标注信息 json

标注示例:

![输入图片说明](https://foruda.gitee.com/images/1661824378437951094/58cff6d3_11359706.png "示例图片1.png")



## 2. 数据转换

**运行 yolact_labelme2coco.py：**

```
# 命令行
python yolact_labelme2coco.py [--input_path] [--output_path] [--split_ratio] [--split_test]
# 示例
python yolact_labelme2coco.py --input_path sample_data --output_path out_data --split_ratio 0.2 --split_test False
```

**参数说明：**

| 参数        | 是否必需 | 说明                | 默认值                           |
| ----------- | -------- | ------------------- | -------------------------------- |
| input_path  | 是       | 输入路径            | sample_data                      |
| output_path | 是       | 输出路径            | yolact_edge/data/coco_small_data |
| split_ratio | 否       | 训练/验证集划分比例 | 0.3                              |
| split_test  | 否       | 是否划分测试集      | False                            |

**输出：**

```
|-- output_path
    |-- train
    |   |-- train_annotation.json
    |   |-- images
    |       |-- 000000000785.jpg
    |       |-- ......
    |-- valid
        |-- val_annotation.json
        |-- images
            |-- 000000008532.jpg
            |-- ......
```



## 3. PC端模型训练

**文件准备：**

进入链接 https://github.com/dbolya/yolact , 点击下图所示方框, 下载预训练模型 yolact_base_54_800000.pth ,将其放入 "weights" 文件夹下，若没有此文件夹，则创建

![输入图片说明](https://foruda.gitee.com/images/1661824551391340635/857f08f6_11359706.jpeg "模型链接.jpg")

**进入 yolact_edge 文件夹，运行 train.py：**

```
# 命令行
python train.py [--resume] [--train_images] [--train_info] [--class_names] [--max_iter] [--print_every_iternum] [--save_folder]
# 示例
python train.py --resume weights/yolact_edge_resnet50_54_800000.pth --train_images data/coco_small_data/train/images --train_info data/coco_small_data/train/train_annotation.json --class_names ('person','bear','car') --max_iter 20 --print_every_iternum 1
```

**参数说明：**

| 参数                | 是否必需 | 说明                                 | 默认值                                           |
| ------------------- | -------- | ------------------------------------ | ------------------------------------------------ |
| resume              | 是       | 预训练模型路径                       | ./weights/yolact_base_54_800000.pth              |
| train_images        | 是       | 训练集文件夹路径                     | data/coco_small_data/train/images                |
| train_info          | 是       | 训练集的标签文件                     | data/coco_small_data/train/train_annotation.json |
| class_names         | 是       | 训练集的类别，元组类型               | ('person','bear','car')                          |
| max_iter            | 否       | 训练迭代次数( 1 个batch为 1 个 iter) | 20                                               |
| print_every_iternum | 否       | 打印信息间隔 iter 数                 | 1                                                |
| save_folder         | 是       | 保存模型路径                         | ./weights/                                       |

**输出：**

在指定文件夹下输出训练好的 pth 模型文件，命名为 'yolact_resnet50_epoch数_iter数.pth'

例如：'yolact_resnet50_6_20.pth‘



## 4. PC端模型评估

#### 4.1 mAP评估

**进入 yolact_edge 目录,运行 eval.py 文件：**

```
# 命令行
python eval.py [--trained_model] [--valid_images] [--valid_info] [--class_names] [--save_folder]
# 示例
python eval.py --trained_model ./weights/yolact_resnet50_6_20.pth --valid_images data/coco_small_data/valid/images --valid_info data/coco_small_data/valid/val_annotation.json --class_names ('person','bear','car') --save_folder results
```

**参数说明：**

| 参数          | 是否必需 | 说明                   | 默认值                                                       |
| ------------- | -------- | ---------------------- | ------------------------------------------------------------ |
| trained_model | 是       | 训练得到的模型路径     | ./weights/yolact_resnet50_6_20.pth                           |
| valid_images  | 是       | 验证集文件夹路径       | data/coco_small_data/valid/images                            |
| valid_info    | 是       | 验证集的标签文件       | data/coco_small_data/valid/val_annotation.json               |
| class_names   | 是       | 验证集的类别，元组类型 | ('person','bear','car')  （注意和训练中的class_names保持一致） |
| save_folder   | 是       | 保存推理结果路径       | results                                                      |

**输出 mAP：**

![image-20220918220048896](C:\Users\HilaryChoi\AppData\Roaming\Typora\typora-user-images\image-20220918220048896.png)

#### 4.1 图片分割结果评估

**进入 yolact_edge 目录,运行 eval.py 文件：**

```
# 命令行
python eval.py [--trained_model] [--class_names] [--image] [--top_k] [--save_folder]
# 示例
python eval.py --trained_model ./weights/yolact_resnet50_6_20.pth --class_names ('person','bear','car') --image data/coco_small_data/train/images/000000186422.jpg:re.jpg --top_k 2 --save_folder results
```

**参数说明：**

| 参数          | 是否必需 | 说明                        | 默认值                                                       |
| ------------- | -------- | --------------------------- | ------------------------------------------------------------ |
| trained_model | 是       | 训练得到的模型路径          | ./weights/yolact_resnet50_6_20.pth                           |
| class_names   | 是       | 验证集的类别，元组类型      | ('person','bear','car')  （注意和训练中的class_names保持一致） |
| image         | 是       | 分割图片的路径:分割结果路径 | None                                                         |
| top_k         | 是       | 只查看得分前 top_k 的 bbox  | 2                                                            |
| save_folder   | 是       | 保存推理结果路径            | results                                                      |

**在指定路径保存图片分割结果：**



## 5. PC端 pth 转 onnx 

**进入 yolact_edge 目录,运行 pth2onnx.py 文件**

```
# 命令行
python pth2onnx.py [--trained_model] [--class_names] [--onnx_file]
# 示例
python pth2onnx.py --trained_model ./weights/yolact_resnet50_6_20.pth --class_names ('person','bear','car') --onnx_file yolact_edge.onnx
```

**参数说明：**

| 参数          | 是否必需 | 说明                   | 默认值                                                       |
| ------------- | -------- | ---------------------- | ------------------------------------------------------------ |
| trained_model | 是       | 训练得到的模型路径     | ./weights/yolact_resnet50_6_20.pth                           |
| class_names   | 是       | 验证集的类别，元组类型 | ('person','bear','car')  （注意和训练中的class_names保持一致） |
| onnx_file     | 是       | 输出的 onnx 模型文件名 | yolact_edge.onnx                                             |

**输出：**

生成 onnx_file 指定的 onnx模型文件



## 6. 200DK 端 onnx 转 om 

**将 onnx 模型传到200DK上，进入所在文件夹，并执行命令:**

```
# 命令行
atc [--model] --framework=5 [--output] --log=error --soc_version=Ascend310
# 示例
atc --model=yolact_edge.onnx --framework=5 --output=yolact_edge --log=error --soc_version=Ascend310
```

**参数说明：**

| 参数   | 是否必需 | 说明               | 默认值           |
| ------ | -------- | ------------------ | ---------------- |
| model  | 是       | 训练得到的模型路径 | yolact_edge.onnx |
| output | 是       | 输出的om模型名字   | yolact_edge      |

**输出：**

生成 output 所指定的 om 模型文件



## 7. 200DK 端推理

**将项目文件打包至200DK上，运行 eval_infer.py：**

```
# 命令行
python eval_infer.py [--trained_model] [--trained_om_model] [--class_names] [--valid_images] [--valid_info] 
# 示例
python eval_infer.py --trained_model weights/yolact_resnet50_6_20.pth --trained_om_model yolact_edge.om --class_names ('person','bear','car') --valid_images data/coco_small_data/valid/images --valid_info data/coco_small_data/valid/val_annotation.json
```

**参数说明：**

| 参数             | 是否必需 | 说明                   | 默认值                                                       |
| ---------------- | -------- | ---------------------- | ------------------------------------------------------------ |
| trained_model    | 是       | PC端训练得到的pth模型  | ./weights/yolact_resnet50_6_20.pth                           |
| trained_om_model | 是       | 200DK端转换后的om模型  | yolact_edge.om                                               |
| valid_images     | 是       | 验证集文件夹路径       | data/coco_small_data/valid/images                            |
| valid_info       | 是       | 验证集的标签文件       | data/coco_small_data/valid/val_annotation.json               |
| class_names      | 是       | 验证集的类别，元组类型 | ('person','bear','car')  （注意和训练中的class_names保持一致） |

**输出 mAP：**

![image-20220918230028055](C:\Users\HilaryChoi\AppData\Roaming\Typora\typora-user-images\image-20220918230028055.png)

