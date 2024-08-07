1. Training

Python train.py --num_class 4 --data_path ../sample_dataset

2. cpu推理

python infer.py 

--weights ./MobileNetV3_large.pth --data ./path/to/imgs --label_path class_indices.json --output_path ./output

Weights: pth文件

Data：图片文件夹

label_path： 训练生成的class_indices.json文件，用于后处理

output_path: 指定路径下，输出cls_output.txt文件

3. 打包

Python model_convert.py 

--weights ./MobileNetV3_large.pth --output_path ./output --data ./path/to/imgs --num_classes 4 

Weights: 模型路径

output_path：输出压缩包路径

num_classes：分类类别数

data：测试图片文件夹