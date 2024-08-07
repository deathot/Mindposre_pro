# Yolov5端到端流程—v1.0

## 1.    数据集制做

### 1.1 Labelme标注

启动labelme，选择图片文件夹，开始标注检测目标；

Lebelme输出示例在sample_data文件夹下，包括图片和标注信息json；

![输入图片说明](https://foruda.gitee.com/images/1661763074629000199/8ce3fadb_11420827.png "屏幕截图")

### 1.2数据集转换脚本

启动命令：python labelme2yolov5.py --input_path= ../sample_data

必选参数：input_path 输入图片文件夹

可选参数：

output_path：输出文件路径

split_ratio：训练、验证划分比例

split_test：是否有测试集。

上述4个参数都需要支持用户可配

**输出**：

![输入图片说明](https://foruda.gitee.com/images/1661907453543086092/63003c46_11420827.png "屏幕截图")
## 2.  模型训练

命令行：

python train.py --img 640 --batch 12 --epochs 50 --data ../make_dataset/output/data.yaml --weights ./weights/yolov5s.pt

 

需要GUI支持用户修改：

--data 数据集转换脚本生成的data.yaml文件地址

--epochs 训练的迭代次数

--batch 每次训练加载的图片数

 

模型训练输出目录. \train_code\runs\exp

 

 

模型评估pending

 

Pt模型推理

![输入图片说明](https://foruda.gitee.com/images/1661907529304331801/6b6cf288_11420827.png "屏幕截图")


## 3.  模型转换

3.1-3.4分别是pt转onnx，onnx简化与算子修改具体脚本。如不想了解详细转模型流程，直接操作3.5即可。

### 3.1 pt转onnx

python ./models/export.py 

--weights E:\code\algorithm\yolov5\yolov5_e2e\train_code\runs\exp1\weights\best.pt

 

Weights参数用户可配；

 

Weights路径下输出onnx
### 3.2 onnx模型使用onnxsim简化(用户不可见)

环境安装：Pip install onnxsim==0.4.6

 

命令行：

Python –m onnxsim {3.1生成的onnx} {简化后的onnx} --skip-optimization

 

示例：

Python -m onnxsim E:\code\algorithm\yolov5\yolov5_e2e\train_code\runs\exp1\weights\best.onnx yolov5s_smi.onnx --skip-optimization
 

### 3.3 onnx算子修改（用户不可见）

当前对slice算子支持不足，通过modify_yolov5.py修改；

命令行：

Python modify_yolov5.py yolov5s.onnx


输出：yolov5s_smi_t.onnx

### 3.4模型打包

把在200dk上运行需要的脚本和模型打包。

python make_edge_infer.py --weights 

E:\code\algorithm\yolov5\yolov5_e2e\train_code\runs\exp1\weights\best_sim_t.onnx --output_path ./ --data 

E:\code\algorithm\yolov5\yolov5_e2e\make_dataset\output\train\images

可配置参数：

weight：onnx模型路径

output: 输出压缩包存放路径

data： 推理图片路径

 

### 3.5 自动化实现3.1-3.4

model_convert.py就是自动化了3.1-3.4的手动步骤。

输出：压缩包内容包括onnx模型，200dk推理脚本、atc转换脚本、数据集、acl公共组件。

启动命令：

Python model_convert.py –weights xxx.pb –output_path 输出rar包文件夹路径 –data 验证数据集路径
![输入图片说明](https://foruda.gitee.com/images/1661907485203251255/1c6a9863_11420827.png "屏幕截图")

 

需要支持用户可配：

Weights路径：xxx.pt

Output_path：压缩包输出文件夹

Data:200dk上验证用的图片

 

## 4. 200DK验证

（1）把压缩包上传到200DK，解压

（2）进入解压缩目录执行atc.sh,得到om模型

（3）执行推理

 

Python v5_object_detect.py –model yolov5s_bs1.om –class_num 2 –iou_threshold 0.6 –conf_threshold 0.5 –output_path ./out

 

命令行参数：

Model om模型地址

Class_num: 检测类别数；

iou_threshold: 重叠度

conf_threshold： 置信度

output_path: 推理结果输出地址

 