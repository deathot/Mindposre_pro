# 模型导出流程

## 1 功能与原理介绍
为了能够在4G内存下运行大型语言模型，我们需要把在线推理的模型权重转化为离线推理模型。
在转化为ONNX离线推理模型之后，我们会对模型进行剪裁，并对decoder与lm_head部分进行int8量化

**注意： 以下操作请在个人PC上完成，请勿在开发者套件上进行以下步骤**

## 2 运行步骤

### 2.1 环境准备

请按照requirements.txt准备环境，建议使用conda新建虚拟环境来管理环境与依赖

```shell
pip install -r ./requirements.txt
```

### 2.2 导出模型
**注意：如需体验ChatYuan-Large-V1，请按照2.2.1导出相关模型并跳过2.2.2步骤**

**如需体验ChatYuan-Large-V2，请按照2.2.2导出相关模型并跳过2.2.1步骤**

#### 2.2.1 导出CHatYuan-Large-V1模型


导出encoder部分模型
```shell
python export_encoder.py --version=v1
```
导出第一次自回归的decoder与lm_head部分模型
```shell
python export_decoder_first.py --version=v1
```

导出第2-N次自回归的decoder与lm_head部分模型
```shell
python export_decoder_iter.py --version=v1
```

#### 2.2.2 导出CHatYuan-Large-V2模型


导出encoder部分模型
```shell
python export_encoder.py --version=v2
```
导出第一次自回归的decoder与lm_head部分模型
```shell
python export_decoder_first.py --version=v2
```

导出第2-N次自回归的decoder与lm_head部分模型
```shell
python export_decoder_iter.py --version=v2
```


### 2.2 模型剪裁
这里我们可以使用onnxsim工具来剪裁一些不必要的算子。onnxsim运行需要较长时间，请耐心等待。


剪裁encoder部分模型
```shell
onnxsim ./encoder.onnx ./encoder_sim.onnx
```

剪裁第一次自回归的decoder与lm_head部分模型
```shell
onnxsim ./decoder_first.onnx ./decoder_first_sim.onnx
```
剪裁第2-N次自回归的decoder与lm_head部分模型
```shell
onnxsim ./decoder_iter.onnx ./decoder_iter_sim.onnx
```

### 2.3 对decoder部分模型进行int8量化 【在4G内存设备上运行此Demo必须执行此步骤】
**如需在4G版本设备上运行请执行PTQ量化步骤对模型进行int8量化**
请使用以下命令进行量化
```shell
python quant_decoder.py
```

### 2.4 使用ATC工具将encoder部分模型转换为混合精度的om模型
**注意： 为防止OOM问题，请在内存大于10G的设备上执行此命令。**

**注意：在Atlas 200I DK A2上执行此命令时，请确保swap分区大于8G**

```shell
atc --model=encoder_sim.onnx \
--framework=5 \
--soc_version=Ascend310B1 \
--output=encoder \
--input_format=ND \
--input_shape="input_ids:1,768;attention_mask:1,768" \
--precision_mode=allow_fp32_to_fp16 
```

