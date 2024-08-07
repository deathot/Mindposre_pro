# 模型转换

通过这个指南，你可以将原始的WeNet模型从PyTorch转换为ONNX，然后再转换为OM，以便于在开发者套件的昇腾AI处理器上实现推理加速。
> 我们在[这个链接](https://ascend-repo.obs.cn-east-2.myhuaweicloud.com/Atlas%20200I%20DK%20A2/DevKit/samples/23.0.RC1/base-samples/notebook-demo-datasets/10-speech-recognition/offline_encoder_sim.onnx)准备好了ONNX模型文件，你可以选择跳过
> 第一步和第二步，直接从第三步开始。
## 1 准备工作
1. 拉取WeNet原始代码仓
    ``` sh
     git clone https://github.com/wenet-e2e/wenet.git
     cd wenet
     git checkout v2.0.1
     cd ..
    ```

2. 创建虚拟环境
    ``` sh
    conda create -n wenet python=3.7.5
    conda activate wenet
    ```

3. 安装依赖
    ```sh
    pip install -r requirements.txt
    ```

4. 下载模型权重和配置文件。打开[链接](https://github.com/wenet-e2e/wenet/blob/main/docs/pretrained_models.md)，
下载multi_cn数据集对应的checkpoint文件，解压后将其放到wenet目录下。
   * p.s. 下载wenetspeech数据集对应的模型也是可以的，但是导出的模型大小会相对大一些。

## 2 导出 ONNX 模型
1. 将模型导出脚本放到wenet目录下
    ``` sh
    cp export_encoder_onnx.py wenet/
    cd wenet
    ```
2. 参照以下命令，修改参数路径，将模型转换为ONNX格式。
    ``` sh
    python .\export_encoder_onnx.py --config .\20210815_unified_conformer_exp\train.yaml --checkpoint .\20210815_unified_conformer_exp\final.pt --cmvn_file .\20210815_unified_conformer_exp\global_cmvn
    ```
3. 执行常量折叠。
    > 常量折叠（constant folding）是指在编译器或解释器层面对程序中的表达式进行求值和简化，将其中包含的常量表达式计算为一个固定的常量值。常量折叠可以减少程序的运行时计算量，提高执行效率。
    ``` sh
    onnxsim offline_encoder.onnx offline_encoder_sim.onnx
    ```

## 3 导出OM模型
* **建议在Linux服务器或者虚拟机转换该模型**
* 为了能进一步优化模型推理性能，我们需要将其转换为om模型进行使用，以下为转换指令：  
    ```shell
    atc --model=offline_encoder_sim.onnx --framework=5 --output=offline_encoder --input_format=ND --input_shape="speech:1,1478,80;speech_lengths:1" --log=error --soc_version=Ascend310B1
    ```
    * 其中转换参数的含义为：  
        * --model：输入模型路径
        * --framework：原始网络模型框架类型，5表示ONNX
        * --output：输出模型路径
        * --input_format：输入Tensor的内存排列方式
        * --input_shape：指定模型输入数据的shape。这里我们在转模型的时候指定了最大的输入音频长度，推荐的长度有：262,326,390,454,518,582,646,710,774,838,902,966,1028,1284,1478，默认使用的长度是1478
        * --log：日志级别
        * --soc_version：昇腾AI处理器型号
