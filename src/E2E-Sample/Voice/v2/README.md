# 智能语音台灯样例运行指南

## 1 样例介绍
智能语音台灯应用通过将语音识别模型WeNet转换为OM模型，使其能够运行在Atlas 200I DK A2开发者套件上的昇腾AI处理器进行加速，主要的工作流程是在网页前端收集用户语音输入，通过后端放入消息队列，后端进行语音识别模型推理解析为文本，并转换为控制命令发送给台灯，从而实现台灯开关控制。交互界面以聊天窗口的形式呈现，用户通过语音输入命令后，聊天机器人会以文本的形式返回控制结果。

## 2 依赖文件
你可以从[这个链接](https://ascend-repo.obs.cn-east-2.myhuaweicloud.com/Atlas%20200I%20DK%20A2/DevKit/samples/23.0.RC1/e2e-samples/Voice/e2e_voice_lamp.zip)获取依赖的前端以及模型文件，并整理成如下结构：
```
├── config.py    
├── dist
│   ├── assets
│   │   ├── avtor-141cd8e9.jpg
│   │   ├── index-0c2196b0.css
│   │   ├── index-0f38e264.js
│   │   └── me-f369eebc.jpg
│   ├── index.html
│   └── vite.svg
├── main.py
└── wenet
    ├── model.py
    ├── offline_encoder.om
    └── vocab.txt
```

## 3 硬件连接以及样例运行
> 模型推理依赖 ais_bench 推理工具，在基础镜像中已安装。若环境中未安装该工具，请参照[对应的代码仓](https://gitee.com/ascend/tools/tree/master/ais-bench_workload/tool/ais_bench)说明进行安装。

请在开发者套件详情页点击[智能语音台灯应用开发指南](https://www.hiascend.com/document/detail/zh/Atlas200IDKA2DeveloperKit/23.0.RC1/Application%20Cases/svladg/svladg_0005.html)了解该部分详细内容。
