# ChatBot多用户Demo

## 概述

本Demo实现了多用户聊天机器人，每个用户可以独立进行聊天，不会相互影响。次Demo基于ChatYuan-Large系列模型，模型转换流程详见ModelConvert目录。前端相关资源生成详见chatjs目录。

前端方面，我们使用了Vue.js框架，后端方面，我们使用了Flask框架。当用户打开前端页面时，前端会生成一个随机的UUID，后端会根据这个UUID记录对应的聊天记录，直到用户关闭页面。当用户关闭页面后，后端会根据UUID删除对应的聊天记录。前端的所有请求都会发送到后端，后端会根据UUID判断是哪个用户的请求，并返回对应的结果。

## 使用说明

**本Demo仅为可行性验证Demo，由于Atalas 200I DK A2上资源有限，为避免OOM，请在打开多页面的情况下，同一时间只有一个页面在进行流式推理输出。（即页面B需要等待页面A完成推理后再发消息）**

## 使用步骤

1. 如需完整体验本Demo，需要在windows上安装node.js并编译前端相关资源，详见chatjs目录。(此步骤非必须，如不进行此步骤，将使用预编译的前端资源)
2. 运行Demo请将demo目录下的所有文件拷贝至Atlas 200I DK A2上。并按照README.md的步骤进行操作。
