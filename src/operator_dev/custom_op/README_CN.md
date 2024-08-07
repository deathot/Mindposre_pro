# README

## 概述

本样例包含了TBE自定义算子、AI CPU自定义算子，同时提供了对应的编译规则文件，开发者可以直接基于本样例追加自己的自定义算子实现代码，然后进行工程的编译即可获得自定义算子安装包。


## 样例介绍

-   TBE自定义算子样例
    -   ScatterNdAdd算子，请参见[ScatterNdAdd](doc/ScatterNdAdd_CN.md)。
    -   Permute算子，请参见[Permute](doc/Permute_CN.md)。
    -   Upsample算子，请参见[Upsample](doc/Upsample_CN.md)。

-   AI CPU自定义算子样例
    -   UniqueCust算子，请参见[Unique](doc/Unique_CN.md)。
    -   AddBlockCust算子，此算子支持分块并行计算，请参见[AddBlockCust](doc/AddBlockCust_CN.md)。

## 环境要求

-   操作系统及架构：Ubuntu 22.04 aarch64
-   python及依赖的库：Python3.7.x（3.7.0 ~ 3.7.11）、Python3.8.x（3.8.0 ~ 3.8.11）、Python 3.9.*x*
-   已完成昇腾AI软件栈的部署。

## 算子工程编译
算子编译依赖Python，请以运行用户执行如下命令设置Python的相关环境变量。
   
      ```
      export LD_LIBRARY_PATH=/root/miniconda3/lib:$LD_LIBRARY_PATH
      ```
    Python安装路径请根据实际情况进行替换.

1. 修改build.sh脚本，根据实际开发环境信息修改相关环境变量配置。

   修改buid.sh脚本头部的如下环境变量。

   - ASCEND\_TENSOR\_COMPILER\_INCLUDE：CANN软件头文件所在路径。

     请取消此环境变量的注释，并修改为CANN软件头文件所在路径，：

     ```
     export ASCEND_TENSOR_COMPILER_INCLUDE=/usr/local/Ascend/ascend-toolkit/latest/compiler/include
     ```

   -   TOOLCHAIN\_DIR：AI CPU算子使用的编译器路径，请取消此环境变量的注释，并按照下述描述修改。
          ```
          export TOOLCHAIN_DIR=/usr/local/Ascend/ascend-toolkit/latest/toolkit/toolchain/hcc
          ```

   -    AICPU\_SOC\_VERSION：昇腾AI处理器的类型，请配置为Ascend310B1


3.  执行算子工程编译。

    - 若您只需要编译TBE算子，请在算子工程目录下执行如下命令。

      **chmod +x build.sh**

      **./build.sh -t**


    - 若您只需要编译AI CPU算子，请在算子工程目录下执行如下命令。

      **chmod +x build.sh**

      **./build.sh -c**

    - 若您既需要编译TBE算子，又需要编译AI CPU算子，请在算子工程目录下执行如下命令。

      **chmod +x build.sh**

      **./build.sh**

    编译成功后，会在当前目录下创建build\_out目录，并在build\_out目录下生成自定义算子安装包**custom\_opp\__<target os\>\_<target architecture\>_.run**。
    
    **说明：**

    -  若重新进行工程编译，请先执行./build.sh clean命令进行编译文件的清理。
    -  若您开发的自定义算子既包含TBE算子，又包含AI CPU算子，请选择同时编译，生成一个自定义算子安装包。因为当前版本，仅支持安装一个自定义算子安装包，后面安装的自定义算子包会覆盖之前安装的算子包。



## 算子部署

在编译生成的自定义算子安装包所在路径下，执行如下命令，安装自定义算子包。

  **./custom\_opp\__<target os\>\_<target architecture\>_.run**
成功部署后，屏幕会显示Successfully字样。

## 目录结构

```
├── CMakeLists.txt //算子工程的CMakeList.txt
├── README.md       
├── custom.proto    // 原始框架为Caffe的自定义算子的proto定义文件    
├── build.sh       //  工程编译入口脚本 
├── cpukernel      // AI CPU算子实现文件及信息库文件所在目录
│   ├── CMakeLists.txt
│   ├── impl    //算子实现文件目录
│   │      ├── xx.cc
│   │      ├── xx.h
│   ├── op_info_cfg   //算子信息库文件目录
│   │      ├── aicpu_kernel
│                ├── xx.ini     //算子信息库文件
├── op_proto     //算子原型定义文件及CMakeList文件所在目录   
│   ├── xx.h
│   ├── xx.cc
│   ├── CMakeLists.txt   //算子IR定义文件的CMakeList.txt，会被算子工程的CMakeList.txt调用
├── tbe 
│   ├── CmakeLists.txt   
│   ├── impl    //算子实现文件目录
│   │      ├── xx.py
│   │      ├── __init__.py      //Python中的package标识文件
│   ├── op_info_cfg   //算子信息库文件目录
│       └── ai_core
│           ├── ${Soc Version}           //昇腾AI处理器类型
│               ├── xx.ini
├── cmake 
│   ├── config.cmake
│   ├── util
│       ├── makeself       //编译相关公共文件存放目录
│       ├── parse_ini_to_json.py       // 将算子信息定义.ini文件转换为信息库json文件的脚本
│       ├── gen_ops_filter.sh          // 用于生成记录支持的TensorFlow的NPU算子文件
├── scripts     //自定义算子工程打包相关脚本
├── tools
```