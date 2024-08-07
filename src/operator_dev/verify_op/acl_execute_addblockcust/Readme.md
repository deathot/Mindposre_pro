# AddBlockCust算子运行验证

## 功能描述

该样例实现了对[AddBlockCust](../../custom_op/doc/AddBlockCust_CN.md)的功能验证，通过将自定义算子转换为单算子离线模型文件，然后通过AscendCL加载单算子模型文件进行运行。

## 环境要求

- 操作系统及架构：Ubuntu 22.04 aarch64
-   芯片：Ascend 310B1
-   python及依赖的库：Python3.7.*x*（3.7.0 ~ 3.7.11）、Python3.8.*x*（3.8.0 ~ 3.8.11）
-   已完成昇腾AI软件栈在开发环境、运行环境上的部署。
-   已参考[custom\_op](../../custom_op)完成自定义算子的编译部署。

## 配置环境变量

- 开发环境上环境变量配置

  1. CANN-Toolkit包提供进程级环境变量配置脚本，供用户在进程中引用，以自动完成CANN基础环境变量的配置，配置示例如下所示

     ```
     source /usr/local/Ascend/ascend-toolkit/set_env.sh
     ```

  2. 算子编译依赖Python,执行如下命令设置Python的相关环境变量。

     ```
     export LD_LIBRARY_PATH=/root/miniconda3/lib:$LD_LIBRARY_PATH
     ```

  3. 设置环境变量。

     设置以下环境变量后，编译脚本会根据“{DDK_PATH}环境变量值/runtime/include/acl”目录查找编译依赖的头文件，根据{NPU_HOST_LIB}环境变量指向的目录查找编译依赖的库文件。“$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。

       ```
       export DDK_PATH=/usr/local/Ascend/ascend-toolkit/latest
       export NPU_HOST_LIB=$DDK_PATH/runtime/lib64/stub
       ```

    - 运行set_env.sh

        ```
        source /usr/local/Ascend/ascend-toolkit/set_env.sh
        ```

## 编译运行

1.  生成AddBlockCust算子的单算子离线模型文件。
    1.  登录开发者套件，并进入样例工程的“acl\_execute\_addblockcust/run/out“目录。
    2.  在run/out目录下执行如下命令，生成单算子模型文件。

        **atc --singleop=test\_data/config/addblockcust_op.json  --soc\_version=*Ascend310B1*  --output=op\_models**

        其中：

        -   singleop：算子描述的json文件。
        -   soc\_version：昇腾AI处理器的型号，此处为Ascend310B1
        -   --output=op\_models：代表生成的模型文件存储在当前目录下的op\_models文件夹下。

        模型转换成功后，会生成如下文件：

        在当前目录的op\_models目录下生成单算子的模型文件**0\_AddBlockCust\_3\_2\_2\_2\_3\_2\_2\_2\_3\_2\_2\_2.om**，命名规范为：序号+opType + 输入的描述\(dateType\_format\_shape\)+输出的描述。


2.  生成测试数据。

    进入样例工程目录的run/out/test\_data/data目录下，执行如下命令：

    **python3 generate\_data.py**

    会在当前目录下生成第一个shape为\(2,2\)，第二个shape为\(2,2\)，数据类型均为int32的数据文件input\_0.bin与input\_1.bin，用于进行AddBlockCust算子的验证。

3. 编译样例工程，生成单算子验证可执行文件。
    1. 切换到样例工程根目录acl\_execute\_addblockcust，然后在样例工程根目录下执行如下命令创建目录用于存放编译文件，例如，创建的目录为“build/intermediates/host“。

       **mkdir -p build/intermediates/host**

    2. 切换到“build/intermediates/host”目录，执行cmake命令生成编译文件。

       -   执行如下命令编译。

           **cd build/intermediates/host**

           **cmake ../../../src -DCMAKE\_CXX\_COMPILER=g++ -DCMAKE\_SKIP\_RPATH=TRUE**


    3. 执行如下命令，生成可执行文件。

       **make**

       会在工程目录的“run/out“目录下生成可执行文件**execute\_custom\_add\_block\_cust\_op**。


4. 执行单算子验证文件。


    1.  执行execute\_custom\_add\_block\_cust\_op文件，验证单算子模型文件。

        在run\_addblockcust/out目录下执行如下命令：

        **chmod +x execute\_custom\_add\_block\_cust\_op**

        **./execute\_custom\_add\_block\_cust\_op**

        会有如下屏显信息（注意：由于数据生成脚本生成的数据文件是随机的，屏显显示的数据会有不同）：

        ```
        [INFO]  Open device[0] success
        [INFO]  file size is:16.
        [INFO]  Set input[0] from test_data/data/input_0.bin success.
        [INFO]  Input[0]:
                74        53        72        30
        [INFO]  Input[0] shape is:2 2
        [INFO]  file size is:16.
        [INFO]  Set input[1] from test_data/data/input_1.bin success.
        [INFO]  Input[1]:
                48        47        79        77
        [INFO]  Input[1] shape is:[2, 2]
        [INFO]  Copy input[0] success
        [INFO]  Copy input[1] success
        [INFO]  Create stream success
        [INFO]  Execute AddBlockCust success
        [INFO]  Synchronize stream success
        [INFO]  Copy output[0] success
        [INFO]  Output[0]:
                122       100       151       107
        [INFO]  Output[0] shape is:2 2
        [INFO]  Write output[0] success. output file = result_files/output_0.bin
        [INFO]  Run op success
        ```

        可见：输出数据等于两个输入数据对应位置的数值之和，AddBlockCust算子验证结果正确。

        result\_files/output\_0.bin：输出数据的二进制文件。

