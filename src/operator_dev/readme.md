
# 自定义算子样例和开发指南



## 目录结构与说明
  

| 目录  | 说明  |
|---|---|
| [custom_op](./custom_op/)  | 5个自定义算子项目源代码，分别是permute,unique_cust, upsample,  add_block_cust, scatter_nd_add|
|[verify_op](./verify_op/)   | 自定义算子的ST测试样例（以add_block_cust算子为例）  |
|[MS_project](./MS_project/) | 通过Mindstudio方式开发自定义算子指南（以AddDsl算子为例） |

## 样例使用方法
custom_op和verify_op目录下的项目文件均需要上传到A200I DK A2上运行。custom_op目录下保存有命令行方式自定义算子样例的完整项目，用户可以按照Readme的说明进行编译和部署，从而将自定义算子安装到CANN的算子库中，用户还可以查看算子样例的源代码来学习如何实现自定义算子。部署完成后用户参考verify_op目录下的文件进行ST测试以验证算子是否正确实现，并可作为开发其他算子ST测试时的参考。  
算子开发和ST测试也可以使用Mindstudio进行，MS_project有使用Mindstudio开发自定义算子和进行ST测试的文档说明。此目录可以在Windows版本的Mindstudio中打开。
