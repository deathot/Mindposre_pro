# SetModelPara（基类）<a name="ZH-CN_TOPIC_0000001589292437"></a>

## 函数功能<a name="section15868409121"></a>

基类NNBaseNode中实现模型推理参数（如神经网络模型路径、模型名字）设置的函数。

通过对该函数赋值为0，使其成为一个纯虚函数，其接口实现在派生类NNObjectDetectNode中实现.。

## 约束说明<a name="section1771773225914"></a>

该纯虚函数的具体实现请参见派生类NNObjectDetectNode中的[SetModelPara（派生类）](SetModelPara（派生类）.md)。

## 函数原型<a name="section16481811131215"></a>

**virtual Result SetModelPara\(\) = 0**

## 参数说明<a name="section2779823101219"></a>

无

## 返回值说明<a name="section7624143271217"></a>

返回Result类型错误码：

-   SUCCESS: 执行成功
-   FAILED：执行失败

