# DestroyModelResource<a name="ZH-CN_TOPIC_0000001589784301"></a>

## 函数功能<a name="section15868409121"></a>

AscendCL接口实现类AclInterfaceImpl中实现模型资源释放的函数，包括释放模型描述、输入、输出信息等。

## 约束说明<a name="section1771773225914"></a>

在程序退出前，应显式调用该接口实现模型资源释放，否则会导致运行异常。

## 函数原型<a name="section16481811131215"></a>

**Result DestroyModelResource\(\)**

## 参数说明<a name="section2779823101219"></a>

无

## 返回值说明<a name="section7624143271217"></a>

返回Result类型错误码：

-   SUCCESS: 执行成功
-   FAILED：执行失败

