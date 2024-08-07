# DestroyAclResource<a name="ZH-CN_TOPIC_0000001538865272"></a>

## 函数功能<a name="section15868409121"></a>

AscendCL接口实现类AclInterfaceImpl中定义AscendCL去初始化的函数，用于释放进程内AscendCL相关资源。

## 约束说明<a name="section1771773225914"></a>

-   在程序退出前，应显式调用该接口实现AscendCL去初始化，否则会导致运行异常。
-   一般与AclInterfaceImpl类中AscendCL初始化函数[InitAclResource](InitAclResource.md)配套使用。

## 函数原型<a name="section16481811131215"></a>

**Result DestroyAclResource\(\)**

## 参数说明<a name="section2779823101219"></a>

无

## 返回值说明<a name="section7624143271217"></a>

返回Result类型错误码：

-   SUCCESS: 执行成功
-   FAILED：执行失败

