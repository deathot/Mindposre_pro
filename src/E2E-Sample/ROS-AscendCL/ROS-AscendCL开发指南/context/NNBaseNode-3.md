# \~NNBaseNode<a name="ZH-CN_TOPIC_0000001538853932"></a>

## 函数功能<a name="section15868409121"></a>

基类NNBaseNode的析构函数，定义了类资源的释放（去初始化）流程，例如modelPara和aclNodeImpl成员变量的销毁等。

## 约束说明<a name="section1771773225914"></a>

一般与基类的构造函数[NNBaseNode](NNBaseNode.md)配套使用，先进行类资源初始化，最后释放类资源（去初始化）。

## 函数原型<a name="section16481811131215"></a>

**\~NNBaseNode\(\)**

## 参数说明<a name="section2779823101219"></a>

无

## 返回值说明<a name="section7624143271217"></a>

无

