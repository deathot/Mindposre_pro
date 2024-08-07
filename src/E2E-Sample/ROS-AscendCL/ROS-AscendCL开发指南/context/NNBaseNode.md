# NNBaseNode<a name="ZH-CN_TOPIC_0000001538772190"></a>

## 函数功能<a name="section15868409121"></a>

基类NNBaseNode的构造函数，负责初始化两个成员变量modelPara和aclNodeImpl。

-   modelPara是模型参数类DnnModelPara对象的指针。
-   aclNodeImpl是AscendCL接口实现类AclInterfaceImpl对象的指针。

## 约束说明<a name="section1771773225914"></a>

一般与基类的析构函数[\~NNBaseNode](NNBaseNode-3.md)配套使用，先进行类资源初始化，最后释放类资源（去初始化）。

## 函数原型<a name="section16481811131215"></a>

**NNBaseNode\(std::string nodeName\)**

## 参数说明<a name="section2779823101219"></a>

<a name="zh-cn_topic_0122830089_table438764393513"></a>
<table><thead align="left"><tr id="zh-cn_topic_0122830089_row53871743113510"><th class="cellrowborder" valign="top" width="29.03%" id="mcps1.1.4.1.1"><p id="zh-cn_topic_0122830089_p1438834363520"><a name="zh-cn_topic_0122830089_p1438834363520"></a><a name="zh-cn_topic_0122830089_p1438834363520"></a>参数名</p>
</th>
<th class="cellrowborder" valign="top" width="24.529999999999998%" id="mcps1.1.4.1.2"><p id="p1769255516412"><a name="p1769255516412"></a><a name="p1769255516412"></a>输入/输出</p>
</th>
<th class="cellrowborder" valign="top" width="46.44%" id="mcps1.1.4.1.3"><p id="zh-cn_topic_0122830089_p173881843143514"><a name="zh-cn_topic_0122830089_p173881843143514"></a><a name="zh-cn_topic_0122830089_p173881843143514"></a>说明</p>
</th>
</tr>
</thead>
<tbody><tr id="zh-cn_topic_0122830089_row2038874343514"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="p1591413302166"><a name="p1591413302166"></a><a name="p1591413302166"></a>nodeName</p>
</td>
<td class="cellrowborder" valign="top" width="24.529999999999998%" headers="mcps1.1.4.1.2 "><p id="p8693185517417"><a name="p8693185517417"></a><a name="p8693185517417"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.44%" headers="mcps1.1.4.1.3 "><p id="zh-cn_topic_0122830089_p19388143103518"><a name="zh-cn_topic_0122830089_p19388143103518"></a><a name="zh-cn_topic_0122830089_p19388143103518"></a>节点源文件中实例化派生类时定义的节点名，类型是std::string。</p>
</td>
</tr>
</tbody>
</table>

## 返回值说明<a name="section7624143271217"></a>

无

