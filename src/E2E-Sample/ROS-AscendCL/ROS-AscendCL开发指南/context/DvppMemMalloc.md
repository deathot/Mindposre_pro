# DvppMemMalloc<a name="ZH-CN_TOPIC_0000001594500545"></a>

## 函数功能<a name="section15868409121"></a>

Utils类中申请DVPP动态内存的函数。

## 约束说明<a name="section1771773225914"></a>

一般与Utils类中内存释放函数[DvppMemFree](DvppMemFree.md)配套使用。

## 函数原型<a name="section16481811131215"></a>

**Result DvppMemMalloc\(void\*\* addrPtr, unsigned int bufSize\)**

## 参数说明<a name="section2779823101219"></a>

<a name="zh-cn_topic_0122830089_table438764393513"></a>
<table><thead align="left"><tr id="zh-cn_topic_0122830089_row53871743113510"><th class="cellrowborder" valign="top" width="29.03%" id="mcps1.1.4.1.1"><p id="zh-cn_topic_0122830089_p1438834363520"><a name="zh-cn_topic_0122830089_p1438834363520"></a><a name="zh-cn_topic_0122830089_p1438834363520"></a>参数名</p>
</th>
<th class="cellrowborder" valign="top" width="24.51%" id="mcps1.1.4.1.2"><p id="p1769255516412"><a name="p1769255516412"></a><a name="p1769255516412"></a>输入/输出</p>
</th>
<th class="cellrowborder" valign="top" width="46.46%" id="mcps1.1.4.1.3"><p id="zh-cn_topic_0122830089_p173881843143514"><a name="zh-cn_topic_0122830089_p173881843143514"></a><a name="zh-cn_topic_0122830089_p173881843143514"></a>说明</p>
</th>
</tr>
</thead>
<tbody><tr id="zh-cn_topic_0122830089_row2038874343514"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="li19911656173919p0"><a name="li19911656173919p0"></a><a name="li19911656173919p0"></a>addrPtr</p>
</td>
<td class="cellrowborder" valign="top" width="24.51%" headers="mcps1.1.4.1.2 "><p id="p8693185517417"><a name="p8693185517417"></a><a name="p8693185517417"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.46%" headers="mcps1.1.4.1.3 "><p id="p337494534810"><a name="p337494534810"></a><a name="p337494534810"></a>用于申请DVPP动态存储空间的指针，类型是void**。</p>
</td>
</tr>
<tr id="row162148265488"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="p59315193221"><a name="p59315193221"></a><a name="p59315193221"></a>bufSize</p>
</td>
<td class="cellrowborder" valign="top" width="24.51%" headers="mcps1.1.4.1.2 "><p id="p8483123822216"><a name="p8483123822216"></a><a name="p8483123822216"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.46%" headers="mcps1.1.4.1.3 "><p id="li1012125919399p0"><a name="li1012125919399p0"></a><a name="li1012125919399p0"></a>申请的动态存储空间大小，类型是unsigned int。</p>
</td>
</tr>
</tbody>
</table>

## 返回值说明<a name="section7624143271217"></a>

返回Result类型错误码：

-   SUCCESS: 执行成功
-   FAILED：执行失败

## 参考资源<a name="section15483143213619"></a>

-   DVPP V2底层逻辑接口调用流程，可参考《应用软件开发指南（C&C++）》中“媒体数据处理V2 \> VPC图片处理典型功能 \> 接口调用流程”章节。
-   DVPP V2缩放功能的接口调用示例，可参考《应用软件开发指南（C&C++）》中“媒体数据处理V2 \> VPC图片处理典型功能 \>缩放”示例。

