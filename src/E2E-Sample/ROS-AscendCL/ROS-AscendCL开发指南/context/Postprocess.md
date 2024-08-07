# Postprocess<a name="ZH-CN_TOPIC_0000001589784305"></a>

## 函数功能<a name="section15868409121"></a>

派生类NNObjectDetectNode中对模型输出进行后处理的函数。

## 约束说明<a name="section1771773225914"></a>

无

## 函数原型<a name="section16481811131215"></a>

**Result Postprocess\(std::vector<DataInfo\> outputData, const sensor\_msgs::msg::Image::ConstSharedPtr imgMsg\)**

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
<tbody><tr id="zh-cn_topic_0122830089_row2038874343514"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="li868514597308p0"><a name="li868514597308p0"></a><a name="li868514597308p0"></a>outputData</p>
</td>
<td class="cellrowborder" valign="top" width="24.51%" headers="mcps1.1.4.1.2 "><p id="p8693185517417"><a name="p8693185517417"></a><a name="p8693185517417"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.46%" headers="mcps1.1.4.1.3 "><p id="p337494534810"><a name="p337494534810"></a><a name="p337494534810"></a>模型推理输出的数据，类型是std::vector&lt;DataInfo&gt;。</p>
</td>
</tr>
<tr id="row162148265488"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="p321518264481"><a name="p321518264481"></a><a name="p321518264481"></a>imgMsg</p>
</td>
<td class="cellrowborder" valign="top" width="24.51%" headers="mcps1.1.4.1.2 "><p id="p92156262487"><a name="p92156262487"></a><a name="p92156262487"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.46%" headers="mcps1.1.4.1.3 "><p id="li11519122313618p0"><a name="li11519122313618p0"></a><a name="li11519122313618p0"></a>ROS2图像消息，类型是sensor_msgs::msg::Image::ConstSharedPtr。</p>
</td>
</tr>
</tbody>
</table>

## 返回值说明<a name="section7624143271217"></a>

返回Result类型错误码：

-   SUCCESS: 执行成功
-   FAILED：执行失败

