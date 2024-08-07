# CvtRosMsgToYuv420sp<a name="ZH-CN_TOPIC_0000001589908993"></a>

## 函数功能<a name="section15868409121"></a>

Utils类中实现图像格式转换的函数，将接收到的ROS2图像消息转换为yuv420sp格式。

## 约束说明<a name="section1771773225914"></a>

无

## 函数原型<a name="section16481811131215"></a>

**Result CvtRosMsgToYuv420sp\(const sensor\_msgs::msg::Image::ConstSharedPtr imgMsg, ImageData& yuvImage, const std::string encoding\)**

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
<tbody><tr id="zh-cn_topic_0122830089_row2038874343514"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="li10509423123915p0"><a name="li10509423123915p0"></a><a name="li10509423123915p0"></a>imgMsg</p>
</td>
<td class="cellrowborder" valign="top" width="24.51%" headers="mcps1.1.4.1.2 "><p id="p8693185517417"><a name="p8693185517417"></a><a name="p8693185517417"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.46%" headers="mcps1.1.4.1.3 "><p id="p62091682371"><a name="p62091682371"></a><a name="p62091682371"></a>ROS2图像消息，类型是sensor_msgs::msg::Image::ConstSharedPtr。</p>
</td>
</tr>
<tr id="row162148265488"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="p59315193221"><a name="p59315193221"></a><a name="p59315193221"></a>yuvImage</p>
</td>
<td class="cellrowborder" valign="top" width="24.51%" headers="mcps1.1.4.1.2 "><p id="p8483123822216"><a name="p8483123822216"></a><a name="p8483123822216"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.46%" headers="mcps1.1.4.1.3 "><p id="li37681919113817p0"><a name="li37681919113817p0"></a><a name="li37681919113817p0"></a>转换后的yuv420sp图像格式，类型是ImageData&amp;。</p>
</td>
</tr>
<tr id="row1873692352217"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="p13736142312222"><a name="p13736142312222"></a><a name="p13736142312222"></a>encoding</p>
</td>
<td class="cellrowborder" valign="top" width="24.51%" headers="mcps1.1.4.1.2 "><p id="p187361923162219"><a name="p187361923162219"></a><a name="p187361923162219"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.46%" headers="mcps1.1.4.1.3 "><p id="li577152103813p0"><a name="li577152103813p0"></a><a name="li577152103813p0"></a>缩放算法，类型是int32_t。</p>
</td>
</tr>
</tbody>
</table>

## 返回值说明<a name="section7624143271217"></a>

返回Result类型错误码：

-   SUCCESS: 执行成功
-   FAILED：执行失败

