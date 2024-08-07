# ReadPicFile<a name="ZH-CN_TOPIC_0000001594605837"></a>

## 函数功能<a name="section15868409121"></a>

Utils类中实现从本地获取图片文件的函数。

## 约束说明<a name="section1771773225914"></a>

无

## 函数原型<a name="section16481811131215"></a>

**Result ReadPicFile\(ImageData& inputPic, const char inputFileName\[\]\)**

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
<tbody><tr id="zh-cn_topic_0122830089_row2038874343514"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="li868514597308p0"><a name="li868514597308p0"></a><a name="li868514597308p0"></a>inputPic</p>
</td>
<td class="cellrowborder" valign="top" width="24.51%" headers="mcps1.1.4.1.2 "><p id="p8693185517417"><a name="p8693185517417"></a><a name="p8693185517417"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.46%" headers="mcps1.1.4.1.3 "><p id="li15204128184012p0"><a name="li15204128184012p0"></a><a name="li15204128184012p0"></a>待读取的yuv图片数据结构体，类型是ImageData&amp;。</p>
</td>
</tr>
<tr id="row162148265488"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="p59315193221"><a name="p59315193221"></a><a name="p59315193221"></a>inputFileName[]</p>
</td>
<td class="cellrowborder" valign="top" width="24.51%" headers="mcps1.1.4.1.2 "><p id="p8483123822216"><a name="p8483123822216"></a><a name="p8483123822216"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.46%" headers="mcps1.1.4.1.3 "><p id="li37681919113817p0"><a name="li37681919113817p0"></a><a name="li37681919113817p0"></a>待读取的图片文件名，类型是const char。</p>
</td>
</tr>
</tbody>
</table>

## 返回值说明<a name="section7624143271217"></a>

返回Result类型错误码：

-   SUCCESS: 执行成功
-   FAILED：执行失败

