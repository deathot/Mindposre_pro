# ConfigureStrideAndBufferSize<a name="ZH-CN_TOPIC_0000001543860746"></a>

## 函数功能<a name="section15868409121"></a>

Utils类中用于配置图像内存大小的函数。

## 约束说明<a name="section1771773225914"></a>

无

## 函数原型<a name="section16481811131215"></a>

**uint32\_t ConfigureStrideAndBufferSize\(hi\_vpc\_pic\_info& pic, uint32\_t widthAlign, uint32\_t heightAlign, bool widthStride32Align\)**

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
<tbody><tr id="zh-cn_topic_0122830089_row2038874343514"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="li868514597308p0"><a name="li868514597308p0"></a><a name="li868514597308p0"></a>pic</p>
</td>
<td class="cellrowborder" valign="top" width="24.51%" headers="mcps1.1.4.1.2 "><p id="p8693185517417"><a name="p8693185517417"></a><a name="p8693185517417"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.46%" headers="mcps1.1.4.1.3 "><p id="li12429154414391p0"><a name="li12429154414391p0"></a><a name="li12429154414391p0"></a>待处理的yuv图像，类型是hi_vpc_pic_info&amp;。</p>
</td>
</tr>
<tr id="row162148265488"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="p59315193221"><a name="p59315193221"></a><a name="p59315193221"></a>widthAlign</p>
</td>
<td class="cellrowborder" valign="top" width="24.51%" headers="mcps1.1.4.1.2 "><p id="p8483123822216"><a name="p8483123822216"></a><a name="p8483123822216"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.46%" headers="mcps1.1.4.1.3 "><p id="li37681919113817p0"><a name="li37681919113817p0"></a><a name="li37681919113817p0"></a>图像宽度要对齐的字节数，类型是uint32_t。</p>
</td>
</tr>
<tr id="row1873692352217"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="li1945773254312p0"><a name="li1945773254312p0"></a><a name="li1945773254312p0"></a>heightAlign</p>
</td>
<td class="cellrowborder" valign="top" width="24.51%" headers="mcps1.1.4.1.2 "><p id="p187361923162219"><a name="p187361923162219"></a><a name="p187361923162219"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.46%" headers="mcps1.1.4.1.3 "><p id="p226716207451"><a name="p226716207451"></a><a name="p226716207451"></a>图像高度要对齐的字节数，类型是uint32_t。</p>
</td>
</tr>
<tr id="row911212501449"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="p10113125034419"><a name="p10113125034419"></a><a name="p10113125034419"></a>widthStride32Align</p>
</td>
<td class="cellrowborder" valign="top" width="24.51%" headers="mcps1.1.4.1.2 "><p id="p07846577448"><a name="p07846577448"></a><a name="p07846577448"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.46%" headers="mcps1.1.4.1.3 "><p id="p181139508446"><a name="p181139508446"></a><a name="p181139508446"></a>是否为宽对齐最小数量，类型是bool。</p>
</td>
</tr>
</tbody>
</table>

## 返回值说明<a name="section7624143271217"></a>

返回配置好的输出内存大小（uint32\_t类型）。

