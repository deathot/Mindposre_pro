# DrawBoundBoxToImage<a name="ZH-CN_TOPIC_0000001538865280"></a>

## 函数功能<a name="section15868409121"></a>

派生类NNObjectDetectNode中在原始图像上绘制目标检测框的函数。

## 约束说明<a name="section1771773225914"></a>

无

## 函数原型<a name="section16481811131215"></a>

**cv::Mat DrawBoundBoxToImage\(std::vector<BBox\>& detectionResults, cv::Mat imgMat\)**

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
<tbody><tr id="zh-cn_topic_0122830089_row2038874343514"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="li4644105313378p0"><a name="li4644105313378p0"></a><a name="li4644105313378p0"></a>detection Results</p>
</td>
<td class="cellrowborder" valign="top" width="24.529999999999998%" headers="mcps1.1.4.1.2 "><p id="p8693185517417"><a name="p8693185517417"></a><a name="p8693185517417"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.44%" headers="mcps1.1.4.1.3 "><p id="p275143155918"><a name="p275143155918"></a><a name="p275143155918"></a>目标检测的结果，类型是std::vector&lt;BBox&gt;&amp;。</p>
</td>
</tr>
<tr id="row162148265488"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="li1823656143710p0"><a name="li1823656143710p0"></a><a name="li1823656143710p0"></a>imgMat</p>
</td>
<td class="cellrowborder" valign="top" width="24.529999999999998%" headers="mcps1.1.4.1.2 "><p id="p92156262487"><a name="p92156262487"></a><a name="p92156262487"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.44%" headers="mcps1.1.4.1.3 "><p id="p76531158597"><a name="p76531158597"></a><a name="p76531158597"></a>原始图像，类型是cv::Mat。</p>
</td>
</tr>
</tbody>
</table>

## 返回值说明<a name="section7624143271217"></a>

返回cv::Mat类型（画了检测框的图像数据）。

