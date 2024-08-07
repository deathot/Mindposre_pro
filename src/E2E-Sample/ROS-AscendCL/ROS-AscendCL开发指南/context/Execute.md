# Execute<a name="ZH-CN_TOPIC_0000001589372665"></a>

## 函数功能<a name="section15868409121"></a>

基类NNBaseNode中定义AscendCL推理流程的函数，只做pipeline的串联，具体每个推理步骤在派生类NNObjectDetectNode中实现.。

## 约束说明<a name="section1771773225914"></a>

无

## 函数原型<a name="section16481811131215"></a>

**Result Execute\(std::vector<DataInfo\> inputs, std::vector<DataInfo\> &outputs\)**

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
<tbody><tr id="zh-cn_topic_0122830089_row2038874343514"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="li67795692518p0"><a name="li67795692518p0"></a><a name="li67795692518p0"></a>inputs</p>
</td>
<td class="cellrowborder" valign="top" width="24.529999999999998%" headers="mcps1.1.4.1.2 "><p id="p8693185517417"><a name="p8693185517417"></a><a name="p8693185517417"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.44%" headers="mcps1.1.4.1.3 "><p id="zh-cn_topic_0122830089_p19388143103518"><a name="zh-cn_topic_0122830089_p19388143103518"></a><a name="zh-cn_topic_0122830089_p19388143103518"></a>模型推理的输入数据，类型是std::vector&lt;DataInfo&gt;。</p>
</td>
</tr>
<tr id="row462010103418"><td class="cellrowborder" valign="top" width="29.03%" headers="mcps1.1.4.1.1 "><p id="p136209073416"><a name="p136209073416"></a><a name="p136209073416"></a>outputs</p>
</td>
<td class="cellrowborder" valign="top" width="24.529999999999998%" headers="mcps1.1.4.1.2 "><p id="p65341162346"><a name="p65341162346"></a><a name="p65341162346"></a>输入</p>
</td>
<td class="cellrowborder" valign="top" width="46.44%" headers="mcps1.1.4.1.3 "><p id="p46201707348"><a name="p46201707348"></a><a name="p46201707348"></a>模型推理的输出数据，类型是std::vector&lt;DataInfo&gt; &amp;。</p>
</td>
</tr>
</tbody>
</table>

## 返回值说明<a name="section7624143271217"></a>

返回Result类型错误码：

-   SUCCESS: 执行成功
-   FAILED：执行失败

