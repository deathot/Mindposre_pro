# 执行ros2指令时提示找不到该指令<a name="ZH-CN_TOPIC_0000001536657842"></a>

## 问题描述<a name="section13793123154311"></a>

在终端窗口输入ros2 xxx指令时（比如创建功能包、运行功能包节点等场景），出现如下提示：

```
-bash: ros2: command not found
```

## 可能的原因<a name="section1877432274411"></a>

终端窗口中没有设置ros2工作空间的环境变量，尤其是新启一个终端窗口的场景。

## 处理方案<a name="section18171811165610"></a>

运行用户在终端窗口执行如下命令，使ros2环境变量生效，其中_$HUMBLE _代表humble版本的ROS2安装目录，一般默认在/opt/ros/humble下。

```
source $HUMBLE/setup.bash
```

