# Atlas200DK镜像手册

# 版本

1.0.1

# 目录
<!-- TOC -->
- [目录](#目录)
- [术语](#术语)
- [前言](#前言)
- [1 最小镜像](#1-最小镜像)
  - [1.1 简介](#11-简介)
  - [1.2 组件](#12-组件)
  - [1.3 镜像大小](#13-镜像大小)
  - [1.4 SD卡容量](#14-SD卡容量)
  - [1.5 镜像验证](#15-镜像验证)
  - [1.6 Q&A](#16-QA)
- [2 基础镜像](#2-基础镜像)
  - [2.1 简介](#21-简介)
  - [2.2 组件](#22-组件)
  - [2.3 组件的使用](#23-组件的使用)
    - [2.3.1 MindX SDK的使用](#231-MindX-SDK的使用)
    - [2.3.2 Samba的使用](#232-Samba的使用)
    - [2.3.3 mqtt-broker的使用](#233-mqtt-broker的使用)
    - [2.3.4 图形界面xfce4的使用](#234-图形界面xfce4的使用)
- [3 CV镜像](#3-CV镜像)
  - [3.1 简介](#31-简介)
  - [3.2 组件](#32-组件)
  - [3.3 组件的使用](#33-组件的使用)
    - [3.3.1 opencv和opencv_contrib的使用](#331-opencv和opencv_contrib的使用)
    - [3.3.2 PIL(Python Imaging Library)的使用](#332-PILPython-Imaging-Library的使用)
- [4 ROS镜像](#4-ROS镜像)
  - [4.1 简介](#41-简介)
  - [4.2 组件](#42-组件)
  - [4.3 组件的使用](#43-组件的使用)
    - [4.3.1 ROS的使用](#431-ROS的使用)
    - [4.3.2 ROS rviz的使用](#432-ROS-rviz的使用)
    - [4.3.3 ROS gazebo的使用](#433-ROS-gazebo的使用)
    - [4.3.4 ROS Moveit!的使用](#434-ROS-Moveit的使用)
  - [4.4 Q&A](#44-QA)
<!-- /TOC -->

# 术语

| 术语       | 简称  |
| ---------- | ----- |
| Atlas200DK | 200DK |

# 前言

本文档介绍了Atlas200DK镜像相关的使用方法。包含4类镜像，分别是：


| 编号 | 镜像类型                     | 预装软件                                                     | 备注                                                         |
| ---- | ---------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1    | 最小镜像<br/>`minimal image` | 1. ubuntu-18.04<br/>2. A200dk-npu-driver-21.0.4            | 通过官网提供的制卡脚本制作的镜像，<br/>使得200DK能够运行的最小镜像。|
| 2    | 基础镜像<br/>`base image`    | 1. ubuntu-18.04<br/>2. A200dk-npu-driver-21.0.4<br/>3. Python 3.9.7<br/>4. CANN 5.1.RC2.alpha007 <br/>5. MindX SDK mxvision 3.0.RC1<br/>6. Xfce 1.4.15<br/>7. Samba 4.7.6 | 在最小镜像的基础上安装了Python\CANN\MindX SDK等昇腾AI开发组件和<br/>Xface远程桌面环境和Samba文件传输软件，用于昇腾AI应用开发。|
| 3    | CV开发镜像<br/>`cv image`    | 1. ubuntu-18.04<br/>2. A200dk-npu-driver-21.0.4<br/>3. Python 3.9.7<br/>4. CANN 5.1.RC2.alpha007 <br/>5. MindX SDK mxvision 3.0.RC1<br/>6. Xfce 1.4.15<br/>7. Samba 4.7.6<br/>8. opencv 4.6<br/>9. opencv_contrib 4.6<br/>10. PIL 9.2 | 在基础镜像上安装了OpenCV\PIL计算机视觉开发常用库和样例，用于CV开发。|
| 4    | ROS开发镜像<br/>`ros image`  | 1. ubuntu-18.04<br/>2. A200dk-npu-driver-21.0.4<br/>3. Python 3.9.7<br/>4. CANN 5.1.RC2.alpha007 <br/>5. MindX SDK mxvision 3.0.RC1<br/>6. Xfce 1.4.15<br/>7. Samba 4.7.6<br/>8. ROS melodic<br/>9. ROS core 1.14.1<br/>10. ROS rviz 1.13.25<br/>11. ROS gazebo 9.0.0<br/>12. ROS moveit 1.0.10-Alpha | 在基础镜像上安装了ROS相关组件（core\rviz\gazebo\moveit)，用于ROS开发。|

# 1 最小镜像

## 1.1 简介

最小镜像是指通过[Atlas 200 DK开发者套件>1.0.13>环境部署>制作SD卡>读卡器场景](https://www.hiascend.com/document/detail/zh/Atlas200DKDeveloperKit/1013/environment/atlased_04_0010.html)方法制作的，仅仅包含OS和固件和驱动，能够使得200DK运行的最小镜像。



## 1.2 组件

| 组件    | 版本         | 软件包                                                     |
| ------- | ------------ | ---------------------------------------------------------- |
| OS      | 18.04      | ubuntu-18.04-server-arm64.iso                            |
| NPU驱动 | 1.0.13.alpha | A200dk-npu-driver-21.0.4-ubuntu18.04-aarch64-minirc.tar.gz |



## 1.3 镜像大小

不同厂家和型号的SD卡实际存储容量有差异，为了适配32G的大部分SD卡，我们制作的镜像大小做如下限制。
| SD卡大小 | 镜像大小 | 根分区大小 |
| -------- | -------- | ---------- |
| 32G      | 29G      | 14G        |



## 1.4 SD卡容量

厂家标示容量计算方法为：1G=1000M，1M=1000KB,1KB=1000B
计算机标示容量计算方法为：1G=1024M，1M=1024KB,

所以，16G的SD卡实际容量为：
`16*1000*1000*1000/1024/1024/1024`≈`14.9GB`，镜像文件大小设置为`14GB`，扇区大小为`512B`，一共`29360128`个扇区

32G的SD卡实际容量为：
`32*1000*1000*1000/1024/1024/1024`≈`29.8GB`，镜像文件大小设置为`29GB`，扇区大小为`512B`，一共`60817408`个扇区，`/`目录分区设置为`14GB`

64G的SD卡实际容量为：
`64*1000*1000*1000/1024/1024/1024`≈`59.6GB`，镜像文件大小设置为`59GB`，扇区大小为`512B`，一共`123731968`个扇区

128G的SD卡实际容量为：
`128*1000*1000*1000/1024/1024/1024`≈`119.2GB`，镜像文件大小设置为`118GB`，扇区大小为`512B`，一共`247463936`个扇区


## 1.5 镜像验证

将烧录好的sd卡插入200DK，上电后能正常进入系统，然后可参考[Atlas 200 DK开发者套件>1.0.13>环境部署>配置网络连接](https://www.hiascend.com/document/detail/zh/Atlas200DKDeveloperKit/1013/environment/atlased_04_0012.html)进行网络配置。


## 1.6 Q&A

### 镜像无法启动

**问题描述**

如果Atlas200DK上面的固件版本是旧版本，而制作镜像使用的的固件版本较新的话，可能导致新版本的镜像无法在老版本的Atlas200DK上启动的问题。

**问题现象** 
使用1.0.13.alpha版本的固件驱动包制作的 **镜像1** 无法在Atlas200DK上面启动，根据[文档首页>Atlas 200 DK开发者套件>1.0.12>产品介绍>安全>通用安全注意事项](https://www.hiascend.com/document/detail/zh/Atlas200DKDeveloperKit/1012/productdesc/atlas200_DK_pdes_19_0003.html)使用1.0.12.alpha版本的固件驱动包制作的 **镜像2** ，将最新制作的 **镜像2** 的SD卡插入Atlas200DK，首次上电启动过程中会进行固件的升级，升级完成后会自动进行重启的操作。固件升级过程当中MINI_LED2和MINI_LED1灯会闪烁。升级完成后当前的固件版本就是1.0.12.alpha 版本；

> - Atlas 200 DK制卡后，首次上电启动过程中会进行固件的升级，升级完成后会自动进行重启的操作，重启后再进行其他组件的安装。
> - 首次启动Atlas 200 DK开发者板时不能断电，以免对Atlas 200 DK开发者板造成损害，再次上电需与上次下电时间保持2S以上的安全时间间隔。

>  **固件升级过程注意事项** 
>  - 不能执行Atlas 200 DK 开发者套件（型号 3000）开发者板断电或重启操作，否则会导致固件升级不完整，单板损坏。
>  - 当新版本升级时才会有固件升级流程，升级时间比较久，预计在15分钟内，请您耐心等待。

升级完成重启后系统即正常运行。

然后再将1.0.13.alpha版本的固件驱动包制作的 **镜像1** 的SD卡插入Atlas 200 DK，此时系统即可正常启动。

**解决办法** 
参考老版本的驱动固件制卡方法，重新使用制卡脚本制作老版本的固件驱动镜像后，首次上电进行固件升级。升级完成后，再换入较新版本的固件与驱动镜像的SD卡，即可正常启动。

| 固件与驱动版本 | 制卡方法                                                     |
| -------------- | ------------------------------------------------------------ |
| 1.0.13.alpha   | [制作SD卡>读卡器场景 1.0.13.alpha](https://www.hiascend.com/document/detail/zh/Atlas200DKDeveloperKit/1012/environment/atlased_04_0010.html) |
| 1.0.12.alpha   | [制作SD卡>读卡器场景 1.0.12.alpha](https://www.hiascend.com/document/detail/zh/Atlas200DKDeveloperKit/1012/environment/atlased_04_0010.html) |
| 1.0.11.alpha   | [制作SD卡>读卡器场景 1.0.11.alpha](https://www.hiascend.com/document/detail/zh/Atlas200DKDeveloperKit/1011/environment/atlased_04_0012.html) |



# 2 基础镜像

## 2.1 简介

基础镜像是指在最小镜像的基础上，安装了对应版本的CANN, MindX SDK的镜像，用于进行昇腾AI应用开发。

## 2.2 组件

| 组件      | 版本             | 软件包                                                     |
| --------- | ---------------- | ---------------------------------------------------------- |
| OS        | 18.04          | ubuntu-18.04-server-arm64.iso                            |
| NPU驱动   | 1.0.13.alpha     | A200dk-npu-driver-21.0.4-ubuntu18.04-aarch64-minirc.tar.gz |
| CANN      | 5.1.RC2.alpha007 | Ascend-cann-toolkit_5.1.RC2.alpha007_linux-aarch64.run     |
| Python    | 3.9.7            | Python-3.9.7.tgz                                           |
| MindX SDK | mxvision 3.0.RC1 | Ascend-mindxsdk-mxvision_3.0.RC1_linux-aarch64.run         |
|Samba| 4.7.6-ubuntu |  |
|mosquitto | 1.4.15 |   |
|xfce4 | 4.12 |  | 
|vnc | 4.1.1+xorg4.3.0 |   |

## 2.3 组件的使用

### 2.3.1 MindX SDK的使用

参考[文档首页>MindX SDK>3.0.RC1>智能视频分析>mxVision 用户指南>使用命令行方式开发>样例介绍>Python运行步骤](https://www.hiascend.com/document/detail/zh/mind-sdk/30rc1/vision/mxvisionug/mxvisionug_0042.html)



### 2.3.2 Samba的使用

**1. Samba简介**

​	Samba是在Linux和UNIX系统上实现SMB协议的一个免费软件，由服务器及客户端程序构成。SMB（Server Messages Block，信息服务块）是一种在局域网上共享文件和打印机的一种通信协议，它为局域网内的不同计算机之间提供文件及打印机等资源的共享服务。SMB协议是客户机/服务器型协议，客户机通过该协议可以访问服务器上的共享文件系统、打印机及其他资源。

**2. 创建samba登录用户**
```
useradd smb_root
smbpasswd -a smb_root
```
然后输入两次密码即可创建成功
查看 /etc/passwd文件,确定用户添加成功
```
cat /etc/passwd
```
有以下行,则添加成功
```
smb_root:x:1001:1001::/home/smb_root:/bin/sh
```
**3. 配置smb.conf**
```
vi /etc/samba/smb.conf
```
在该文件末尾进行配置如下内容
```
[share]
comment = share folder
browseable = yes
path = /home/data/share
create mask = 0755
directory mask = 0755
valid users = smb_root
force user = smb_root
force group = smb_root
public = yes
available = yes
writable = yes
```
>path路径为共享目录的路径,可自行选择.

重启samba服务
```
/etc/init.d/smbd restart 或者 service smbd restart
```
重启成功标志
```
[ ok ] Restarting smbd (via systemctl): smbd.service.
```

**4. 创建共享路径**
```
mkdir -p /home/data/share
cd /home/data
chmod -R 775 share
chown -R smb_root:smb_root share
```
**5. Windows下访问共享文件**

在Windows资源管理器地址上输入 \\\\+ip（比如我的samba服务器IP地址是192.168.1.2，则输入\\\\192.168.1.2），登陆samba服务

![输入图片说明](https://images.gitee.com/uploads/images/2022/0707/162113_11d9ddda_8237041.png "192.168.1.2.PNG")

**6. samba凭据**

此处用户和密码为前面步骤中linux环境设置的用户和密码,下图示例为smb_root和密码

![输入图片说明](https://images.gitee.com/uploads/images/2022/0707/162316_12ac08b3_8237041.png "输入密码.PNG")

**7. 共享目录文件**

linux目录下文件

![输入图片说明](https://images.gitee.com/uploads/images/2022/0707/162429_8ad52056_8237041.png "linux.PNG")

windows共享文件

![输入图片说明](https://images.gitee.com/uploads/images/2022/0707/162457_f9f422dd_8237041.png "文件内容.PNG")


### 2.3.3 mqtt-broker的使用

**1. mosquitto介绍**

参考 [mosquitto官网](https://mosquitto.org/) 

​	Eclipse Mosquitto 是一个开源（EPL/EDL 许可）消息代理，它实现了 MQTT 协议版本 5.0、3.1.1 和 3.1。Mosquitto重量轻，适用于从低功耗单板计算机到完整服务器的所有设备。MQTT 协议提供了一种使用发布/订阅模型执行消息传递的轻量级方法。这使得它适用于物联网消息传递，例如使用低功耗传感器或移动设备（例如电话，嵌入式计算机或微控制器）。Mosquitto 项目还提供了一个用于实现 MQTT 客户端的 C 库，以及非常流行的mosquitto_pub和mosquitto_sub命令行 MQTT 客户端。

**2. mqtt 简单测试**

使用两个命令窗口,一个用于订阅名为主题的"test",另一个用于向其发布消息.
Topic 是代理用于过滤每个已连接客户端的消息的标签。例如客户端订阅Topic “test” 将仅侦听其他客户端发布到同一 Topic “test” 的消息。

***终端命令***
```
mosquitto_sub -t "test"
```
将消息发布到主题 “test”，打开第二个终端，并向该 “test” 主题发布消息。
```
mosquitto_pub -m "message from mosquitto_pub cient" -t "test"
```
***测试结果***

![输入图片说明](https://images.gitee.com/uploads/images/2022/0707/193635_18d6ddb6_8237041.png "mqtt.PNG")
![输入图片说明](https://images.gitee.com/uploads/images/2022/0707/193657_ec0973e3_8237041.png "mqtt结果.PNG")

**3. mqtt加密测试**

生成mosquitto_passwd实用程序
```
mosquitto_passwd -c /etc/mosquitto/passwd zztest
Password: xxx
```
> 说明: xxx为输入的密码

为 Mosquitto 创建一个配置文件,指向刚刚创建的密码文件,创建一个空白文件
```
vi /etc/mosquitto/conf.d/default.conf
```
写入如下内容并保存
```
allow_anonymous false
password_file /etc/mosquitto/passwd
```
重启mosquitto
```
systemctl restart mosquitto
```
启动客户端
```
mosquitto_sub -t "test" -u "zztest" -P "xxx"
```
> 注意: xxx为前面步骤设置的密码

打开第二个终端输入不带密码的消息
```
mosquitto_pub -t "test" -m "message from mosquitto_pub client"
```
测试结果,发送消息失败,被拒绝
![输入图片说明](https://images.gitee.com/uploads/images/2022/0707/195234_58087b12_8237041.png "密码消息.PNG")

发送带密码消息
```
mosquitto_pub -t "test" -m "message from mosquitto_pub client with passwd" -u "zztest" -P "xxx"
```
> 注意: xxx为前面设置的密码

测试结果

![输入图片说明](https://images.gitee.com/uploads/images/2022/0707/200058_c1d92cd5_8237041.png "密码消息1.PNG")
![输入图片说明](https://images.gitee.com/uploads/images/2022/0707/195924_66881a3a_8237041.png "密码测试结果.PNG")

### 2.3.4 图形界面xfce4的使用
**1. xfce介绍**

[xfce官网](https://xfce.org/)
[xfce百度百科](https://baike.baidu.com/item/Xfce/343576)
> xfce是一个轻量级的类Unix的桌面系统, 快速加载并用来执行程序, 且占用系统资源少, xfce 是一款适用于多种 *NIX 系统的轻量级桌面环境, 它被设计用来提高您的效率，在节省系统资源的同时，能够快速加载和执行应用程序。

**2. Ubuntu启动vnc4server**

***更改vnc配置参数***

root和非root环境下，vnc的配置参数是不一样的。建议不要在root模式下启动vnc，因为这种方式启动后默认就是root模式，而且无法退回普通用户模式。而在普通用户下启动，可以随时获取root权限。
root下路径： ```/root/.vnc/xstartup```

普通用户路径: ``` /home/username/.vnc/xstartup ```
> - 注: username为当前用户

如果没有这个路径,可以先启动vncserver,"1"为开放的端口号.
```
vncserver :1
```
通过这个命令开启vnc服务，首次开启会让你输入密码，6-8位，输多了自动取前8位。
启动过程中,系统会生成此文件, 然后进行参数配置
为防止文件丢失,可以先备份
```
cp /home/username/.vnc/xstartup /home/username/.vnc/xstartup.bak
```
备份完成之后,再修改配置文件

```
vim /home/username/.vnc/xstartup
```
将文件中内容替换为(这个文件的内容只有xfce4适用)：
```
#!/bin/sh
 
# Uncomment the following two lines for normal desktop:
# unset SESSION_MANAGER
# exec /etc/X11/xinit/xinitrc
#xrdb $HOME/.Xresources
#xsettroot -solid grey
#startxfce4&
 
[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources
xsetroot -solid grey
vncconfig -iconic &
 
x-terminal-emulator -geometry 80x24+10+10 -ls -title "$VNCDESKTOP Desktop" &
#x-window-manager &
 
sesion-manager & xfdesktop & xfce4-panel &
xfce4-menu-plugin &
xfsettingsd &
xfconfd &
xfwm4 &
```
之后再启动才可以进入GUI
```
vncserver :1
```
如果无法启动,可以先停止vncserver服务
```
vncserver -kill :1
```
再使用启动命令进行启动vncserver

**3. 通过vncviewer连接服务器**

打开vncviewer,输入ip:端口号,如:

![输入图片说明](https://images.gitee.com/uploads/images/2022/0713/111457_89a999af_8237041.png "打开vncviewer.PNG")

连接成功

![输入图片说明](https://images.gitee.com/uploads/images/2022/0713/111519_9bc3d819_8237041.png "连接成功.PNG")

输入前面vncserver设置的密码

![输入图片说明](https://images.gitee.com/uploads/images/2022/0713/111600_28861ff2_8237041.png "输入密码.PNG")

图形界面

![输入图片说明](https://images.gitee.com/uploads/images/2022/0713/111613_20ec407a_8237041.png "图形界面.PNG")



# 3 CV镜像

## 3.1 简介

CV镜像在基础镜像的基础上安装OpenCV\PIL视觉开发组件、提供C++\Python接口的OpenCV库文件以及相关Sample实例。

## 3.2 组件

| 组件           | 版本  |
| -------------- | ----- |
| opencv         | 4.6   |
| opencv_contrib | 4.6   |
| PIL            | 9.2.0 |
| torch          | 1.8.0 |
| torchvision    | 0.9.1 |

## 3.3 组件的使用

### 3.3.1 opencv和opencv_contrib的使用

``` bash
cd /home/HwHiAiUser/sample/OpenCV/opencv-4.x/samples
mkdir build
cd build
cmake ..
make -j2
```

编译好的sample样例在`/home/HwHiAiUser/sample/OpenCV/opencv-4.x/samples/build/cpp`路径下，下面运行一个打印矩阵的样例`example_cpp_cout_mat`，打印出不同格式的矩阵。
``` bash
./example_cpp_cout_mat
I =
[1, 0, 0, 0;
 0, 3.141592653589793, 0, 0;
 0, 0, 1, 0;
 0, 0, 0, 1];

r (default) =
[ 91,   2,  79, 179,  52, 205, 236,   8, 181;
 239,  26, 248, 207, 218,  45, 183, 158, 101;
 102,  18, 118,  68, 210, 139, 198, 207, 211;
 181, 162, 197, 191, 196,  40,   7, 243, 230;
  45,   6,  48, 173, 242, 125, 175,  90,  63;
  90,  22, 112, 221, 167, 224, 113, 208, 123;
 214,  35, 229,   6, 143, 138,  98,  81, 118;
 187, 167, 140, 218, 178,  23,  43, 133, 154;
 150,  76, 101,   8,  38, 238,  84,  47,   7;
 117, 246, 163, 237,  69, 129,  60, 101,  41];

r (matlab) =
(:, :, 1) =
 91, 179, 236;
239, 207, 183;
102,  68, 198;
181, 191,   7;
 45, 173, 175;
 90, 221, 113;
214,   6,  98;
187, 218,  43;
150,   8,  84;
117, 237,  60
(:, :, 2) =
  2,  52,   8;
 26, 218, 158;
 18, 210, 207;
162, 196, 243;
  6, 242,  90;
 22, 167, 208;
 35, 143,  81;
167, 178, 133;
 76,  38,  47;
246,  69, 101
(:, :, 3) =
 79, 205, 181;
248,  45, 101;
118, 139, 211;
197,  40, 230;
 48, 125,  63;
112, 224, 123;
229, 138, 118;
140,  23, 154;
101, 238,   7;
163, 129,  41;

r (python) =
[[[ 91,   2,  79], [179,  52, 205], [236,   8, 181]],
 [[239,  26, 248], [207, 218,  45], [183, 158, 101]],
 [[102,  18, 118], [ 68, 210, 139], [198, 207, 211]],
 [[181, 162, 197], [191, 196,  40], [  7, 243, 230]],
 [[ 45,   6,  48], [173, 242, 125], [175,  90,  63]],
 [[ 90,  22, 112], [221, 167, 224], [113, 208, 123]],
 [[214,  35, 229], [  6, 143, 138], [ 98,  81, 118]],
 [[187, 167, 140], [218, 178,  23], [ 43, 133, 154]],
 [[150,  76, 101], [  8,  38, 238], [ 84,  47,   7]],
 [[117, 246, 163], [237,  69, 129], [ 60, 101,  41]]];

r (numpy) =
array([[[ 91,   2,  79], [179,  52, 205], [236,   8, 181]],
       [[239,  26, 248], [207, 218,  45], [183, 158, 101]],
       [[102,  18, 118], [ 68, 210, 139], [198, 207, 211]],
       [[181, 162, 197], [191, 196,  40], [  7, 243, 230]],
       [[ 45,   6,  48], [173, 242, 125], [175,  90,  63]],
       [[ 90,  22, 112], [221, 167, 224], [113, 208, 123]],
       [[214,  35, 229], [  6, 143, 138], [ 98,  81, 118]],
       [[187, 167, 140], [218, 178,  23], [ 43, 133, 154]],
       [[150,  76, 101], [  8,  38, 238], [ 84,  47,   7]],
       [[117, 246, 163], [237,  69, 129], [ 60, 101,  41]]], dtype='uint8');

r (csv) =
 91,   2,  79, 179,  52, 205, 236,   8, 181
239,  26, 248, 207, 218,  45, 183, 158, 101
102,  18, 118,  68, 210, 139, 198, 207, 211
181, 162, 197, 191, 196,  40,   7, 243, 230
 45,   6,  48, 173, 242, 125, 175,  90,  63
 90,  22, 112, 221, 167, 224, 113, 208, 123
214,  35, 229,   6, 143, 138,  98,  81, 118
187, 167, 140, 218, 178,  23,  43, 133, 154
150,  76, 101,   8,  38, 238,  84,  47,   7
117, 246, 163, 237,  69, 129,  60, 101,  41
;

r (c) =
{ 91,   2,  79, 179,  52, 205, 236,   8, 181,
 239,  26, 248, 207, 218,  45, 183, 158, 101,
 102,  18, 118,  68, 210, 139, 198, 207, 211,
 181, 162, 197, 191, 196,  40,   7, 243, 230,
  45,   6,  48, 173, 242, 125, 175,  90,  63,
  90,  22, 112, 221, 167, 224, 113, 208, 123,
 214,  35, 229,   6, 143, 138,  98,  81, 118,
 187, 167, 140, 218, 178,  23,  43, 133, 154,
 150,  76, 101,   8,  38, 238,  84,  47,   7,
 117, 246, 163, 237,  69, 129,  60, 101,  41};

p = [5, 1];
p3f = [2, 6, 7];
shortvec = [1;
 2;
 3]
points = [0, 0;
 5, 1;
 10, 2;
 15, 3;
 20, 4;
 25, 5;
 30, 6;
 35, 0;
 40, 1;
 45, 2;
 50, 3;
 55, 4;
 60, 5;
 65, 6;
 70, 0;
 75, 1;
 80, 2;
 85, 3;
 90, 4;
 95, 5];
```


再运行一个python版本的样例，提取图片当中的直线，因为没有安装图形界面相关的功能，把代码当中图片显示的相关代码注释掉，修改为保存文件：

源码路径：`/home/HwHiAiUser/opencv/opencv_contrib-4.x/samples/python2/lsd_lines_extraction.py`

``` 
# python3 lsd_lines_extraction.py

This example shows the functionalities of lines extraction finished by LSDDetector class.

USAGE: lsd_lines_extraction.py [<path_to_input_image>]
```
输入图片如下，图片路径为`../data/corridor.jpg`。
![corridor](https://images.gitee.com/uploads/images/2022/0726/144751_68aec115_9317615.png "corridor.png")

输出图片为`output.jpg`，图片如下：
![output](https://images.gitee.com/uploads/images/2022/0726/144911_45209ae4_9317615.png "output.png")


### 3.3.2 PIL(Python Imaging Library)的使用

**1. Pillow简介**

[Pillow Github代码仓](https://github.com/python-pillow/Pillow)
[Pillow Handbook](https://pillow.readthedocs.io/en/latest/handbook/index.html)
Python Imaging Library 为您的 Python 解释器添加了图像处理功能。
这个库提供了广泛的文件格式支持、高效的内部表示和相当强大的图像处理能力。
核心图像库旨在快速访问以几种基本像素格式存储的数据。
它为通用图像处理工具提供坚实的基础。

**2. PIL的使用**

下面使用PIL库对图像进行滤波、轮廓和边缘检测：
``` python
from PIL import Image
from PIL import ImageFilter                         ## 调取ImageFilter

imgF = Image.open("/home/HwHiAiUser/opencv/opencv_contrib-4.x/samples/data/corridor.jpg")
bluF = imgF.filter(ImageFilter.BLUR)                ##均值滤波
conF = imgF.filter(ImageFilter.CONTOUR)             ##找轮廓
edgeF = imgF.filter(ImageFilter.FIND_EDGES)         ##边缘检测

bluF.save("bluF.png")
conF.save("conF.png")
edgeF.save("edgeF.png")
```
结果如下

![PIL](https://images.gitee.com/uploads/images/2022/0705/182900_df04e8b8_9317615.png "PIL.png")


# 4 Ros镜像
## 4.1 简介
[百度百科](https://baike.baidu.com/item/ros/4710560?fr=aladdin)
[Ros官网](https://www.ros.org/)
> ros是机器人操作系统（Robot Operating System）的英文缩写。ROS是用于编写机器人软件程序的一种具有高度灵活性的软件架构。它包含了大量工具软件、库代码和约定协议，旨在简化跨机器人平台创建复杂、鲁棒的机器人行为这一过程的难度与复杂度。它提供了操作系统应有的服务，包括硬件抽象，底层设备控制，常用函数的实现，进程间消息传递，以及包管理。它也提供用于获取、编译、编写、和跨计算机运行代码所需的工具和库函数。
## 4.2 组件
| 组件          | 版本               | 软件包                                                        |
|---------------|------------------|------------------------------------------------------------|
| OS          | 18.04          | ubuntu-18.04-server-arm64.iso                            |
| NPU驱动       | 1.0.13.alpha     | A200dk-npu-driver-21.0.4-ubuntu18.04-aarch64-minirc.tar.gz |
| CANN        | 5.1.RC2.alpha007 | Ascend-cann-toolkit_5.1.RC2.alpha007_linux-aarch64.run     |
| Python      | 3.9.7            | Python-3.9.7.tgz                                           |
| MindX SDK   | mxvision 3.0.RC1 | Ascend-mindxsdk-mxvision_3.0.RC1_linux-aarch64.run         |
| Xfce        | 1.4.15          | -                                                          |
| Samba          |   4.7.6               | -                                                           |
| ROS          | melodic                 | -                                                           |
| ROS core          | 1.14.1                 | -                                               |
| ROS rviz          | 1.13.25                 | -                                               |
| ROS gazebo          | 9.0.0                  | -                                               |
| ROS moveit           | 1.0.10-Alpha                 | -                                       |

## 4.3 组件的使用
### 4.3.1 ROS的使用

打开3个终端，以下称为终端1、终端2和终端3。
打开终端1，执行以下命令：
> 【找不到roscore？-> Q&A】

`roscore`

打开终端2，执行以下命令后会弹出一个“小乌龟”框：

`rosrun turtlesim turtlesim_node`

打开终端3，执行以下命令，并用键盘的“上下左右”键控制“小乌龟”：

`rosrun turtlesim turtle_teleop_key`


### 4.3.2 ROS rviz的使用
**官方文档：** [http://wiki.ros.org/rviz](http://wiki.ros.org/rviz)

 **1. rviz简介** 

rviz是ros的一个可视化工具，用于可视化传感器的数据和状态信息。
rviz支持丰富的数据类型，通过加载不同的Dispalys类型来可视化，每一个Dispaly都有一个独特的名字。

 **2. 发送基础形状至RVIZ（C++）** 

 **1) catkin工作目录如下所示** 

```
|---workspace
|    |---src
|      |---package_1
|         |---CMakeLists.txt
|         |---src 
|             |---xxx.cpp
       |---package_2
...
|
|      |---package_n
```

 **2) 创建程序包** 

首先执行以下命令，创建自己的工作空间以及包目录

`mkdir –p ./rviz_demo/src`

![输入图片说明](https://images.gitee.com/uploads/images/2022/0714/141355_26768466_11225962.png "屏幕截图.png")

执行以下命令，创建catkin工作空间

`catkin_create_pkg using_markers roscpp visualization_msgs`

![输入图片说明](https://images.gitee.com/uploads/images/2022/0714/141506_5a036846_11225962.png "屏幕截图.png")

完成后，目录及文件如下：

![输入图片说明](https://images.gitee.com/uploads/images/2022/0714/141529_3d3c11f7_11225962.png "屏幕截图.png")

 **3) 创建节点** 

执行以下命令，创建文件

```
cd ./rviz_demo/src/src
touch basic_shapes.cpp
```

代码内容可访问以下链接进行拷贝：
[https://raw.githubusercontent.com/ros-visualization/visualization_tutorials/indigo-devel/visualization_marker_tutorials/src/basic_shapes.cpp ](http://raw.githubusercontent.com/ros-visualization/visualization_tutorials/indigo-devel/visualization_marker_tutorials/src/basic_shapes.cpp)

 **4) 编辑CMakeLists.txt文件** 

进入目录usring_markers，在CMakeList.txt的内容最后添加以下内容：

```
add_executable(basic_shapes src/basic_shapes.cpp)
target_link_libraries(basic_shapes ${catkin_LIBRARIES})
```

完成后目录结构如下：

![输入图片说明](https://images.gitee.com/uploads/images/2022/0714/141956_f9b96a5b_11225962.png "屏幕截图.png")

 **5) 编译源码** 

进入到rviz_demo目录，执行以下命令进行编译：
`catkin_make`

 **6) 运行节点** 

进入到rviz_demo目录，执行以下命令运行节点：

`rosrun using_markers basic_shapes`
> 这时会提示Warning提醒你没有订阅者。

![输入图片说明](https://images.gitee.com/uploads/images/2022/0714/142159_44cb438d_11225962.png "屏幕截图.png")

 **7) 启动rviz** 

开启两个终端，一个执行以下命令启动roscore：

`roscore`

另一个终端非root执行以下命令启动rivz：

`rviz`

> 如果启动不成功,可能是环境变量未设置,设置环境变量
> 
> `source /opt/ros/melodic/setup.bash`

弹出rviz界面如下:

![输入图片说明](https://images.gitee.com/uploads/images/2022/0714/142334_8178560c_11225962.png "屏幕截图.png")

第一次启动需要将Fixed Frame修改为my_frame，然后点击add，选择Markers，再点击ok就可以看到rviz显示区域出现了代码中设置的形状。
> 为什么要设置成my_frame? --> 查看basic_shapes.cpp的内容“marker.header.frame_id = "/my_frame";”

![输入图片说明](https://images.gitee.com/uploads/images/2022/0714/142400_9dfb36f1_11225962.png "屏幕截图.png")
![输入图片说明](https://images.gitee.com/uploads/images/2022/0714/142917_40e474e0_11225962.png "屏幕截图.png")

至此你已经完成了rviz的简单使用教程，更多样例和教程请参考[官方文档](http://wiki.ros.org/rviz)。

### 4.3.3 ROS gazebo的使用

 **1. gazebo启动**
使用以下命令启动：
```
gazebo
```

> 回显内容：
> HwHiAiUser@davinci-mini:~$ gazebo
> [Err] [REST.cc:205] Error in REST request
> 
> libcurl: (51) SSL: no alternative certificate subject name matches target host name 'api.ignitionfuel.org'


![输入图片说明](https://images.gitee.com/uploads/images/2022/0714/160234_202f0037_8237041.png "gazebo.PNG")

如果gazebo无法启动,可能是上一个进程在关闭gazebo界面后,进程未正常关闭,使用top命令查看进程,是否有gzserver
```
top | grep gzserver
```
若结果如下,则证明gazebo未正常关闭

> root@davinci-mini:/home/HwHiAiUser/gazebo_model# top | grep gzserver
> 10395 HwHiAiU+  20   0 3683380 149940  93348 S  17.6  1.9   3:51.51 gzserver
> 10395 HwHiAiU+  20   0 3683380 149940  93348 S  12.9  1.9   3:51.90 gzserver
> 10395 HwHiAiU+  20   0 3683380 149940  93348 S  12.5  1.9   3:52.28 gzserver

使用kill命令杀死进程
```
kill 10395
```
切换至普通用户,再次使用gazebo命令重启,可正常启动.
```
gazebo
```
> 若非此原因无法重启,可能是model库加载不正确导致的，gazebo软件开启的时候会自动从网络下载模型，因此这个过程比较漫长，主要是网络问题,可在网上搜索查询解决方案,进行相应的修复

若未安装，使用以下命令进行安装：

`sudo apt install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros ros-melodic-gazebo-msgs ros-melodic-gazebo-plugins ros-melodic-gazebo-ros-control`


 **2. 教程-创建小车模型** 

 **1) 创建功能包** 

执行以下命令，创建ros工作空间

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```


> 回显内容：
> Creating symlink "/home/HwHiAiUser/catkin_ws/src/CMakeLists.txt" pointing to "/opt/ros/melodic/share/catkin/cmake/toplevel.cmake" 

```
cd ~/catkin_ws
catkin_make
```


> 回显内容：
> Base path: /home/HwHiAiUser/catkin_ws
> Source space: /home/HwHiAiUser/catkin_ws/src
> Build space: /home/HwHiAiUser/catkin_ws/build
> Devel space: /home/HwHiAiUser/catkin_ws/devel
> Install space: /home/HwHiAiUser/catkin_ws/install
> \####
> \#### Running command: "cmake /home/HwHiAiUser/catkin_ws/src -DCATKIN_DEVEL_PREFIX=/home/HwHiAiUser/catkin_ws/devel -DCMAKE_INSTALL_PREFIX=/home/HwHiAiUser/catkin_ws/install -G Unix Makefiles" in "/home/HwHiAiUser/catkin_ws/build"
> \####
> -- The C compiler identification is GNU 7.5.0
> -- The CXX compiler identification is GNU 7.5.0
> -- Check for working C compiler: /usr/bin/cc
> -- Check for working C compiler: /usr/bin/cc -- works
> -- Detecting C compiler ABI info
> -- Detecting C compiler ABI info - done
> -- Detecting C compile features
> -- Detecting C compile features - done
> -- Check for working CXX compiler: /usr/bin/c++
> -- Check for working CXX compiler: /usr/bin/c++ -- works
> -- Detecting CXX compiler ABI info
> -- Detecting CXX compiler ABI info - done
> -- Detecting CXX compile features
> -- Detecting CXX compile features - done
> -- Using CATKIN_DEVEL_PREFIX: /home/HwHiAiUser/catkin_ws/devel
> -- Using CMAKE_PREFIX_PATH: /home/HwHiAiUser/catkin_ws/devel;/opt/ros/melodic
> -- This workspace overlays: /home/HwHiAiUser/catkin_ws/devel;/opt/ros/melodic
> -- Found PythonInterp: /usr/bin/python2 (found suitable version "2.7.17", minimum required is "2")
> -- Using PYTHON_EXECUTABLE: /usr/bin/python2
> -- Using Debian Python package layout
> -- Using empy: /usr/bin/empy
> -- Using CATKIN_ENABLE_TESTING: ON
> -- Call enable_testing()
> -- Using CATKIN_TEST_RESULTS_DIR: /home/HwHiAiUser/catkin_ws/build/test_results
> -- Found gtest sources under '/usr/src/googletest': gtests will be built
> -- Found gmock sources under '/usr/src/googletest': gmock will be built
> -- Found PythonInterp: /usr/bin/python2 (found version "2.7.17")
> -- Looking for pthread.h
> -- Looking for pthread.h - found
> -- Looking for pthread_create
> -- Looking for pthread_create - not found
> -- Looking for pthread_create in pthreads
> -- Looking for pthread_create in pthreads - not found
> -- Looking for pthread_create in pthread
> -- Looking for pthread_create in pthread - found
> -- Found Threads: TRUE
> -- Using Python nosetests: /usr/bin/nosetests-2.7
> -- catkin 0.7.29
> -- BUILD_SHARED_LIBS is on
> -- BUILD_SHARED_LIBS is on
> -- Configuring done
> -- Generating done
> -- Build files have been written to: /home/HwHiAiUser/catkin_ws/build
> \####
> \#### Running command: "make -j8 -l8" in "/home/HwHiAiUser/catkin_ws/build"
> \####
`source devel/setup.bash`

 **2) 创建功能包** 

执行以下命令，创建功能包

```
cd ~/catkin_ws/src
catkin_create_pkg diff_wheeled_robot_gazebo roscpp tf geometry_msgs urdf rviz xacro
```


> 回显内容：
> Created file diff_wheeled_robot_gazebo/package.xml
> Created file diff_wheeled_robot_gazebo/CMakeLists.txt
> Created folder diff_wheeled_robot_gazebo/include/diff_wheeled_robot_gazebo
> Created folder diff_wheeled_robot_gazebo/src
> Successfully created files in /home/HwHiAiUser/catkin_ws/src/diff_wheeled_robot_gazebo. Please adjust the values in package.xml.

 **3) 创建基本文件夹** 

执行以下命令，创建文件夹

```
cd diff_wheeled_robot_gazebo/
mkdir urdf meshes launch world
```

 **4) 拷贝模型等文件** 

① 将[ **gazebo工程包** ](https://pan.baidu.com/s/1KBGmehQ-7PSpTVbVUrEOXA?pwd=y4b9)urdf文件夹中的 diff_wheeled_robot.xacro 文件和 wheel.urdf.xacro 文件拷贝进自己的工程的urdf文件夹。

② 将[ **gazebo工程包** ](https://pan.baidu.com/s/1KBGmehQ-7PSpTVbVUrEOXA?pwd=y4b9)mesh文件夹中的三维模型 caster_wheel.stl 文件拷贝进自己工程的meshes文件夹。

③ 将[ **gazebo工程包** ](https://pan.baidu.com/s/1KBGmehQ-7PSpTVbVUrEOXA?pwd=y4b9)launch文件夹中的diff_wheeled_gazebo.launch文件拷贝进自己工程的launch文件夹。

 **5) 编译并启动节点** 

执行以下命令，编译工作空间

```
cd ~/catkin_ws
catkin_make
```

> 回显内容：
> Base path: /home/HwHiAiUser/catkin_ws
> Source space: /home/HwHiAiUser/catkin_ws/src
> Build space: /home/HwHiAiUser/catkin_ws/build
> Devel space: /home/HwHiAiUser/catkin_ws/devel
> Install space: /home/HwHiAiUser/catkin_ws/install
> \####
> \#### Running command: "cmake /home/HwHiAiUser/catkin_ws/src -DCATKIN_DEVEL_PREFIX=/home/HwHiAiUser/catkin_ws/devel -DCMAKE_INSTALL_PREFIX=/home/HwHiAiUser/catkin_ws/install -G Unix Makefiles" in "/home/HwHiAiUser/catkin_ws/build"
> \####
> -- Using CATKIN_DEVEL_PREFIX: /home/HwHiAiUser/catkin_ws/devel
> -- Using CMAKE_PREFIX_PATH: /home/HwHiAiUser/catkin_ws/devel;/opt/ros/melodic
> -- This workspace overlays: /home/HwHiAiUser/catkin_ws/devel;/opt/ros/melodic
> -- Found PythonInterp: /usr/bin/python2 (found suitable version "2.7.17", minimum required is "2")
> -- Using PYTHON_EXECUTABLE: /usr/bin/python2
> -- Using Debian Python package layout
> -- Using empy: /usr/bin/empy
> -- Using CATKIN_ENABLE_TESTING: ON
> -- Call enable_testing()
> -- Using CATKIN_TEST_RESULTS_DIR: /home/HwHiAiUser/catkin_ws/build/test_results
> -- Found gtest sources under '/usr/src/googletest': gtests will be built
> -- Found gmock sources under '/usr/src/googletest': gmock will be built
> -- Found PythonInterp: /usr/bin/python2 (found version "2.7.17")
> -- Using Python nosetests: /usr/bin/nosetests-2.7
> -- catkin 0.7.29
> -- BUILD_SHARED_LIBS is on
> -- BUILD_SHARED_LIBS is on
> -- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
> -- ~~  traversing 1 packages in topological order:
> -- ~~  - diff_wheeled_robot_gazebo
> -- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
> -- +++ processing catkin package: 'diff_wheeled_robot_gazebo'
> -- ==> add_subdirectory(diff_wheeled_robot_gazebo)
> -- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
> -- Configuring done
> -- Generating done
> -- Build files have been written to: /home/HwHiAiUser/catkin_ws/build
> \####
> \#### Running command: "make -j8 -l8" in "/home/HwHiAiUser/catkin_ws/build"
> \####
> Scanning dependencies of target diff_wheeled_robot_gazebo_xacro_generated_to_devel_space_
> Built target diff_wheeled_robot_gazebo_xacro_generated_to_devel_space

执行以下命令，启动节点

`roslaunch diff_wheeled_robot_gazebo diff_wheeled_gazebo.launch`
![输入图片说明](https://images.gitee.com/uploads/images/2022/0715/111457_9cd9352b_11225962.png "屏幕截图.png")

 **3. 教程-控制小车模型** 

 **1) 插件说明** 

控制小车移动所使用的插件是 libgazebo_ros_diff_drive.so。此插件的添加代码已经写在了diff_wheeled_robot.xacro文件中如下：

```
<gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <rosDebugLevel>Debug</rosDebugLevel>
      <publishWheelTF>false</publishWheelTF>
      <robotNamespace>/</robotNamespace>
      <publishTf>1</publishTf>
      <publishWheelJointState>false</publishWheelJointState>
      <alwaysOn>true</alwaysOn>
      <updateRate>100.0</updateRate>
      <leftJoint>front_left_wheel_joint</leftJoint>
      <rightJoint>front_right_wheel_joint</rightJoint>
      <wheelSeparation>${2*base_radius}</wheelSeparation>
      <wheelDiameter>${2*wheel_radius}</wheelDiameter>
      <broadcastTF>1</broadcastTF>
      <wheelTorque>30</wheelTorque>
      <wheelAcceleration>1.8</wheelAcceleration>
      <commandTopic>cmd_vel</commandTopic>
      <odometryFrame>odom</odometryFrame> 
      <odometryTopic>odom</odometryTopic> 
      <robotBaseFrame>base_footprint</robotBaseFrame>
    </plugin>
  </gazebo>
```
其中可以指定的参数包括轮子的关节、轮子的间距、车轮直径、里程计的主题等等。这里面最重要的一个参数是控制命令主题 commandTopic，用于驱动车轮的运动。我们可以通过向 **/cmd_vel** 主题发布数据来控制小车的运动。

 **2) 测试小车模型** 

① 启动节点打开小车模型。

② 执行以下命令，发布数据以控制小车

```
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}'

```

至此，可在gazebo界面看到小车开始进行圆周运动。

 **4. 教程-键盘控制小车** 

 **1) 创建功能包** 

```
cd ~/catkin/src
catkin_create_pkg diff_wheeled_robot_control rospy tf geometry_msgs urdf rviz xacro
```


> 回显内容：
> Created file diff_wheeled_robot_control/package.xml
> Created file diff_wheeled_robot_control/CMakeLists.txt
> Created folder diff_wheeled_robot_control/src
> Successfully created files in /home/HwHiAiUser/catkin_ws/src/diff_wheeled_robot_control. Please adjust the values in package.xml.

将[ **control工程包** ](https://pan.baidu.com/s/1T41KeZDsoyVSW3O-WoQINg?pwd=oa7s)中的launch文件夹和scripts文件夹拷贝到新创建的功能包中。

 **2) 编译启动** 

① 编译

```
cd ~/catkin
catkin_make
```

> 回显内容：
> 
> Base path: /home/HwHiAiUser/catkin_ws
> Source space: /home/HwHiAiUser/catkin_ws/src
> Build space: /home/HwHiAiUser/catkin_ws/build
> Devel space: /home/HwHiAiUser/catkin_ws/devel
> Install space: /home/HwHiAiUser/catkin_ws/install
> \####
> \#### Running command: "cmake /home/HwHiAiUser/catkin_ws/src -DCATKIN_DEVEL_PREFIX=/home/HwHiAiUser/catkin_ws/devel -DCMAKE_INSTALL_PREFIX=/home/HwHiAiUser/catkin_ws/install -G Unix Makefiles" in "/home/HwHiAiUser/catkin_ws/build"
> \####
> -- Using CATKIN_DEVEL_PREFIX: /home/HwHiAiUser/catkin_ws/devel
> -- Using CMAKE_PREFIX_PATH: /home/HwHiAiUser/catkin_ws/devel;/opt/ros/melodic
> -- This workspace overlays: /home/HwHiAiUser/catkin_ws/devel;/opt/ros/melodic
> -- Found PythonInterp: /usr/bin/python2 (found suitable version "2.7.17", minimum required is "2")
> -- Using PYTHON_EXECUTABLE: /usr/bin/python2
> -- Using Debian Python package layout
> -- Using empy: /usr/bin/empy
> -- Using CATKIN_ENABLE_TESTING: ON
> -- Call enable_testing()
> -- Using CATKIN_TEST_RESULTS_DIR: /home/HwHiAiUser/catkin_ws/build/test_results
> -- Found gtest sources under '/usr/src/googletest': gtests will be built
> -- Found gmock sources under '/usr/src/googletest': gmock will be built
> -- Found PythonInterp: /usr/bin/python2 (found version "2.7.17")
> -- Using Python nosetests: /usr/bin/nosetests-2.7
> -- catkin 0.7.29
> -- BUILD_SHARED_LIBS is on
> -- BUILD_SHARED_LIBS is on
> -- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
> -- ~~  traversing 2 packages in topological order:
> -- ~~  - diff_wheeled_robot_control
> -- ~~  - diff_wheeled_robot_gazebo
> -- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
> -- +++ processing catkin package: 'diff_wheeled_robot_control'
> -- ==> add_subdirectory(diff_wheeled_robot_control)
> -- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
> -- +++ processing catkin package: 'diff_wheeled_robot_gazebo'
> -- ==> add_subdirectory(diff_wheeled_robot_gazebo)
> -- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
> -- Configuring done
> -- Generating done
> -- Build files have been written to: /home/HwHiAiUser/catkin_ws/build
> \####
> \#### Running command: "make -j8 -l8" in "/home/HwHiAiUser/catkin_ws/build"
> \####
> Scanning dependencies of target diff_wheeled_robot_control_xacro_generated_to_devel_space_
> Built target diff_wheeled_robot_control_xacro_generated_to_devel_space_
> Built target diff_wheeled_robot_gazebo_xacro_generated_to_devel_space_

② 启动

开启一个终端启动小车模型

`roslaunch diff_wheeled_robot_gazebo diff_wheeled_gazebo.launch`

开启另一个终端启动控制程序

`roslaunch diff_wheeled_robot_control keyboard_teleop.launc`


> 如果启动失败，则执行以下命令：
> 
> source ~/catkin_ws/devel/setup.bash
> chmod +x ~/catkin_ws/src/diff_wheeled_robot_control/scripts/diff_wheeled_robot_key

至此你可以看到回显提示，通过键盘输入来控制小车。

### 4.3.4 ROS Moveit!的使用
官方教程请点击[官方教程](https://docs.ros.org/en/melodic/api/moveit_tutorials/html/)

 **1. Moveit!简介** 

MoveIt! 是一个机器人（mobile manipulation）相关的工具集软件，集成了各种 SOTA 库，包括：运动规划（Motion Planning）、操作（Manipulation）、3D 感知（Perception）、运动学（Kinematics）、碰撞检测（Collision Checking）、控制（Control）、导航（Navigation）。

 **2. 下载样例代码** 

执行以下命令下载样例代码：

```
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src
git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel
```

 **3. 构建Catkin工作空间** 

**1) 执行以下代码安装包的依赖**

```
cd ~/ws_moveit/src
rosdep install -y --from-paths . --ignore-src --rosdistro melodic
```
回显如下表示成功：
> \#All required rosdeps installed successfully

**2) 执行以下代码配置catkin**

```
cd ~/ws_moveit
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
```

回显如下表示成功：
> Initialized new catkin workspace in `/home/HwHiAiUser/ws_moveit`

**3) 执行以下代码构建catkin**

`catkin build`
回显如下表示成功：


> [build] Found '1' packages in 0.0 seconds.
> [build] Updating package table.
> Starting  >>> catkin_tools_prebuild
> Finished  <<< catkin_tools_prebuild                [ 8.6 seconds ]
> Starting  >>> panda_moveit_config
> Finished  <<< panda_moveit_config                  [ 8.0 seconds ]
> [build] Summary: All 2 packages succeeded!
> [build]   Ignored:   None.
> [build]   Warnings:  None.
> [build]   Abandoned: None.
> [build]   Failed:    None.
> [build] Runtime: 16.7 seconds total.
> [build] Note: Workspace packages have changed, please re-source setup files to use them.

**4) 执行以下代码更新环境变量**

`source ~/ws_moveit/devel/setup.bash`

可以执行以下命令将更新环境变量的命令写入.bashrc【可选】

`echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc`

 **4. 启动demo** 

执行以下命令启动demo:

`roslaunch panda_moveit_config demo.launch rviz_tutorial:=true`

![输入图片说明](https://images.gitee.com/uploads/images/2022/0719/202951_7713baee_11225962.png "屏幕截图.png")

选择Add，再选择MotionPlanning，点击ok确定。

![输入图片说明](https://images.gitee.com/uploads/images/2022/0719/203216_cce64cea_11225962.png "屏幕截图.png")

选择Global Options，设置Fixed Frame为panda_link0。

![输入图片说明](https://images.gitee.com/uploads/images/2022/0719/203323_5310e00c_11225962.png "屏幕截图.png")

选择MotionPlanning，
设置Robot Description为robot_description
设置Planning Scene Topic为/planning_scene
设置Planning Group为panda_arm
设置Trajectory Topic为/move_group/display_planned_path

![输入图片说明](https://images.gitee.com/uploads/images/2022/0719/203618_78f9927b_11225962.png "屏幕截图.png")

 **5. 播放可视化机器人** 

选中Planned Path选项卡中的Show Robot Visual复选框
取消选中Scene Robot选项卡中的Show Robot Visual复选框
选中Planning Request选项卡中的Query goal State复选框。
选中Planning Request选项卡中的Query Start State复选框。

![输入图片说明](https://images.gitee.com/uploads/images/2022/0719/204122_2fd401a2_11225962.png "屏幕截图.png")

对应于橙色手臂的一个标记将用于设置运动规划的“目标状态”，对应于绿色手臂的另一个标记用于设置运动规划的“开始状态”。拖动红色/蓝色/绿色的坐标圈，使橙色手臂处于新的状态。

![输入图片说明](https://images.gitee.com/uploads/images/2022/0719/204241_7ede3da0_11225962.png "屏幕截图.png")

在MotionPlanning下的Planning里点击Plan按钮，即可播放该可视化机器人的运动轨迹。若没有该框，打开Panels下拉，选中MotionPlanning。

![输入图片说明](https://images.gitee.com/uploads/images/2022/0719/204659_eccce061_11225962.png "屏幕截图.png")

打开Panels下拉，选中MotionPlanning-Slider。可在该框中点击play按钮进行播放轨迹，或者拖动进度条查看状态。

![输入图片说明](https://images.gitee.com/uploads/images/2022/0719/204929_a1eb2e71_11225962.png "屏幕截图.png")

至此，你完成了简单的Moveit!使用教程。

## 4.4 Q&A

1) sudo rosdep init 找不到命令失败？

- 原因：rosdep未安装。
- 解决：执行 sudo apt install python-rosdep 安装。

2) sudo rosdep init 已经存在文件20-default.list？

- 原因：20-default.list文件已存在，不能被覆盖。
- 解决：执行 sudo rm /etc/ros/rosdep/sources.list.d/20-default.list 删除文件。

3) rosdep update超时失败？

- 原因：服务器访问受限。
- 解决：添加代理，（参考链接：https://zhuanlan.zhihu.com/p/392082731 本步骤较多，请耐心配置，亲测有效！）

4) ROS安装找不到roscore命令？

- 原因：环境变量未设置。
- 解决：执行命令 source /opt/ros/melodic/setup.bash，其中melodic根据实际环境修改。

5) sudo命令报错？
- 原因：HwHiAiUser不具备root权限.
- 解决：切换root用户执行以下命令：
`vim /etc/sudoers`
在# User privilege specification下面添加以下内容：
`HwHiAiUser ALL=(ALL) ALL`
退出保存即可。
