### A200DK镜像制卡方法
当前A200DK的镜像制作方法有2种
1. 昇腾官网文档提供的方法，参考链接[Atlas 200 DK开发者套件 安装部署流程](https://www.hiascend.com/document/detail/zh/Atlas200DKDeveloperKit/1013/environment/atlased_04_0002.html)，这种方法仅仅将OS和昇腾NPU驱动进行了打包，制作出来的镜像是使得Atlas 200 DK能够启动的最小集合，没有安装CANN、MindX、Sample和第三方的大量依赖，开发者需要根据自己的需求自行安装各种依赖，耗费大量时间。
2. 昇腾论坛提供的dd镜像，参考链接[更方便的200DK合设环境搭建方法——dd镜像](https://www.hiascend.com/forum/thread-0217101703106643028-1-1.html), 这种镜像在第一种方法制作的镜像基础上，安装好了CANN的依赖和CANN，并匹配好了昇腾NPU驱动和CANN的版本，并通过社区的力量不断更新迭代驱动和CANN的版本，推出最新的镜像。MindX、Sample和第三方的大量依赖仍然需要用户自行安装。dd镜像是通过在Linux环境下通过`dd`命令导出的镜像文件，使用dd镜像要求SD卡的容量大小必须大于等于dd镜像文件大小本身，否则镜像文件无法烧写到SD卡里面。dd镜像文件较大（大约4~5GB），通过网盘存储和分享，下载速度较慢。

我们在前2种镜像制作方法的基础上，进行了用户体验方面的改善，推出了**昇腾开发者套件镜像烧录工具**，集镜像在线下载、烧录、备份功能于一体。同时我们在镜像方面进行了以下改进：
1. 针对CV、ROS等常见开发场景，镜像预安装了昇腾NPU驱动、CANN、CANN Sample、MindX SDK(mxvision)、OpenCV(Python\C++接口）、PIL、ROS框架、miniconda和运行CANN、MindX SDK推理应用样例所需的第三方依赖环境。使得开发者开箱即用，免去安装大量依赖软件包的等待时间。镜像预置CANN、MindX SDK Sample样例，帮助开发者30分钟上手运行样例，直观感受昇腾开发者套件在AI视觉、机器人等领域的应用优势。
2. 对镜像文件进行了压缩，降低传输下载的带宽要求，并在镜像首次启动时进行自动扩容，以充分利用整张SD卡的存储空间。
3. 镜像存储在华为云obs，提供高速下载通道。
上述3种进行对比如下：

| 对比项 | 方法1昇腾官网文档脚本制作 | 方法2昇腾论坛dd镜像 | 方法3昇腾开发者套件镜像烧录工具  |
| ----- | ------| ------|  -----|
| OS+NPU驱动固件    |  是      |  是    | 是       |
| 预装常用软件     | 否       | 否     | 是      |
| 预置Sample样例     | 否       | 否     | 是       |
| 支持SD卡自动扩容     | 否       | 否     | 是       |


### A200DK 镜像内容
参考[Atlas 200 DK开发者套件 安装部署流程](https://www.hiascend.com/document/detail/zh/Atlas200DKDeveloperKit/1013/environment/atlased_04_0002.html)提供的镜像制作脚本:
> 下载制卡入口脚本`make_sd_card.py`:
    `wget https://gitee.com/ascend/tools/raw/master/makesd/generic_script/make_sd_card.py`
下载制作SD卡操作系统的脚本`make_ubuntu_sd.sh`。
`wget https://gitee.com/ascend/tools/raw/master/makesd/generic_script/make_ubuntu_sd.sh`

可以得到下图的镜像布局，一个完整的镜像包含了5个部分：
1. 分区头
2. 分区1
3. 分区2
4. 分区3
5. sectorRsv保留区域

其中sectorRsv保留区域存储了内核、固件等文件，是A200DK启动的关键组成部分，sectorRsv保留区域有2部分组成：COMPONENTS_MAIN 和 COMPONENTS_BACKUP 2部分，二者的大小固定为73728个扇区。其中的COMPONENTS_MAIN_OFFSET 和 COMPONENTS_BACKUP_OFFSET 2个偏移值保存在分区头的固定位置，A200DK的启动代码BootLoader读取分区头信息后获取COMPONENTS_MAIN_OFFSET 和 COMPONENTS_BACKUP_OFFSET 2个偏移，首先尝试从主分区启动，如果主分区无法启动，再尝试从备分区启动。

![输入图片说明](https://foruda.gitee.com/images/1667286983019288977/ac404274_9317615.png "屏幕截图")



### A200DK 分区查看

将SD卡通过读卡器插入到一台Ubuntu PC，切换到root用户，执行`fdisk -l`，查看SD卡里镜像的分区大小情况：

``` bash
Disk /dev/sda：29.72 GiB，31914983424 字节，62333952 个扇区
Disk model: Storage Device  
单元：扇区 / 1 * 512 = 512 字节
扇区大小(逻辑/物理)：512 字节 / 512 字节
I/O 大小(最小/最佳)：512 字节 / 512 字节
磁盘标签类型：dos
磁盘标识符：0x73628cb9

设备       启动     起点     末尾     扇区  大小 Id 类型
/dev/sda1           2048 29362175 29360128   14G 83 Linux
/dev/sda2       29362176 31459327  2097152    1G 83 Linux
/dev/sda3       31459328 62186495 30727168 14.7G 83 Linux
```

``` 
>>> 73728*2
147456
>>> 62186495+1+147456
62333952
>>>
```

可以看出，这张SD卡总大小为62333952个扇区， 每个扇区大小为512字节，前3个分区的总大小为`62186495+1=62186496`个扇区，sectorRsv保留区域的COMPONENTS_MAIN 和 COMPONENTS_BACKUP 2部分大小为147456个扇区，总计`62186495+1+147456=62333952`个扇区。



### sectorRsv保留区域备份

执行下面的命令，读取sectorRsv保留区域的COMPONENTS_MAIN 和 COMPONENTS_BACKUP 2部分，注意` skip=62186496` 的含义是从输入端跳过62186496个二扇区，从下一个扇区开始读取：

``` 
dd if=/dev/sda of=sectorRsv_main_backup.img bs=512 count=147456 skip=62186496
```

```
# ll
-rw-r--r-- 1 root   root   75497472 Nov  1 15:36 sectorRsv_main_backup.img
```
简单计算一下读取出来的`sectorRsv_main_backup.img`大小是否符合预期：

``` bash
>>> 147456*512
75497472
```

可知读取大小符合预期。



### 使用gparted工具压缩第三个扇区的空闲空间

在Ubuntu PC上安装`gparted`工具:

```
apt-get update
apt-get install gparted
```

在命令行执行`gparted`打开工具的图形界面如下，在右上角选择要操作的磁盘，注意一定要选正确，对应SD卡的磁盘，如果不确定，可以通过插拔SD卡来判断对应的是哪个磁盘，这里选择的是`/dev/sda`：

![输入图片说明](https://foruda.gitee.com/images/1667288516936490174/94638164_9317615.png "屏幕截图")

在`/dev/sda3`分区点击鼠标右键，弹出如下列表，点击`卸载(U)`，将该分区先卸载：

![输入图片说明](https://foruda.gitee.com/images/1667288553773216726/58107f78_9317615.png "屏幕截图")

卸载完成后，再在`/dev/sda3`分区点击鼠标右键，弹出如下列表

![输入图片说明](https://foruda.gitee.com/images/1667288652506329338/1c833e8c_9317615.png "屏幕截图")

![输入图片说明](https://foruda.gitee.com/images/1667288682964109963/a1ff41fb_9317615.png "屏幕截图")

![输入图片说明](https://foruda.gitee.com/images/1667288730091336237/5741b830_9317615.png "屏幕截图")

![输入图片说明](https://foruda.gitee.com/images/1667288800313253504/bb974a9f_9317615.png "屏幕截图")
![输入图片说明](https://foruda.gitee.com/images/1667288834355224528/71c139da_9317615.png "屏幕截图")

压缩过程的详细信息导出如下：

``` 
GParted 1.3.1

configuration --enable-libparted-dmraid --enable-online-resize

libparted 3.4

========================================

Device:	/dev/nvme0n1
型号:	SAMSUNG MZVLB1T0HBLR-00000
序列号：	
扇区大小：	512
总扇区数：	2000409264
 
磁头数:	255
扇区/磁道数：	2
柱面数：	3922371
 
分区表：	gpt
 
分区	Type	Start	End	标识	Partition Name	文件系统	卷标	挂载点
/dev/nvme0n1p1	Primary	2048	206847	boot, esp	EFI system partition	fat32		/boot/efi
/dev/nvme0n1p2	Primary	206848	239615	msftres	Microsoft reserved partition	未知		
/dev/nvme0n1p3	Primary	239616	677306151	msftdata	Basic data partition	ntfs		
/dev/nvme0n1p4	Primary	677306368	1449609215	msftdata	Basic data partition	ntfs	新加卷	
/dev/nvme0n1p6	Primary	1449609216	1999237119			ext4		/, /var/snap/firefox/common/host-hunspell
/dev/nvme0n1p5	Primary	1999237120	2000406527	hidden, diag		ntfs		
========================================

Device:	/dev/sda
型号:	Mass Storage Device
序列号：	
扇区大小：	512
总扇区数：	62333952
 
磁头数:	255
扇区/磁道数：	2
柱面数：	122223
 
分区表：	msdos
 
分区	Type	Start	End	标识	Partition Name	文件系统	卷标	挂载点
/dev/sda1	Primary	2048	29362175			ext3	ubuntu_fs	/media/yangbo/ubuntu_fs
/dev/sda2	Primary	29362176	31459327			ext3	ubuntu_fs	/media/yangbo/ubuntu_fs1
/dev/sda3	Primary	31459328	62186495			ext3	ubuntu_fs	
========================================

将 /dev/sda3 由 14.65 GiB 缩小至 70.00 MiB  00:00:35    ( 成功 )
    	
校准 /dev/sda3  00:00:01    ( 成功 )
    	
路径：/dev/sda3 (分区)
起始位置：31459328
终止位置：62186495
大小：30727168（14.65 GiB）
检查 /dev/sda3 上的文件系统错误并在可能的情况下修正  00:00:16    ( 成功 )
    	
e2fsck -f -y -v -C 0 '/dev/sda3'  00:00:16    ( 成功 )
    	
第 1 遍：检查 inode、块，和大小
第 2 遍：检查目录结构
第 3 遍：检查目录连接性
第 4 遍：检查引用计数
第 5 遍：检查组概要信息

24 个已使用的 inode（0.00%，总共 966656）
0 个不连续的文件（0.0%）
0 个不连续的目录（0.0%）
含有一次/二次/三次间接块的 inode 数：0/0/0
85721 个已使用的块（2.23%，总共 3840896）
0 个坏块
1 个大文件

6 个普通文件
9 个目录
0 个字符设备文件
0 个块设备文件
0 个队列文件
0 个链接
0 个符号链接 （0 个快速符号链接）
0 个套接字文件
------------
15 个文件
e2fsck 1.46.5 (30-Dec-2021)
缩小文件系统  00:00:17    ( 成功 )
    	
resize2fs -p '/dev/sda3' 71680K  00:00:17    ( 成功 )
    	
将 /dev/sda3 上的文件系统调整为 17920 个块（每块 4k）。
开始第 2 遍（最多 1 遍）
正在重定位块 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
开始第 3 遍（最多 118 遍）
正在扫描 inode 表 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
/dev/sda3 上的文件系统大小已经调整为 17920 个块（每块 4k）。

resize2fs 1.46.5 (30-Dec-2021)
将分区由 14.65 GiB 缩小至 70.00 MiB  00:00:01    ( 成功 )
    	
旧的起始位置：31459328
旧的终止位置：62186495
旧的大小：30727168（14.65 GiB）
新起始位置：31459328
新的终止位置：31602687
新大小：143360（70.00 MiB）
```

```
Disk /dev/sda：29.72 GiB，31914983424 字节，62333952 个扇区
Disk model: Storage Device  
单元：扇区 / 1 * 512 = 512 字节
扇区大小(逻辑/物理)：512 字节 / 512 字节
I/O 大小(最小/最佳)：512 字节 / 512 字节
磁盘标签类型：dos
磁盘标识符：0x73628cb9

设备       启动     起点     末尾     扇区 大小 Id 类型
/dev/sda1           2048 29362175 29360128  14G 83 Linux
/dev/sda2       29362176 31459327  2097152   1G 83 Linux
/dev/sda3       31459328 31602687   143360  70M 83 Linux
```

3. write sectorRsv_main_backup.img to the end of sd fan
```
# dd if=sectorRsv_main_backup.img of=/dev/sda seek=31602688
记录了147456+0 的读入
记录了147456+0 的写出
75497472字节（75 MB，72 MiB）已复制，11.5758 s，6.5 MB/s
```


4. modify partition header:
``` bash
# bash writePartitionHeader.sh /dev/sda 512 31602688
记录了0+1 的读入
记录了0+1 的写出
4字节已复制，0.000296664 s，13.5 kB/s
记录了0+1 的读入
记录了0+1 的写出
4字节已复制，0.000127065 s，31.5 kB/s
记录了0+1 的读入
记录了0+1 的写出
96字节已复制，0.000121041 s，793 kB/s
记录了0+1 的读入
记录了0+1 的写出
96字节已复制，0.000160865 s，597 kB/s
```


writePartitionHeader.sh:
``` bash
#!/bin/bash

# ************************writePartitionHeader**************************************
# Description:  write partirion header
# ******************************************************************************
function writePartitionHeader()
{
    #sector 512��?��?��?
    secStart=16
    MAIN_HEADER=$(printf "%#x" $COMPONENTS_MAIN_OFFSET)
    BACK_HEADER=$(printf "%#x" $COMPONENTS_BACKUP_OFFSET)

    MAIN_A=$(printf "%x" $(( ($MAIN_HEADER & 0xFF000000) >> 24 )))
    MAIN_B=$(printf "%x" $(( ($MAIN_HEADER & 0x00FF0000) >> 16 )))
    MAIN_C=$(printf "%x" $(( ($MAIN_HEADER & 0x0000FF00) >> 8)))
    MAIN_D=$(printf "%x" $(( $MAIN_HEADER & 0x000000FF )))

    BACKUP_A=$(printf "%x" $(( ($BACK_HEADER & 0xFF000000) >> 24 )))
    BACKUP_B=$(printf "%x" $(( ($BACK_HEADER & 0x00FF0000) >> 16 )))
    BACKUP_C=$(printf "%x" $(( ($BACK_HEADER & 0x0000FF00) >> 8)))
    BACKUP_D=$(printf "%x" $(( $BACK_HEADER & 0x000000FF )))

    #echo 55AA55AA | xxd -r -ps > magic
    echo -e -n "\x55\xAA\x55\xAA" > magic

    echo -e -n "\x$MAIN_D\x$MAIN_C\x$MAIN_B\x$MAIN_A" > components_main_base
    echo 0000 0000 0000 0000 0000 0000\
        0004 0000 0000 0000 0008 0000 0000 0000\
        0004 0000 0000 0000 0010 0000 0000 0000\
        0010 0000 0000 0000 0020 0000 0000 0000\
        0000 0100 0000 0000 0000 0000 0000 0000\
        0000 0000 0000 0000 0000 0000 0000 0000 | xxd -r -ps >> components_main_base

    echo -e -n "\x$BACKUP_D\x$BACKUP_C\x$BACKUP_B\x$BACKUP_A" > components_backup_base
    echo 0000 0000 0000 0000 0000 0000\
        0004 0000 0000 0000 0008 0000 0000 0000\
        0004 0000 0000 0000 0010 0000 0000 0000\
        0010 0000 0000 0000 0020 0000 0000 0000\
        0000 0100 0000 0000 0000 0000 0000 0000\
        0000 0000 0000 0000 0000 0000 0000 0000 | xxd -r -ps >> components_backup_base

    dd if=magic of=${DEV_NAME} count=1 seek=$[secStart] bs=$sectorSize
    dd if=magic of=${DEV_NAME} count=1 seek=$[secStart+1] bs=$sectorSize
    dd if=components_main_base of=${DEV_NAME} count=1 seek=$[secStart+2] bs=$sectorSize
    dd if=components_backup_base of=${DEV_NAME} count=1 seek=$[secStart+3] bs=$sectorSize

    rm -rf magic
    rm -rf components_main_base
    rm -rf components_backup_base
}



DEV_NAME=$1
sectorSize=$2
COMPONENTS_MAIN_OFFSET=$3
COMPONENTS_BACKUP_OFFSET=$[COMPONENTS_MAIN_OFFSET+73728]

writePartitionHeader

```


now check whether this sd card can boot on Atlas 200DK.
启动后切换root用户，`fdisk -l`命令查看`/dev/mmcblk1p3`分区正确的压缩了70M。

``` bash
root@davinci-mini:/home/HwHiAiUser# fdisk -l
Disk /dev/mmcblk1: 29.7 GiB, 31914983424 bytes, 62333952 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes
Disklabel type: dos
Disk identifier: 0x73628cb9

Device         Boot    Start      End  Sectors Size Id Type
/dev/mmcblk1p1          2048 29362175 29360128  14G 83 Linux
/dev/mmcblk1p2      29362176 31459327  2097152   1G 83 Linux
/dev/mmcblk1p3      31459328 31602687   143360  70M 83 Linux

```


5. 将SD卡通过读卡器插入Ubuntu PC，打开SD卡的文件系统，将sd_expand.sh扩容脚本拷贝到/var/目录下面，然后修改`/etc/rc.local`文件如下：
/etc/rc.local 检查并调用自动扩容脚本：
``` bash
#!/bin/sh -e
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will exit 0 on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.
cd /var/


/bin/bash /var/minirc_boot.sh /opt/mini/A200dk-npu-driver-21.0.4-ubuntu18.04-aarch64-minirc.tar.gz

if [ ! -e /var/sd_expanded ]; then
    /bin/bash /var/sd_expand.sh
fi

exit 0
```


首次启动自动扩容脚本`sd_expand.sh`如下：
``` bash 
#!/bin/bash

ASCEND_SECLOG="/var/log/ascend_seclog"
bootLogFile="${ASCEND_SECLOG}/ascend_run_servers.log"

log() {
    cur_date=`date +"%Y-%m-%d %H:%M:%S"`
    echo "[$cur_date] [SD CARD EXPAND] "$1 >> $bootLogFile
}

MMCBLK1_PATH="/dev/mmcblk1"
CURRENT_P3_SIZE=""
CHECK_HOME_MOUNT=""
USED_MOUNT_SIZE=""
CHECK_DISK_SIZE=""
SD_EXPANSION_SZIE=""

function check_parted_exsits()
{
    ret=$(type -p parted)
    if [[ ! -f ${ret} ]]; then
        log "parted is not existed, please install it first"
        return 1
    fi
    return 0
}

function check_bc_exsits()
{
    ret=$(type -p bc)
    if [[ ! -f ${ret} ]]; then
        log "bc is not existed, please install it first"
        return 1
    fi
    return 0 = 512 字节
扇区大小(逻辑/物理)：512 字节 / 512 字节
I/O 大小(最小/最佳)：512 字节 / 512 字节
磁盘标签类型：dos
磁盘标识符：0x73628cb9

设备       启动     起点     末尾     扇区 大小 Id 类型
/dev/sda1           2048 29362175 29360128  14G 83 Linux
/dev/sda2       29362176 31459327  2097152   1G 83 Linux
/dev/sda3       31459328 31602687   143360  70M 83 Linux
root@yangbo-MACHC-WAX9:/home/yangbo/work/image_expand#
}

function check_expect_exsits()
{
    ret=$(type -p expect)
    if [[ ! -f ${ret} ]]; then
        log "expect is not existed, please install it first"
        return 1
    fi
    return 0
}

function get_disk_capacity()
{sectorRsv_main_backup
    CHECK_DISK_SIZE=$(fdisk -l | awk '{if($2=="/dev/mmcblk1:"){print $7}}')
    CURRENT_P3_SIZE=$(fdisk -l | awk '{if($1=="/dev/mmcblk1p3"){print $4}}')
    check_bc_exsits
    if [[ $? != 0 ]]; then
        return 1
    fi

    USED_MOUNT_SIZE=$(echo $(fdisk -l | awk '{if($1=="/dev/mmcblk1p3"){print $3}}') + 1 + 147456 |bc)
    return 0
}

function calculate_expansion_size()
{
    total_sd_size=$(fdisk -l | awk '{if($2=="/dev/mmcblk1:"){print $5}}')
    SD_EXPANSION_SZIE=$(echo "${total_sd_size} - (72 * 1024 * 1024) - 512"|bc)

    return 0
}

function set_home_capacity_expansion()
{
    check_parted_exsits
    if [[ $? -ne 0 ]]; then
        return 1
    fi

    check_expect_exsits
    if [[ $? -ne 0 ]]; then
        return 1
    fi

    calculate_expansion_size
    if [[ $? -ne 0 ]]; then
        return 1
    fi

expect <<-END
spawn parted ${MMCBLK1_PATH}
expect "parted"
send "p\n"
expect "parted"
send "resizepart 3\n"
expect "Yes"
send "Yes\r"
expect "End"
send "${SD_EXPANSION_SZIE}B\n"
expect "parted"
send "q\n"
expect eof
exit
END

    resize2fs /dev/mmcblk1p3
    log "home expansion success!"
    return 0
}

function main()
{
    get_disk_capacity
    ret=$?
    if [[ "${ret}" -ne 0 ]]; then
        return "${ret}"
    fi

    check_bc_exsits
    if [[ $? -ne 0 ]]; then
        return 1
    fi

    if [[ $(echo "${USED_MOUNT_SIZE} >= ${CHECK_DISK_SIZE}"|bc) -eq 1 ]]; then
        log "used size has been equal disk size, can not expansion!"
        return 1
    fi

    log "the home can expansion"

    set_home_capacity_expansion
    if [[ $? -ne 0 ]]; then
        return 1
    fi

    return 0
}

main
if [[ $? -ne 0 ]]; then
    log "sd card expand failed!"
else
    log "sd card expand success!"
    touch /var/sd_expanded
    echo "successful" > /var/sd_expanded
fi

```


6. read image from sd to Ubuntu PC, just read head+p1+p2+p2+ COMPONENTS_MAIN + COMPONENTS_BACKUP 
```
Disk /dev/sda：29.72 GiB，31914983424 字节，62333952 个扇区
Disk model: Storage Device  
单元：扇区 / 1 * 512 = 512 字节
扇区大小(逻辑/物理)：512 字节 / 512 字节
I/O 大小(最小/最佳)：512 字节 / 512 字节
磁盘标签类型：dos
磁盘标识符：0x73628cb9

设备       启动     起点     末尾     扇区 大小 Id 类型
/dev/sda1           2048 29362175 29360128  14G 83 Linux
/dev/sda2       29362176 31459327  2097152   1G 83 Linux
/dev/sda3       31459328 31602687   143360  70M 83 Linux
```
``` 
>>> 73728*2
147456
>>> 31602687+1+147456
31750144
>>> 

```


```
dd if=/dev/sda of=auto_expand_sd.img bs=512 count=31750144
记录了31750144+0 的读入
记录了31750144+0 的写出
16256073728字节（16 GB，15 GiB）已复制，905.08 s，18.0 MB/s

```






7. Write the image to another sd card to check whether can boot successful. 


8. calculate sha256sum of image
```
# sha256sum auto_expand_sd.img > auto_expand_sd.img.sha256

# cat auto_expand_sd.img.sha256 
3affe7fad9b98a9fdc85931e873959c9c39e85d830eee390b1d9f0177e6a654d  auto_expand_sd.img
```

9. zip the image, upload to huawei OBS, use Ascend-imager tool to download and write to a SD card.

```
zip auto_expand_sd.img.zip auto_expand_sd.img
```



10. 通过**昇腾开发者套件镜像烧录工具**，将上述上传OBS的镜像写入SD卡，然后插入A200DK，上电，查看是否能够正常启动，启动后登录系统，查看SD卡扩容情况：
    查看扩容是否成功：

    ```
    # cat /var/sd_expanded
    successful
    ```

    查看磁盘使用情况，可以发现镜像的第三分区已经扩展到了整张SD卡：

    ``` bash
    root@davinci-mini:/var# df -h
    Filesystem      Size  Used Avail Use% Mounted on
    /dev/root        14G  875M   13G   7% /
    devtmpfs        3.6G  4.0K  3.6G   1% /dev
    tmpfs           3.8G     0  3.8G   0% /dev/shm
    tmpfs           3.8G  320K  3.8G   1% /run
    tmpfs           5.0M     0  5.0M   0% /run/lock
    tmpfs           3.8G     0  3.8G   0% /sys/fs/cgroup
    /dev/mmcblk1p2  976M  1.9M  923M   1% /var/log/npu/slog
    /dev/mmcblk1p3  102G   51M  101G   1% /home
    tmpfs           777M     0  777M   0% /run/user/1000
    root@davinci-mini:/var# fdisk -l
    Disk /dev/mmcblk1: 117.8 GiB, 126437294080 bytes, 246947840 sectors
    Units: sectors of 1 * 512 = 512 bytes
    Sector size (logical/physical): 512 bytes / 512 bytes
    I/O size (minimum/optimal): 512 bytes / 512 bytes
    Disklabel type: dos
    Disk identifier: 0xb1275e40
    
    Device         Boot    Start       End   Sectors   Size Id Type
    /dev/mmcblk1p1          2048  29362175  29360128    14G 83 Linux
    /dev/mmcblk1p2      29362176  31459327   2097152     1G 83 Linux
    /dev/mmcblk1p3      31459328 246800383 215341056 102.7G 83 Linux
    
    root@davinci-mini:/var# python3
    Python 3.6.9 (default, Jan 26 2021, 15:33:00)
    [GCC 8.4.0] on linux
    Type "help", "copyright", "credits" or "license" for more information.
    >>> 246800383+1+73728*2
    246947840
    ```

​    如果失败了，定位问题请查看`/var/log/ascend_seclog/ascend_run_servers.log`日志文件。

