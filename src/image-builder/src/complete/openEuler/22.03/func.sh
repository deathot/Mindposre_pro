#!/usr/bin/env bash

# ==============================
# 1. 预处理
# ==============================

# 预处理函数
pre_process() {
  set -e
  log "Begin to conduct pre-process."
  run_command sed -i '/export LD_LIBRARY_PATH=\/home\/HwHiAiUser\/Ascend\/acllib\/lib64/d' /home/HwHiAiUser/.bashrc
}

# ==============================
# 2. dnf
# 本部分对dnf所需依赖进行下载
# ==============================

# dnf依赖数组
# 镜像压缩和扩容所需依赖
DNF_DEPENDENCIES+=("dosfstools" "dos2unix" "expect" "parted" "dump")
# 系统所需依赖
DNF_DEPENDENCIES+=("git" "gcc" "g++" "make" "wget" "yum" "python3-devel")
# 视频解析以及外设等依赖
DNF_DEPENDENCIES+=("ffmpeg" "v4l-utils")
# Wi-Fi所需依赖
DNF_DEPENDENCIES+=("NetworkManager-wifi")

# dnf下载函数
dnf_install() {
  set -e
  log "Begin to install dnf dependencies."
  dnf_install_packages "${DNF_DEPENDENCIES[@]}"
  log "Finish installing dnf dependencies."
}

# dnf下载工具函数
dnf_install_packages() {
  local dnf_required=("$@")
  local dnf_installed
  local dnf_to_install

  for package in "${dnf_required[@]}"; do
      if [[ "$package" == *\** ]]; then
          dnf_to_install+=("$package")
      elif dnf list installed "$package" >/dev/null 2>&1; then
          dnf_installed+=("$package")
      else
          dnf_to_install+=("$package")
      fi
  done

  if [ ${#dnf_installed[@]} -ne 0 ]; then
      log "These packages have been installed: ${dnf_installed[*]}."
  fi

  if [ ${#dnf_to_install[@]} -ne 0 ]; then
      log "These packages are to install: ${dnf_to_install[*]}"
      run_command dnf install -y "${dnf_to_install[@]}"
  else
      log "No packages to install."
  fi
}

# ==============================
# 3. python依赖
# 选择使用原生的python3.9.11
# ==============================

# pip依赖数组
# pip 第三方依赖
PIP_DEPENDENCIES=("torch==1.13.0" "torchaudio==0.13.0" "torchvision==0.14.0" "scikit-video" "ipywidgets" "ffmpeg-python" "IPython==7.34.0" "numpy==1.22.4" "albumentations" "jupyterlab" "matplotlib" "pycocotools" "protobuf==3.20.0")
# CANN pip依赖
PIP_DEPENDENCIES+=("attrs" "numpy" "decorator" "cffi" "pyyaml" "pathlib2" "psutil" "protobuf" "scipy" "requests" "absl-py")
# atc python依赖
PIP_DEPENDENCIES+=("sympy")

# pip配置以及下载函数
python_pip_install() {
  set -e
  log "Begin to install python dependencies."

  # 创建python软链接
  ln -s /usr/bin/python3.9 /usr/bin/python
  chmod 777 /usr/bin/python

  # 若不存在，则新建~/.pip目录
  run_command mkdir -p ~/.pip

  # 配置pip下载源为豆瓣源
  log "Modify pip downloading sources."
  local pip_source_cfg
  pip_source_cfg=$(jq -r '.python.source' "$config_path")
  if [ "$pip_source_cfg" != "" ]; then
    printf "%s" "$pip_source_cfg" | sed "s/'\|\"//g" > ~/.pip/pip.conf
  fi

  log "Installing pip dependencies."
  run_command pip install --no-input "${PIP_DEPENDENCIES[@]}"

  # 下载安装aclruntime
  install_pip_dependency_from_config "aclruntime" "acl-runtime"
  # 下载安装ais_bench
  install_pip_dependency_from_config "ais_bench" "ais_bench"

  log "Finish installing python dependencies."
}

# 根据配置文件下载python依赖
install_pip_dependency_from_config() {
    local dependency_name="$1"
    local config_key="$2"
    local file_path
    file_path=$(download_dependencies "$dependency_name" "$config_key")
    run_command pip install "$file_path" --no-input
}

# ==============================
# 4. CANN,mxVision,acllite等昇腾依赖依赖
# mxVision安装需要CANN，需要先安装CANN再安装mxVision
# acllite直接下载源码复制进去即可
# ==============================

# CANN 环境变量脚本路径
CANN_ENV_PATH="$path_install"/Ascend/ascend-toolkit/set_env.sh
# 昇腾环境变量脚本路径
ASCEND_DEVKIT_PATH=/etc/profile.d/ascend-devkit.sh

# CANN 下载安装函数
install_cann() {
  log "Begin to add CANN."

  # 下载 CANN
  log "Download CANN."
  local cann_file_path
  cann_file_path=$(download_dependencies "cann" "cann")

  # 安装 CANN
  log "Install CANN."
  run_command chmod +x "$cann_file_path"
  run_command "$cann_file_path" --quiet --install --install-for-all --force

  if [ ! -e "$ASCEND_DEVKIT_PATH" ] || ! grep -q "ascend-toolkit" "$ASCEND_DEVKIT_PATH"; then
    printf "source %s\n" "$CANN_ENV_PATH" >> "$ASCEND_DEVKIT_PATH"
  fi
  log "Finish adding CANN."
}

# mxVision 安装路径
MXVISION_INSTALL_PATH="$HOME/Ascend"

# mxVision 下载安装函数
install_mxvision() {
  log "Begin to add mxVision."
  # 下载并安装mxvision
  log "Download mxVision."
  if [ -f "$CANN_ENV_PATH" ]; then source "$CANN_ENV_PATH"; fi

  local mxvision_file_path
  mxvision_file_path=$(download_dependencies "mxvision" "mxvision")

  log "Install mxVision."
  run_command chmod +x "$mxvision_file_path"
  run_command "$mxvision_file_path" --quiet --install --install-path="$MXVISION_INSTALL_PATH"

  log "Finish adding mxVision."
}

# acllite安装路径
ACLLITE_INSTALL_PATH="$path_install/Ascend/thirdpart/aarch64/"
# samples代码仓库网址
SAMPLES_REPO_URL="https://gitee.com/ascend/samples.git"

# acllite 下载安装函数
install_acllite() {
  set -e
  log "Begin to add acllite."

  # 下载 acllite
  log "Download acllite."
  mkdir -p "$ACLLITE_INSTALL_PATH"
  if [ -z "$ACLLITE_PATH" ]; then
    if [ ! -e "$path_download"/samples ]; then
        run_command git clone -b v0.8.0 "$SAMPLES_REPO_URL" "$path_download"/samples
    fi
    log "Copy acllite to ascend path."
    run_command cp -r "$path_download"/samples/python/common/acllite "$ACLLITE_INSTALL_PATH"
  fi
  log "Finish adding acllite."
}

# ==============================
# 5. 音频与图形化界面
# 音频需要替换内核
# 本地图形化界面需要替换dtb
# 本地和远程图形化界面均采用xfce
# ==============================

# 音频dnf依赖数组
DNF_AUDIO_DEPENDENCIES=("alsa-tools" "alsa-utils")

# 音频安装函数
add_audio() {
  set -e
  log "Begin to build audio function."

  # 下载要更换的内核部分
  log "Download kernel for replacing."
  local path_audio="$path_download"/audio

  if [ ! -e "$path_audio" ] || [ -z "$(ls -A "$path_audio")" ] > /dev/null 2>&1; then
    local audio_zip_path
    audio_zip_path=$(download_dependencies "audio" "audio")
    path_audio_backup=$path_audio\.backup
    run_command unzip "$audio_zip_path" -d "$path_audio_backup"
    run_command mv "$path_audio_backup" "$path_audio"
  fi

  # 替换内核
  log "Replace kernel."
  run_command dd if="$path_audio"/Image of=/dev/mmcblk1 count=61440 seek=32768 bs=512

  # 下载音频所需依赖
  dnf_install_packages "${DNF_AUDIO_DEPENDENCIES[@]}"

  log "Finish building audio function."
}

# 本地图形化界面依赖
# 安装字库
DNF_LOCAL_DESKTOP_DEPENDENCIES+=("dejavu-fonts" "liberation-fonts" "gnu-*-fonts" "google-*-fonts")
# 安装Xorg
DNF_LOCAL_DESKTOP_DEPENDENCIES+=("xorg-*")
# 安装XFCE及组件
DNF_LOCAL_DESKTOP_DEPENDENCIES+=("xfwm4" "xfdesktop" "xfce4-*" "xfce4-*-plugin" "network-manager-applet" "*fonts")
# 安装登录管理器
DNF_LOCAL_DESKTOP_DEPENDENCIES+=("lightdm" "lightdm-gtk")
# 安装火狐
DNF_LOCAL_DESKTOP_DEPENDENCIES+=("firefox")
# 安装中文输入法
DNF_LOCAL_DESKTOP_DEPENDENCIES+=("ibus-libpinyin")

# 本地图形化界面
add_local_desktop() {
  set -e
  log "Begin to build local desktop function."
  # 安装所需依赖
  log "Begin to install dnf dependencies."
  dnf_install_packages "${DNF_LOCAL_DESKTOP_DEPENDENCIES[@]}"

  # 更新lib和ko
  log "Download graphic libs and kos."
  local path_graphic="$path_download"/graphic
  if [ ! -e "$path_graphic" ] || [ -z "$(ls -A "$path_graphic")" ] > /dev/null 2>&1; then
    local graphic_zip_path
    graphic_zip_path=$(download_dependencies "graphic" "graphic")
    path_graphic_backup=$path_graphic\.backup
    run_command unzip "$graphic_zip_path" -d "$path_graphic_backup"
    run_command mv "$path_graphic_backup" "$path_graphic"
  fi

  # 更换dtb
  log "Update dtb."
  run_command dd if="$path_graphic"/dt.img of=/dev/mmcblk1 count=4096 seek=114688 bs=512
  run_command dd if="$path_graphic"/dt.img of=/dev/mmcblk1 count=4096 seek=376832 bs=512

  # 复制ko到相应路径
  if [ ! -f /lib/modules/5.10.0+/ascend_vdp_drm.ko  ]; then
    run_command cp "$path_graphic"/ascend_vdp_drm.ko /lib/modules/5.10.0+/
  fi
  if [ "$(lsmod | grep ascend_vdp_drm)" == "" ]; then
      run_command insmod /lib/modules/5.10.0+/ascend_vdp_drm.ko
  fi

  # 配置本地桌面服务，设置图形桌面驱动自动加载
  log "Create and turn on local desktop service."
  printf "#\041/bin/bash
insmod /lib/modules/5.10.0+/ascend_vdp_drm.ko
nmcli radio wifi on
systemctl restart display-manager.service
" > /usr/local/bin/toggle_graphical_desktop.sh
  run_command chmod +x /usr/local/bin/toggle_graphical_desktop.sh
  printf "[Unit]
Description=%s
Wants=start-davinci.service
After=start-davinci.service

[Service]
ExecStart=%s
Restart=on-failure
RestartSec=10s

[Install]
WantedBy=multi-user.target" /usr/local/bin/toggle_graphical_desktop.sh /usr/local/bin/toggle_graphical_desktop.sh > /etc/systemd/system/toggle_graphical_desktop.service
  run_command systemctl daemon-reload

  # 设置默认桌面为XFCE 通过root权限用户设置
  if ! grep -q "user-session=xfce" /etc/lightdm/lightdm.conf.d/60-lightdm-gtk-greeter.conf; then
    printf 'user-session=xfce' >> /etc/lightdm/lightdm.conf.d/60-lightdm-gtk-greeter.conf
  fi
  # 设置桌面服务自启动
  run_command systemctl enable toggle_graphical_desktop
  # 设置开机自启动图形界面
  run_command systemctl enable lightdm
  # 设置系统默认以图形界面登录
  run_command systemctl set-default graphical.target

  # 更换默认终端
  local highest_priority_terminal
  highest_priority_terminal=$(update-alternatives --display x-terminal-emulator | grep priority | sort -k4 -n -r | head -n1 | cut -d' ' -f1)
  if [[ $highest_priority_terminal != "/usr/bin/xfce4-terminal" ]]; then
    # 如果不是/usr/bin/xfce4-terminal，则获取最高的priority值
    local highest_priority
    highest_priority=$(update-alternatives --display x-terminal-emulator | grep priority | awk '{print $4}' | sort -n | tail -n1)
    # 如果非数字，则默认设置为0
    if ! [[ $highest_priority =~ ^[0-9]+$ ]]; then
      highest_priority=0
    fi
    # 将/usr/bin/xfce4-terminal的权限设置为最高加一
    local xfce4_terminal_priority
    xfce4_terminal_priority=$((highest_priority+1))
    # 确保xfce4-terminal在alternatives列表中并设置priority
    update-alternatives --install /usr/bin/x-terminal-emulator x-terminal-emulator /usr/bin/xfce4-terminal $xfce4_terminal_priority
    # 设置xfce4-terminal为默认terminal emulator
    update-alternatives --set x-terminal-emulator /usr/bin/xfce4-terminal
  fi

  # 添加壁纸
  run_command cp "$path_graphic"/ascend-wallpaper.png /usr/share/backgrounds/xfce/

  # 添加中文输入法自启
  if ! grep -q "ibus" /home/HwHiAiUser/.bashrc; then
    printf "# Use ibus as input method engine
export GTK_IM_MODULE=ibus
export QT_IM_MODULE=ibus
export XMODIFIERS=@im=ibus
ibus-daemon -drx" >> /home/HwHiAiUser/.bashrc
  fi
}

# 构建远程桌面依赖数组
# 安装字库
DNF_REMOTE_DESKTOP_DEPENDENCIES+=("dejavu-fonts" "liberation-fonts" "gnu-*-fonts" "google-*-fonts")
# 安装Xorg
DNF_REMOTE_DESKTOP_DEPENDENCIES+=("xorg-*")
# 安装XFCE及组件
DNF_REMOTE_DESKTOP_DEPENDENCIES+=("xfwm4" "xfdesktop" "xfce4-*" "xfce4-*-plugin" "network-manager-applet" "*fonts")
# 安装火狐
DNF_REMOTE_DESKTOP_DEPENDENCIES+=("firefox")
# 安装中文输入法
DNF_REMOTE_DESKTOP_DEPENDENCIES+=("ibus-libpinyin")
# 安装VNC
DNF_REMOTE_DESKTOP_DEPENDENCIES+=("tigervnc-server")

add_remote_desktop() {
  set -e
  log "Begin to build remote desktop function."
  log "Begin to install remote desktop dependencies."
  dnf_install_packages "${DNF_REMOTE_DESKTOP_DEPENDENCIES[@]}"

  # 配置vnc server启动环境
  log "Create and turn on vnc server service."
  # 文件路径
  run_command mkdir -p "$HOME"/.vnc
  # 创建并写入文件内容
  printf "#!/bin/bash
startxfce4 &" > "$HOME/.vnc/xstartup"
  run_command chmod +x "$HOME/.vnc/xstartup"
  # vnc 自启动
  printf "[Unit]
Description=Start TightVNC server at startup
After=syslog.target network.target

[Service]
Type=forking
User=root
PAMName=login
PIDFile=/home/%%u/.vnc/%%H:%%i.pid
ExecStartPre=-/usr/bin/vncserver -kill :%%i > /dev/null 2>&1
ExecStart=/usr/bin/vncserver -depth 24 -geometry 1920x1080 :%%i
ExecStop=/usr/bin/vncserver -kill :%%i

[Install]
WantedBy=multi-user.target" > /etc/systemd/system/vncserver@.service
  run_command systemctl daemon-reload
  if [ "$(systemctl is-enabled vncserver@1.service)" != "enabled" ]; then
    run_command systemctl enable vncserver@1.service
  fi

  # 提示修改密码时设置vnc server
  log "Set password for vnc server"
  local password="Mind@123"
  local expect_script
  expect_script=$(cat <<- END
  spawn vncserver
  expect "Password:*"
  send "$password\r"
  expect "Verify:*"
  send "$password\r"
  expect "Would you like to enter a view-only password (y/n)?*"
  send "n\r"
  exit
END
  )
  if [ ! -e "$HOME"/.vnc/passwd ]; then
    if [ "$FLAG_VERBOSE" == true ]; then
      expect <<< "$expect_script"
    else
      expect <<< "$expect_script" > /dev/null 2>&1
    fi
  fi

  # 更换默认终端
  local highest_priority_terminal
  highest_priority_terminal=$(update-alternatives --display x-terminal-emulator | grep priority | sort -k4 -n -r | head -n1 | cut -d' ' -f1)
  if [[ $highest_priority_terminal != "/usr/bin/xfce4-terminal" ]]; then
    # 如果不是/usr/bin/xfce4-terminal，则获取最高的priority值
    local highest_priority
    highest_priority=$(update-alternatives --display x-terminal-emulator | grep priority | awk '{print $4}' | sort -n | tail -n1)
    # 如果非数字，则默认设置为0
    if ! [[ $highest_priority =~ ^[0-9]+$ ]]; then
      highest_priority=0
    fi
    # 将/usr/bin/xfce4-terminal的权限设置为最高加一
    local xfce4_terminal_priority
    xfce4_terminal_priority=$((highest_priority+1))
    # 确保xfce4-terminal在alternatives列表中并设置priority
    update-alternatives --install /usr/bin/x-terminal-emulator x-terminal-emulator /usr/bin/xfce4-terminal $xfce4_terminal_priority
    # 设置xfce4-terminal为默认terminal emulator
    update-alternatives --set x-terminal-emulator /usr/bin/xfce4-terminal
  fi

  # 修改远程桌面壁纸
  log "Modify remote desktop wallpaper."
  local path_graphic="$path_download"/graphic
  if [ ! -e "$path_graphic" ] || [ -z "$(ls -A "$path_graphic")" ] > /dev/null 2>&1; then
    local graphic_zip_path
    graphic_zip_path=$(download_dependencies "graphic" "graphic")
    path_graphic_backup=$path_graphic\.backup
    run_command unzip "$graphic_zip_path" -d "$path_graphic_backup"
    run_command mv "$path_graphic_backup" "$path_graphic"
  fi
  run_command cp "$path_graphic"/ascend-wallpaper.png /usr/share/backgrounds/xfce/

  # 添加中文输入法自启
  if [ ! -e "$HOME"/.vnc/ibus-autostartup ]; then
    printf "export GTK_IM_MODULE=ibus
export QT_IM_MODULE=ibus
export XMODIFIERS=@im=ibus
ibus-daemon -drx" > "$HOME"/.vnc/ibus-autostartup
    chmod 777 "$HOME"/.vnc/ibus-autostartup
  fi
}

# 添加WiFi
add_wifi() {
  set -e
  log "Begin to build wifi function."
  log "Begin to install wifi dependencies."
  local path_wifi="$path_download"/wifi
  if [ ! -e "$path_wifi" ] || [ -z "$(ls -A "$path_wifi")" ]; then
    local wifi_zip_path
    wifi_zip_path=$(download_dependencies "wifi" "wifi")
    path_wifi_backup=$path_wifi\.backup
    run_command unzip "$wifi_zip_path" -d "$path_wifi_backup"
    run_command mv "$path_wifi_backup" "$path_wifi"
  fi

  # 设置USB WiFi驱动自动加载
  log "Set usb wifi auto loaded."
  run_command mkdir -p /usr/lib/firmware/rtlwifi/
  run_command cp "$path_wifi"/*.bin /usr/lib/firmware/rtlwifi/
  run_command cp "$path_wifi"/*.ko /lib/modules/5.10.0+/
  printf "rfkill
cfg80211
mac80211
rtlwifi
rtl_usb
rtl8192c-common
rtl8192cu" > /etc/modules-load.d/usb-wifi-drivers.conf
  run_command depmod -a

  log "Finish building wifi function."
}

# ==============================
# 6. 后处理
# 包括镜像的一些小的功能补齐，还有整个镜像过程当中中间文件的清理
# ==============================

# 昇腾环境变量脚本路径
ASCEND_DEVKIT_PATH=/etc/profile.d/ascend-devkit.sh

# 后处理函数
post_process() {
  set -e
  log "Begin to conduct post process"
  # 添加oom_killer
  printf "#\041/bin/bash
chmod 666 /sys/fs/cgroup/memory/usermemory/tasks
echo 1 > /proc/sys/vm/enable_oom_killer
echo 0 > /sys/fs/cgroup/memory/usermemory/memory.oom_control
" > /var/oom_killer.sh
  run_command chmod +x /var/oom_killer.sh
  printf "[Unit]
Description=%s
Wants=start-davinci.service
After=start-davinci.service

[Service]
ExecStart=%s
Restart=on-failure
RestartSec=10s

[Install]
WantedBy=multi-user.target" /var/oom_killer.sh /var/oom_killer.sh> /etc/systemd/system/oom_killer.service
  if [ ! -e "$ASCEND_DEVKIT_PATH" ] || [ "$(cat < "$ASCEND_DEVKIT_PATH" | grep usermemory)" == "" ]; then
    printf "echo \$\$ > /sys/fs/cgroup/memory/usermemory/tasks\n" >> "$ASCEND_DEVKIT_PATH"
  fi
  run_command systemctl daemon-reload
  run_command systemctl enable oom_killer

  # 修改启动欢迎文本
  printf "Welcome to Atlas 200I DK A2
This system is only applicable to individual developers and cannot be used for commercial purposes.
By using this system, you have agreed to the Huawei Software License Agreement.
Please refer to the agreement for details on https://www.hiascend.com/software/protocol
Reference resources
* Home page: https://www.hiascend.com/hardware/developer-kit-a2
* Documentation: https://www.hiascend.com/hardware/developer-kit-a2/resource
* Online courses: https://www.hiascend.com/edu/courses
* Online experiments: https://www.hiascend.com/zh/edu/experiment
* Forum: https://www.hiascend.com/forum/
* Code: https://gitee.com/HUAWEI-ASCEND/ascend-devkit" > /etc/motd
  log "Post process finished."
}

# 定制化处理函数
accident_handler() {
  cleanup_temporary_files "$package_file_backup_path"
  cleanup_temporary_files "$path_download"/"$audio_zip_backup_name"
  cleanup_temporary_files "$path_download"/"$graphic_zip_backup_name"
  cleanup_temporary_files "$path_download"/"$wifi_zip_backup_name"
  cleanup_temporary_files "$path_download"
}
