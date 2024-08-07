#!/usr/bin/env bash

# ==============================
# 1. 预处理
# 修改系统区域和时间
# 删除ubuntu自带的mac地址绑定文件，以实现usb0的mac地址随机分配
# ==============================

# 预处理函数
pre_process() {
  set -e
  log "Modify local timezone."
  run_command timedatectl set-timezone Asia/Shanghai

  # 删除固定usb0mac地址的系统文件
  log "Remove fixed usb0mac address system file if exists."
  local usb0_mac_config_file="/usr/lib/systemd/network/99-default.link"
  if [ -f "$usb0_mac_config_file" ]; then
    run_command rm "$usb0_mac_config_file"
  fi
}

# ==============================
# 2. apt
# 本部分对apt所需依赖进行下载
# ==============================

# apt依赖数组
# 镜像压缩和扩容所需依赖
APT_DEPENDENCIES=("dosfstools" "dos2unix" "expect" "parted" "dump")
# 系统所需依赖
APT_DEPENDENCIES+=("git" "gcc" "g++" "software-properties-common")
# 视频解析以及外设等依赖
APT_DEPENDENCIES+=("ffmpeg" "v4l-utils")
# CANN所需依赖
APT_DEPENDENCIES+=("g++" "make" "cmake" "zlib1g" "zlib1g-dev" "openssl" "libsqlite3-dev" "libssl-dev" "libffi-dev" "unzip" "pciutils" "net-tools" "libblas-dev" "gfortran" "libblas3")
# 登录文字修改
APT_DEPENDENCIES+=("figlet" "lolcat")
# 图形化界面、Wifi所需依赖
APT_DEPENDENCIES+=("network-manager")
# 解决git Gnu TLS报错
APT_DEPENDENCIES+=("gnutls-bin")

# apt下载依赖
apt_install() {
  set -e
  log "Begin to install apt dependencies."
  apt_install_packages "${APT_DEPENDENCIES[@]}"
  log "Finish installing apt dependencies."
}

# apt下载工具函数
apt_install_packages() {
  local apt_required=("$@")
  local apt_installed
  local apt_to_install

  for package in "${apt_required[@]}"; do
      if dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q "ok installed"; then
          apt_installed+=("$package")
      else
          apt_to_install+=("$package")
      fi
  done

  if [ ${#apt_installed[@]} -ne 0 ]; then
      log "These packages have been installed: ${apt_installed[*]}."
  fi

  if [ ${#apt_to_install[@]} -ne 0 ]; then
      log "These packages are to install: ${apt_to_install[*]}"
      run_command apt-get update
      export DEBIAN_FRONTEND=noninteractive
      run_command apt-get -y install "${apt_to_install[@]}"
  else
      log "No packages to install."
  fi
}

# ==============================
# 3. miniconda
# 本部分为miniconda的依赖构建部分
# 包括下载安装、修改源、下载对应版本python、miniconda自启动等
# ==============================

# python 版本
PYTHON_VERSION="3.9.2"
# miniconda安装的目录名称
MINICONDA_NAME="miniconda3"
# miniconda目标安装绝对路径
MINICONDA_PATH="$path_install/$MINICONDA_NAME"

# miniconda配置函数
install_miniconda() {
  set -e
  log "Begin to install miniconda."
  if ! activate_conda > /dev/null 2>&1; then
    log "Download miniconda."
    local miniconda_file_path
    miniconda_file_path=$(download_dependencies "miniconda" "miniconda")
    log "Install miniconda."
    run_command bash "$miniconda_file_path" -b -f -p "$MINICONDA_PATH"

    log "Miniconda automatic initialization."
    local conda_initialize_string="# >>> conda initialize >>>
# \041\041 Contents within this block are managed by 'conda init' \041\041
__conda_setup=\"\$('$MINICONDA_PATH/bin/conda' 'shell.bash' 'hook' 2> /dev/null)\"
if [ \$? -eq 0 ]; then
   eval \"\$__conda_setup\"
else
   if [ -f \"$MINICONDA_PATH/etc/profile.d/conda.sh\" ]; then
       . \"$MINICONDA_PATH/etc/profile.d/conda.sh\"
   else
       export PATH=\"$MINICONDA_PATH/bin:\$PATH\"
   fi
fi
unset __conda_setup
# <<< conda initialize <<<\n"

  for user in root HwHiAiUser; do
    update_bashrc "$user" "conda initialize" "$conda_initialize_string"
  done

      # 修改miniconda的下载源
      log "Modify downloading sources of miniconda."
      local conda_source_cfg
      conda_source_cfg=$(jq -r '.conda.source' "$config_path")
      printf "%s" "$conda_source_cfg" > "$path_install"/.condarc
  fi

  # conda激活
  log "Activate conda."
  if ! activate_conda; then return 1; fi

  # 确保miniconda下载源已更新
  log "Clean conda downloading sources to use new sources."
  run_command conda clean -i -y

  # 下载python3.9.2
  log "Downloading python $PYTHON_VERSION."
  local current_python_version
  current_python_version=$(python --version 2>&1 | awk '{print $2}')
  if [ "$current_python_version" != "$PYTHON_VERSION" ]; then
    run_command conda config --append channels conda-forge
    # insecure 模式下载python3.9.2以绕过ssl认证
    run_command conda install python=$PYTHON_VERSION -k -y
  fi
  log "Finish installing miniconda."
}

# conda激活函数
activate_conda() {
    # conda 初始化
    local conda_setup
    if conda_setup="$("$MINICONDA_PATH/bin/conda" 'shell.bash' 'hook' 2>/dev/null)"; then
        eval "$conda_setup"
    elif [ -f "$MINICONDA_PATH/etc/profile.d/conda.sh" ]; then
        . "$MINICONDA_PATH/etc/profile.d/conda.sh"
    else
        export PATH="$MINICONDA_PATH/bin:$PATH"
    fi

    # conda 环境激活
    if ! conda activate; then return 1; fi
}

# ==============================
# 4. python依赖
# 本部分为下载安装python，配置python，和下载python依赖部分
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

  # conda激活
  if ! activate_conda; then return 1; fi

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
# 5. CANN,mxVision,acllite等昇腾依赖依赖
# mxVision安装需要CANN，需要先安装CANN再安装mxVision
# acllite直接下载源码复制进去即可
# ==============================

# CANN 环境变量脚本路径
CANN_ENV_PATH="$path_install/Ascend/ascend-toolkit/set_env.sh"

# CANN 下载安装函数
install_cann() {
  set -e
  log "Begin to add CANN."

  # conda 激活
  if ! activate_conda; then return 1; fi
  # 下载 CANN
  log "Download CANN."
  local cann_file_path
  cann_file_path=$(download_dependencies "cann" "cann")
  # 安装 CANN
  log "Install CANN."
  run_command chmod +x "$cann_file_path"
  run_command "$cann_file_path" --quiet --install --install-for-all --force

  # 添加环境变量
  local source_string="source $CANN_ENV_PATH\n"
  for user in root HwHiAiUser; do
    update_bashrc "$user" "$CANN_ENV_PATH" "$source_string"
  done

  log "Finish adding CANN."
}

# mxVision 安装路径
MXVISION_INSTALL_PATH="/usr/local/Ascend/"

# mxVision 下载安装函数
install_mxvision() {
  set -e
  log "Begin to add mxVision."

  # conda 激活
  if ! activate_conda; then return 1; fi

  # 下载 mxVision
  log "Download mxVision."
  if [ -f "$CANN_ENV_PATH" ]; then source "$CANN_ENV_PATH"; fi

  local mxvision_file_path
  mxvision_file_path=$(download_dependencies "mxvision" "mxvision")

  # 安装 mxVision
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
# 6. 音频与图形化界面
# 音频需要替换内核
# 本地图形化界面需要替换dtb
# 本地和远程图形化界面均采用xfce（现网版本本地图形化界面为lxqt）
# ==============================

# 音频apt依赖数组
APT_AUDIO_DEPENDENCIES=("alsa-tools" "alsa-utils")

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
  apt_install_packages "${APT_AUDIO_DEPENDENCIES[@]}"

  log "Finish building audio function."
}

# 本地图形化界面依赖
# xfce及其相关依赖
APT_LOCAL_DESKTOP_DEPENDENCIES+=("xfce4" "xfce4-goodies" "network-manager-gnome" "lightdm" "firefox" "libdrm-dev")
# 中文输入法引擎
APT_LOCAL_DESKTOP_DEPENDENCIES+=("ibus" "ibus-clutter" "ibus-gtk" "ibus-gtk3" "ibus-pinyin")
# 中文语言包和字体
APT_LOCAL_DESKTOP_DEPENDENCIES+=("language-pack-zh-hans" "fonts-droid-fallback" "ttf-wqy-zenhei" "ttf-wqy-microhei" "fonts-arphic-ukai" "fonts-arphic-uming")

# 本地图形化界面
add_local_desktop() {
  log "Begin to build local desktop function."

  # 配置firefox下载渠道
  if [ ! -f /etc/apt/sources.list.d/mozillateam-ubuntu-ppa-jammy.list ]; then
    log "Modify firefox downloading source."
    run_command sysctl net.ipv6.conf.all.disable_ipv6=1
    run_command add-apt-repository ppa:mozillateam/ppa -y
    # prioritize the apt version of firefox over the snap version
    printf "Package: *
Pin: release o=LP-PPA-mozillateam
Pin-Priority: 1001

Package: firefox
Pin: version 1:1snap1-0ubuntu2
Pin-Priority: -1
" > /etc/apt/preferences.d/mozilla-firefox
    # 配置launchpad反向代理
    printf "deb https://launchpad.proxy.ustclug.org/mozillateam/ppa/ubuntu/ jammy main" > /etc/apt/sources.list.d/mozillateam-ubuntu-ppa-jammy.list
  fi

  # 安装所需依赖
  log "Download local desktop apt dependencies."
  apt_install_packages "${APT_LOCAL_DESKTOP_DEPENDENCIES[@]}"

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
  run_command systemctl enable toggle_graphical_desktop

  # 选择sddm为默认桌面管理器
  run_command dpkg-reconfigure lightdm
  # 配置文件路径
  run_command mkdir -p /etc/lightdm/lightdm.conf.d
  # 添加隐藏用户配置到 sddm.conf 文件
  run_command printf "[Seat:*]
user-session=xfce
hidden-users=HwBaseUser HwDmUser HwSysUser" > /etc/lightdm/lightdm.conf
  # 重启 sddm 服务使配置生效
  run_command systemctl restart lightdm

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
ibus-daemon -drx\n" >> /home/HwHiAiUser/.bashrc
  fi
  log "Finish building local desktop function."
}

# 构建远程桌面依赖数组
# vnc及其相关依赖
APT_REMOTE_DESKTOP_DEPENDENCIES=("network-manager-gnome" "xfce4" "xfce4-goodies" "tightvncserver" "firefox")
# 中文输入法引擎
APT_REMOTE_DESKTOP_DEPENDENCIES+=("ibus" "ibus-clutter" "ibus-gtk" "ibus-gtk3" "ibus-pinyin")
# 中文语言包和字体
APT_REMOTE_DESKTOP_DEPENDENCIES+=("language-pack-zh-hans" "fonts-droid-fallback" "ttf-wqy-zenhei" "ttf-wqy-microhei" "fonts-arphic-ukai" "fonts-arphic-uming")

# 远程图形化界面安装函数
add_remote_desktop() {
  log "Begin to build remote desktop function."
  # 配置firefox下载渠道
  if [ ! -f /etc/apt/sources.list.d/mozillateam-ubuntu-ppa-jammy.list ]; then
    log "Modify firefox downloading source."
    run_command sysctl net.ipv6.conf.all.disable_ipv6=1
    run_command add-apt-repository ppa:mozillateam/ppa -y
    # prioritize the apt version of firefox over the snap version
    printf "Package: *
Pin: release o=LP-PPA-mozillateam
Pin-Priority: 1001

Package: firefox
Pin: version 1:1snap1-0ubuntu2
Pin-Priority: -1
" > /etc/apt/preferences.d/mozilla-firefox
    # 配置launchpad反向代理
    printf "deb https://launchpad.proxy.ustclug.org/mozillateam/ppa/ubuntu/ jammy main" > /etc/apt/sources.list.d/mozillateam-ubuntu-ppa-jammy.list
  fi

  log "Download remote desktop apt dependencies."
  apt_install_packages "${APT_REMOTE_DESKTOP_DEPENDENCIES[@]}"

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

  # 重新安装netplan
  log "Reinstall netplan."
  if ! dpkg-query -W -f='${Status}' netplan.io 2>/dev/null | grep -q "ok installed"; then
    run_command apt-get install netplan.io -y
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

  # 中文输入法自启动，需要在图形化界面上进行操作
  if [ ! -e "$HOME"/.vnc/ibus-autostartup ]; then
    printf "export GTK_IM_MODULE=ibus
export QT_IM_MODULE=ibus
export XMODIFIERS=@im=ibus
ibus-daemon -drx\n" > "$HOME"/.vnc/ibus-autostartup
    chmod 777 "$HOME"/.vnc/ibus-autostartup
  fi

  log "Finish building remote desktop function."
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
# 7. 后处理
# 包括镜像的一些小的功能补齐，还有整个镜像过程当中中间文件的清理
# ==============================

# 后处理函数
post_process() {
  # 修改libstdc++的软链接路径
  log "Beging to change dynamic link for atc."
  local so_target_link
  local so_6_target_link
  so_target_link="$(readlink -f "$MINICONDA_PATH"/lib/libstdc++.so)"
  so_6_target_link="$(readlink -f "$MINICONDA_PATH"/lib/libstdc++.so.6)"
  if [ "$so_target_link" == "$MINICONDA_PATH"/lib/libstdc++.so.6.0.29 ]; then
    run_command mv "$MINICONDA_PATH"/lib/libstdc++.so "$MINICONDA_PATH"/lib/libstdc++.so.old
    run_command ln -s /usr/lib/aarch64-linux-gnu/libstdc++.so.6 "$MINICONDA_PATH"/lib/libstdc++.so
  fi
  if [ "$so_6_target_link" == "$MINICONDA_PATH"/lib/libstdc++.so.6.0.29 ]; then
    run_command mv "$MINICONDA_PATH"/lib/libstdc++.so.6 "$MINICONDA_PATH"/lib/libstdc++.so.6.old
    run_command ln -s /usr/lib/aarch64-linux-gnu/libstdc++.so.6 "$MINICONDA_PATH"/lib/libstdc++.so.6
  fi
  log "Finish changing dynamic link for atc."

  # 解决后处理插件使用报错问题
  log "Copy libpython3.so to /usr/lib64."
  run_command cp "$MINICONDA_PATH"/lib/libpython3.9.so.1.0 /usr/lib64
  if [ -f /root/.cache/gstreamer-1.0/registry.aarch64.bin ]; then
      run_command rm /root/.cache/gstreamer-1.0/registry.aarch64.bin
  fi

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
WantedBy=multi-user.target" /var/oom_killer.sh /var/oom_killer.sh > /etc/systemd/system/oom_killer.service
  local oom_string="echo \$\$ > /sys/fs/cgroup/memory/usermemory/tasks\n"
  for user in root HwHiAiUser; do
    update_bashrc "$user" "usermemory" "$oom_string"
  done
  run_command systemctl daemon-reload
  run_command systemctl enable oom_killer

  # 修改启动显示文本
  log "Change startup text."
  if [ -e /usr/games/lolcat ] && [ ! -e /usr/bin/lolcat ]; then
    run_command mv /usr/games/lolcat /usr/bin
  fi
  run_command rm /etc/update-motd.d/*
  if [ ! -e /etc/update-motd.d/01-ascend-devkit-startup-text ]; then
      printf "#\041/bin/sh
figlet -tk -w 120 \"Ascend-devkit\" | lolcat -f -S 22
printf \"Welcome to Atlas 200I DK A2\\\n\"
[ -r /etc/lsb-release ] && . /etc/lsb-release

if [ -z \"\$DISTRIB_DESCRIPTION\" ] && [ -x /usr/bin/lsb_release ]; then
        # Fall back to using the very slow lsb_release utility
        DISTRIB_DESCRIPTION=\$(lsb_release -s -d)
fi
printf \"This system is based on %%s (%%s %%s %%s)\\\n\" \"\$DISTRIB_DESCRIPTION\" \"\$(uname -o)\" \"\$(uname -r)\" \"\$(uname -m)\"
printf \"\\\n\"
printf \"This system is only applicable to individual developers and cannot be used for commercial purposes.\\\n\"
printf \"\\\n\"
printf \"By using this system, you have agreed to the Huawei Software License Agreement.\\\n\"
printf \"Please refer to the agreement for details on https://www.hiascend.com/software/protocol\\\n\"
printf \"\\\n\"
printf \"Reference resources\\\n\"
printf \"* Home page: https://www.hiascend.com/hardware/developer-kit-a2\\\n\"
printf \"* Documentation: https://www.hiascend.com/hardware/developer-kit-a2/resource\\\n\"
printf \"* Online courses: https://www.hiascend.com/edu/courses\\\n\"
printf \"* Online experiments: https://www.hiascend.com/zh/edu/experiment\\\n\"
printf \"* Forum: https://www.hiascend.com/forum/\\\n\"
printf \"* Code: https://gitee.com/HUAWEI-ASCEND/ascend-devkit\\\n\"
printf \"\\\n\"" > /etc/update-motd.d/01-ascend-devkit-startup-text
  fi
  run_command chmod +x /etc/update-motd.d/01-ascend-devkit-startup-text
  # 再次修改时区
  run_command timedatectl set-timezone Asia/Shanghai
}

# 定制化处理函数
accident_handler() {
  cleanup_temporary_files "$package_file_backup_path"
  cleanup_temporary_files "$path_download"/"$audio_zip_backup_name"
  cleanup_temporary_files "$path_download"/"$graphic_zip_backup_name"
  cleanup_temporary_files "$path_download"/"$wifi_zip_backup_name"
  cleanup_temporary_files "$path_download"
}
