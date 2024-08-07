#!/usr/bin/env bash

# ==============================
# 1. 获取构建最小镜像所需依赖
# 本部分用于对制作镜像所需依赖的软件包，工具脚本下载
# ==============================

# 获取基础镜像
get_base_image() {
  set -e
  # 下载所需发行版的镜像文件
  log "Download $distribution_name image."
  local pattern="$distribution_name"".*""$version.*LTS.*aarch64.*\.iso"
  local index="base_image"
  download_dependencies "$pattern" "$index"
}

# 获取定制的文件系统
get_file_system() {
  set -e
  # 直接下载官方提供的定制文件系统
  log "Download customized file system."
  local pattern=".*Sample.*""$distribution_name.*"
  local matching_files=()
  for file in "$path_download"/*; do
    if [[ $(basename "$file") =~ $pattern ]]; then
      matching_files+=("$file")
    fi
  done

  if [ ${#matching_files[@]} -gt 1 ]; then
    log "Multiple files match the pattern $pattern. Please provide a unique file."
    exit 1
  elif [ ${#matching_files[@]} -eq 1 ]; then
    log "File ${matching_files[0]} already exists."
    return 0
  else
    local pattern="Ascend*sample-root-filesystem*.zip"
    local index="file_system"
    local customized_file_system_zip_path
    customized_file_system_zip_path=$(download_dependencies "$pattern" "$index")

    # 解压缩
    local customized_file_system_name
    customized_file_system_name="$(unzip -l "$customized_file_system_zip_path" | grep "Sample.*$distribution_name" | sed 's/.*[[:space:]]\{1,\}\([^[:space:]]\{1,\}\)$/\1/')"
    run_command unzip -j "$customized_file_system_zip_path" "$customized_file_system_name" -d "$path_download"
  fi
}

# 获取npu驱动包
get_npu_driver() {
  set -e
  # 下载驱动文件
  log "Download npu driver."
  local pattern="Ascend.*npu.*driver.*.run"
  local index="npu"
  download_dependencies "$pattern" "$index"
}

# 获取制卡工具
get_hdk() {
  set -e
  # 下载镜像工具文件
  log "Download ascend hdk."
  local path_sdtool="$path_download"/sdtool
  if ! ls "$path_sdtool" > /dev/null 2>&1; then
    local path_sdtool_tar="$path_sdtool".tar.gz
    if ! ls "$path_sdtool_tar" > /dev/null 2>&1; then
      local pattern="Ascend.*hdk.*sdk.*soc.*.zip"
      local index="hdk"
      local ascend_hdk_zip_path
      ascend_hdk_zip_path=$(download_dependencies "$pattern" "$index")

      # 解压缩sdtool压缩包
      local path_sdtool_tar_name
      path_sdtool_tar_name=$(basename "$path_sdtool_tar")
      run_command unzip -j "$ascend_hdk_zip_path" Ascend310B-sdk/"$path_sdtool_tar_name" -d "$path_download"

      # 解压缩sdtool
      run_command tar -xzvf "$path_sdtool_tar" -C "$path_download"
    fi
  fi
  cp -arf  "${path_sdtool:?}"/* "$path_download"/
}

# ==============================
# 2. 写卡
# 使用hdk sdk中的sdtool工具包制作镜像并写入卡中
# ==============================

# 设备写入函数
write_to_device() {
  set -e
  # 开始向目标设备写入内容
  log "Begin to write to device $dev_name."
  if ! [ -x "$(command -v "$path_download"/emmc-head)" ]; then
    log 'Error: emmc-head is not installed.' >&2
    exit 1
  fi
  # 修改镜像工具配置文件
  printf "#\041/bin/bash
MAKE_IMGPK_FLAG=off
FS_BACKUP_FLAG=off
ROOT_PART_SIZE=20480
LOG_PART_SIZE=1024
HOME_DATA_SIZE=2048" > "$path_download"/mksd.conf
  run_command cd "$path_download"
  expect <<-END > /dev/null 2>&1
      # 设置超时时间为10分钟
      set timeout 360
      # 开始制卡
      spawn python3 $path_download/make_sd_card.py local $dev_name
      expect "Please input Y: continue*"
      # 确认已经下载好所有依赖
      send "Y\n"
      # 等待完成的返回内容
      expect "Make SD Card successfully!"
      exit
END
  run_command cd - > /dev/null 2>&1
  log "Finish writing to $dev_name."
}

# ==============================
# 3. 后处理
# 写卡后有些配置需要修改如root登陆权限，网络配置等
# 同时还要清理中间临时文件
# ==============================

post_process() {
  set -e
  log "Conduct post processing."
  mount "$dev_name"2 "$path_mount"
  sync
  new_root_password="Mind@123"
  chroot "$path_mount" /bin/bash -c "
# Change the user's password
echo 'root:${new_root_password}' | chpasswd
"
  run_command sed -i "s/http:\/\/repo.openeuler.org\/openEuler-22.03-LTS\//https:\/\/repo.huaweicloud.com\/openeuler\/openEuler-22.03-LTS\//" "$path_mount"/etc/yum.repos.d/openEuler.repo
  printf "TYPE=Ethernet
BOOTPROTO=dhcp
DEFROUTE=no
NAME=eth0
DEVICE=eth0
ONBOOT=yes" > "$path_mount"/etc/sysconfig/network-scripts/ifcfg-eth0
  printf "TYPE=Ethernet
BOOTPROTO=static
DEFROUTE=yes
NAME=eth1
DEVICE=eth1
ONBOOT=yes
IPADDR=192.168.137.100
PREFIX=24
GATEWAY=192.168.137.1
DNS1=8.8.8.8
DNS2=114.114.114.114" > "$path_mount"/etc/sysconfig/network-scripts/ifcfg-eth1
  printf "TYPE=Ethernet
BOOTPROTO=static
DEFROUTE=yes
NAME=usb0
DEVICE=usb0
ONBOOT=yes
IPADDR=192.168.0.2
PREFIX=24" > "$path_mount"/etc/sysconfig/network-scripts/ifcfg-usb0

  sleep 3
  umount "$path_mount"
  cleanup_temporary_files "$path_mount"
  cleanup_temporary_files "$PARENT_PATH"/minirc_install_hook.sh
  cleanup_temporary_files "$PARENT_PATH"/boot_image_info
  cleanup_temporary_files "$PARENT_PATH"/parttion_head_info
}

# 定制化处理函数
accident_handler() {
  cleanup_temporary_files "$package_file_backup_path"
  cleanup_temporary_files "$path_download"
  cleanup_temporary_files "$path_mount"
  cleanup_temporary_files "$PARENT_PATH"/minirc_install_hook.sh
  cleanup_temporary_files "$PARENT_PATH"/boot_image_info
  cleanup_temporary_files "$PARENT_PATH"/parttion_head_info
}
