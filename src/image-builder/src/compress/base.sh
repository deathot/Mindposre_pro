#!/usr/bin/env bash

# ==============================
# 1. 参数解析与help信息
# 本部分用于对脚本参数进行解析，进而影响脚本的运行过程
# 同时提供help信息帮助用户了解脚本的基本运行命令
# ==============================

set -e

USAGE="Usage: bash $(basename "$0") [-nc] \"distribution path\" \"source disk\" \"target image\" [\"mount path\"].
-n           add new config.ini to target image
-c           using xz compress the image."

while getopts "hnc" opt; do
  case "$opt" in
    h)
      printf "%s\\n" "$USAGE"
      exit 2
      ;;
    n)
      FLAG_RENEW_NET_CONFIG=true
      ;;
    c)
      FLAG_COMPRESS=true
      ;;
    ?)
      echo "Usage: bash $(basename "$0") [-nc] \"distribution path\" \"source disk\" \"target image\" [\"mount path\"],
 more detailed information please run bash $(basename "$0") -h."
      exit 1
      ;;
  esac
done
shift "$((OPTIND-1))"

# ==============================
# 2. 环境变量
# 在此处对重要的环境变量进行命令和赋值
# ==============================

# 脚本相关信息
SCRIPT_NAME=${BASH_SOURCE[0]}
# 获取当前文件父路径
PARENT_PATH=$(cd "$(dirname "$SCRIPT_NAME")" && pwd)
# 发行版名称代号
distribution_name=$(basename "$(dirname "$(realpath "$PARENT_PATH"/"$1")")")
# 版本号
version=$(basename "$(realpath "$PARENT_PATH"/"$1")")
# 源磁盘
source_disk="$2"
# 目标镜像
target_image="$3"
# 设置mount路径
mount_dir="$4"

# ==============================
# 3. base脚本函数
# 此部分的函数为一些工具类函数
# 包括日志函数，异常结束处理函数，中断处理函数，权限检查，入参检查，依赖检查等
# ==============================

# 日志函数
log() {
  # 打印信息
  echo "[$(date "+%Y-%m-%d %H:%M:%S")] [COMPRESS] $1"
}

# 异常结束处理函数
termination_handler() {
  log "The script will stop for some reasons."
  # 关闭报错即退出
  set +e
  # 处理临时文件
  if df -h | grep -q "$mount_dir"; then
      umount "$mount_dir"
  fi
  rm -r "$mount_dir"
  # 卸除虚拟磁盘挂载
  if [ -e "$path_visual_disk" ]; then
    losetup -d "$path_visual_disk"
  fi
  # 回到用户调用脚本的当前路径
  cd "$(pwd)"
}

# 中断处理函数
interrupt_handler() {
  # 取消ERR标志的trap以防后面的ERR进入循环
  trap - ERR
  printf "\n"
  termination_handler
  log "The script has been terminated by ctrl c."
  exit
}

# 权限检查函数
authority_check_base() {
  set -e
  if [ "$EUID" -ne 0 ]; then
    log "Please use root to run this script."
    return 1
  fi
  export USER=root
  if [ "$(umask)" != "0022" ]; then
    log "Please check umask value of root because it should be 0022."
    return 1
  fi
}

# 参数检查函数
pre_check_parameter() {
  set -e
  [ -z "$source_disk" ] && log "Source disk must be provided." && return 1
  [ -z "$target_image" ] && log "Target image must be provided." && return 1
  mount_dir=${mount_dir:-"$PARENT_PATH"/"$distribution_name"/"$version"/compress}
  mount_dir=${mount_dir%/}
  mkdir -p "$mount_dir"
}

# 依赖和配置文件检查函数
pre_check_dependencies() {
  log "Software dependencies check."
  for dep in parted expect mkfs.fat dump; do
    if ! type -p "$dep" > /dev/null; then
      log "$dep is not existed, please install it first"; return 1
    fi
  done
  log "Config dependencies check."

  # 配置依赖检查
  expand_file_path="$PARENT_PATH/$distribution_name/$version/expand.sh"
  if [ ! -f "$expand_file_path" ]; then
      log "$expand_file_path does not exist"; return 1
  fi
  config_file_path="$PARENT_PATH/config.ini"
  if [ ! -f "$config_file_path" ]; then
      log "$config_file_path does not exist"; return 1
  fi
  E2E_samples_download_tool_path="$PARENT_PATH/E2E_samples_download_tool.sh"
  if [ ! -f "$E2E_samples_download_tool_path" ]; then
      log "$E2E_samples_download_tool_path does not exist"; return 1
  fi
	return 0
}

# ==============================
# 4. 获取源磁盘信息和构建目标磁盘信息
# 按照既定目标规划相关分区容量，获取源磁盘的信息，并根据源磁盘信息构建目标磁盘主分区容量
# ==============================

# 根据已知容量设置分区的容量
set_capacity_for_known_part() {
  # 4K对齐参数
  sector_size_aligned=$((2048*512))
  # SD卡保留容量
  part_disk_reserved_target_size=2048
  # 保留272M系统裸分区
  part_emmc_target_size=$(((1+15+128+128)*1024*2))
  # 交换分区容量
  part_exchange_target_size=$((50*1024*2))
  # 开发者套件保留分区
  part_ascend_reserved_target_size=2048
  # 是否含有交换分区标志
  swap_flag=false
}

# 获取源分区信息
acquire_source_info() {
  log "Acquire source disk information"
  # 判断磁盘是否存在
  if ! fdisk -l | grep -q "$1"; then
    log "Error! the device $1 does not exit!"
    return 1
  fi
  # 获取磁盘相应文件系统类型的分区数量
  mapfile -t parts_ext4_source < <(blkid | grep "$1" | grep ext4 | cut -d ':' -f 1 | sort)
  mapfile -t parts_fat_source < <(blkid | grep "$1" | grep fat | cut -d ':' -f 1)
  parts_ext4_source_amount=${#parts_ext4_source[*]}
  parts_fat_source_amount=${#parts_fat_source[*]}
  # 如果ext4分区数量大于1，则只保留2分区和fat分区
  if [ "$parts_ext4_source_amount" -lt 1 ]; then
    log "Device $1 has just zero ext4 file system partition."
    return 1
  fi
  part_root_source="$1"2
  # 如果没有fat分区后面会创建新的fat分区，fat分区大小固定为50M
  part_exchange_source=${parts_fat_source[0]:-""}
}

# 获取虚拟磁盘初始化容量
acquire_initial_capacity_for_visual_disk() {
  log "Begin to acquire initial capacity for visual disk"
  # 删除原始sd卡的交换分区
  mount "$1" "$mount_dir"
  sync
  if [ -e "$mount_dir"/swapfile ]; then
      swap_flag=true
      rm -r "$mount_dir"/swapfile
  fi
  sleep 3
  umount "$mount_dir"
  # 对目标文件系统进行检查
  e2fsck -y -f "$1" >/dev/null 2>&1
  # 获取可压缩最大大小，单位：块，每个为4K
  block_num=$(resize2fs -P "$1" | cut -d ' ' -f 7) >/dev/null 2>&1
  log "The minimal file system size of $1 is $block_num blocks"
  # 设置root分区目标容量
  part_root_target_size=$(((block_num*4*1024/sector_size_aligned+1)*sector_size_aligned/512))
  # 计算初始虚拟磁盘容量
  initial_capacity=$((part_disk_reserved_target_size+part_emmc_target_size+part_ascend_reserved_target_size+part_root_target_size+part_exchange_target_size+$2*1024*2))
  log "The initial capacity for visual disk is $initial_capacity sectors"
}

# ==============================
# 5. 创建虚拟磁盘
# 使用全零文件创建虚拟磁盘
# ==============================

# 创建全零文件
create_zeroes_image() {
  log "Create all-zero image"
  initial_capacity_bytes=$((initial_capacity*512))
  if [ -f "$1" ]; then
      rm "$1"
  fi
  fallocate -l "$initial_capacity_bytes" "$1" >/dev/null 2>&1
  fallocate -z -l "$initial_capacity_bytes" "$1" >/dev/null 2>&1
}

# 创建虚拟磁盘
create_visual_disk() {
  log "Create visual disk"
  # 创造虚拟磁盘
  path_visual_disk=$(losetup --show -Pvf "$1")
  # 设置目标分区号
  part_ascend_reserved_target="${path_visual_disk}p1"
  part_root_target="${path_visual_disk}p2"
  part_exchange_target="${path_visual_disk}p3"
}

# ==============================
# 6. 填充磁盘
# 计算分区的起始与结束后，创建分区与文件系统，将源磁盘内容填充至目标磁盘，并添加标签
# ==============================

# 计算各分区的起始与结束
calculate_parts_start_and_end() {
  # 计算emmc分区的起始和结束
  part_emmc_target_start="$part_disk_reserved_target_size"
  part_emmc_target_end=$((part_emmc_target_start+part_emmc_target_size-1))
  # 计算占位保留分区的起始和结束
  part_ascend_reserved_target_start=$((part_emmc_target_end+1))
  part_ascend_reserved_target_end=$((part_ascend_reserved_target_start+part_ascend_reserved_target_size-1))
  # 计算root分区的起始和结束
  part_root_target_start=$((part_ascend_reserved_target_end+1))
  part_root_target_end=$((part_root_target_start+part_root_target_size-1))
  # 计算exchange分区的起始和结束
  part_exchange_target_start=$((part_root_target_end+1))
  part_exchange_target_end=$((part_exchange_target_start+part_exchange_target_size-1))
}

# 创建分区
create_part() {
  log "Create partition $2"
  parted -s -a optimal "$1" Unit s mkpart primary "$3" "$4" "$5" >/dev/null 2>&1
  partprobe
}

# 创建文件系统
make_fs() {
  log "Make file system $1"
  if [[ "$2" == "fat32" ]]; then
      mkfs.fat "$1" >/dev/null 2>&1
  else
      mkfs.ext4 -F "$1" >/dev/null 2>&1
  fi
  partprobe
}

# 复制分区
move_source_to_target() {
  [ -z "$2" ] && log "Source is empty, finish moving" && return 0
  log "move content from $2 to $3"
  type="$(blkid | grep "$3" | awk '{ delete vars; for(i = 1; i <= NF; ++i) { n = index($i, "="); if(n) { vars[substr($i, 1, n - 1)] = substr($i, n + 1) } } Var = vars["Var"] } { print vars["TYPE"] }' | cut -d'"' -f2)"
  if [[ "$type" == *"fat"* ]]; then
    expect <<- END > /dev/null 2>&1
     spawn fsck "$2"
     expect "[*2*q*"
     send "2\n"
     expect eof
     exit
END
    dd if="$2" of="$3" bs=512 count="$4"
  else
    fsck "$2" > /dev/null 2>&1
    mount "$3" "$mount_dir"
    sync
    cd "$mount_dir" || exit >/dev/null 2>&1
    dump -0ay -f - "$2" | restore -rf - >/dev/null 2>&1
    cd - || exit >/dev/null 2>&1
    sleep 3
    umount "$mount_dir"
  fi
  return 0
}

# 添加标签
add_label() {
  log "Add label to partition $1"
  type="$(blkid | grep "$1" | awk '{ delete vars; for(i = 1; i <= NF; ++i) { n = index($i, "="); if(n) { vars[substr($i, 1, n - 1)] = substr($i, n + 1) } } Var = vars["Var"] } { print vars["TYPE"] }' | cut -d'"' -f2)"
  if [[ "$type" == *"fat"* ]]; then
    fatlabel "$1" "$2" > /dev/null 2>&1
  else
    e2label "$1" "$2" > /dev/null 2>&1
  fi
}

# ==============================
# 7. 配置修改
# 修改fstab以正确挂载，补回源磁盘的swap文件，向目标磁盘添加扩容脚本，根据config.ini修改目标磁盘网络配置，最后清理中间文件和文件夹
# ==============================

# 创建并修改fstab
create_and_write_fstab() {
  log "Create fstab for $1"
  mount "$part_root_target" "$mount_dir"
  sync
  # 挂载exchange分区
  mkdir -p "$mount_dir$2"
  type=${3:-$(blkid | grep "$1" | awk '{ delete vars; for(i = 1; i <= NF; ++i) { n = index($i, "="); if(n) { vars[substr($i, 1, n - 1)] = substr($i, n + 1) } } Var = vars["Var"] } { print vars["TYPE"] }' | cut -d'"' -f2)}
  UUID="$(blkid | grep "$1" | awk '{ delete vars; for(i = 1; i <= NF; ++i) { n = index($i, "="); if(n) { vars[substr($i, 1, n - 1)] = substr($i, n + 1) } } Var = vars["Var"] } { print vars["UUID"] }' | cut -d'"' -f2)"
  fstab_configuration+="UUID=$UUID   $2  $type   defaults   0   0\n"
  fstab_configuration+="tmpfs   /var/log   tmpfs   rw,mode=0755,size=128M   0   0\n"
  log "Write fstab to $mount_dir/etc/fstab"
  echo -e "$fstab_configuration" > "$mount_dir"/etc/fstab
  sleep 3
  umount "$mount_dir"
}

# 后处理函数
post_process() {
  if [ "$swap_flag" == "true" ]; then
    write_back_swap_file "$part_root_source" || return 1
  fi
  write_expand_script "$part_root_target" || return 1
  write_net_config "$part_exchange_target" || return 1
  clean_temp_file_and_path || return 1
}

# 补回交换空间文件
write_back_swap_file() {
  log "Write back swap file to $1"
  mount "$1" "$mount_dir"
  sync
  # 写回交换分区
  fallocate -z -l 8G "$mount_dir"/swapfile
  chmod 600 "$mount_dir"/swapfile
  grep -q "swap" "$mount_dir"/etc/fstab || echo -e "/swapfile    swap    swap    defaults    0    0" >> "$mount_dir"/etc/fstab
  sleep 3
  umount "$mount_dir"
}

# 写入扩容脚本
write_expand_script() {
  log "Write expand script to $1"
  mount "$1" "$mount_dir"
  sync
  # 删除扩容标志文件
  rm -f "$mount_dir"/var/mini_upgraded
  rm -f "$mount_dir"/etc/NetworkManager/system-connections/*
  # 将扩容脚本拷贝至镜像
  cp "$expand_file_path" "$mount_dir"/var/
  # 不同版本的minirc_boot.sh代码不同，此处grep两次是为了兼容性
  grep -q "expand.sh" "$mount_dir"/var/minirc_boot.sh || sed -i "/logBootTimeStamp \"minirc_boot--->permit_create_new_user start : \"/i logBootTimeStamp \"minirc_boot--->expand start :\"\nchmod +x /var/expand.sh\n/bin/bash /var/expand.sh >> \$bootLogFile" "$mount_dir"/var/minirc_boot.sh
  grep -q "expand.sh" "$mount_dir"/var/minirc_boot.sh || sed -i '/log "\[INFO\]mini boot start"/a\device_sys_init_log \"expand start\"\nchmod +x /var/expand.sh\n/bin/bash /var/expand.sh > /var/log/ascend_seclog/ascend_devkit_expand.log' "$mount_dir"/var/minirc_boot.sh
  # 将E2E下载工具脚本拷贝至镜像
  [ ! -f "$mount_dir"/usr/local/E2E_samples_download_tool.sh ] && cp "$E2E_samples_download_tool_path" "$mount_dir"/usr/local && chmod 777 "$mount_dir"/usr/local/E2E_samples_download_tool.sh
  sleep 3
  umount "$mount_dir"
}

# 修改网络配置文件
write_net_config() {
  log "Write net config file to $1"
  mount "$1" "$mount_dir"
  sync
  rm -rf "$mount_dir"/'System Volume Information'
  [ ! -f "$mount_dir"/config.ini ] || [ "$FLAG_RENEW_NET_CONFIG" == "true" ] && log "Add or overwrite net config file" && cp "$config_file_path" "$mount_dir"
  sleep 3
  umount "$mount_dir"
}

# 清理临时文件和目录
clean_temp_file_and_path() {
  log "Clean temporary files and paths"
  # 处理临时文件
  if [ -e "$mount_dir" ]; then
    umount "$mount_dir" 2>/dev/null
    rm -r "$mount_dir"
  fi
  # 卸除虚拟磁盘挂载
  if [ -e "$path_visual_disk" ]; then
    losetup -d "$path_visual_disk"
  fi
  log "Cleanup finished."
}

# ==============================
# 8. 主函数
# 控制整个脚本流程
# ==============================

main() {
  set -e
  # 将异常结束函数赋予ERR标志
  trap termination_handler ERR
  # 将中断处理函数赋予SIGINT标志
  trap interrupt_handler SIGINT

  # 检查权限和依赖
  authority_check_base || return 1
  pre_check_parameter || return 1
  pre_check_dependencies || return 1
  # 设置已知容量的分区的容量
  set_capacity_for_known_part || return 1
  # 获取源分区信息
  acquire_source_info "$source_disk" || return 1
  # 获取目标压缩容量
  acquire_initial_capacity_for_visual_disk "$part_root_source" 50 || return 1
  # 创造全零镜像文件
  create_zeroes_image "$target_image" || return 1
  # 创造虚拟磁盘并且将分区路径赋予目标分区
  create_visual_disk "$target_image" || return 1
  # 计算各目标分区的起始与结束位置
  calculate_parts_start_and_end || return 1

  # 对虚拟磁盘建立gpt分区表
  parted -s "$path_visual_disk" mklabel gpt || return 1
  # 创建分区
  create_part "$path_visual_disk" "$part_ascend_reserved_target" "ext4" "$part_ascend_reserved_target_start" "$part_ascend_reserved_target_end" || return 1
  create_part "$path_visual_disk" "$part_root_target" "ext4" "$part_root_target_start" "$part_root_target_end" || return 1
  create_part "$path_visual_disk" "$part_exchange_target" "fat32" "$part_exchange_target_start" "$part_exchange_target_end" || return 1
  # 设置文件系统
  make_fs "$part_ascend_reserved_target" "ext4" || return 1
  make_fs "$part_root_target" "ext4" || return 1
  make_fs "$part_exchange_target" "fat32" || return 1
  # 填充分区
  log "move emmc content from source to target"
  dd if="$source_disk" of="$path_visual_disk" skip="$part_disk_reserved_target_size" seek="$part_disk_reserved_target_size" bs=512 count="$part_emmc_target_size" >/dev/null 2>&1 || return 1
  move_source_to_target "$path_visual_disk" "$part_root_source" "$part_root_target" || return 1
  move_source_to_target "$path_visual_disk" "$part_exchange_source" "$part_exchange_target" "$part_exchange_target_size" || return 1
  # 创建标签
  add_label "$part_ascend_reserved_target" "reserved_fs" || return 1
  add_label "$part_root_target" "root_fs" || return 1
  add_label "$part_exchange_target" "exchange_fs" || return 1
  # 创建分区表并写入相应位置
  create_and_write_fstab "$part_exchange_target" "/exchange" || return 1

  # 后处理
  post_process || return 1
  if [ "$FLAG_COMPRESS" == "true" ]; then
    # 压缩镜像文件
    xz -ck -T0 -9 -vv "$3" > "$3".xz || return 1
  fi
}

if ! main "$1" "$2" "$3"; then
  log "sd card compress failed!"
else
  log "sd card compress success!"
fi
