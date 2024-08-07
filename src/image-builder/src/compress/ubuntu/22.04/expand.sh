#!/usr/bin/env bash

# 日志函数
log() {
  cur_date=$(date "+%Y-%m-%d %H:%M:%S")
  echo "[$cur_date] [SD CARD EXPAND] ""$1"
}

# 预检查依赖和配置文件
pre_check_dependencies() {
	log "Check some soft does exist."
	if [[ ! -f $(type -p parted) ]]; then
		log "parted is not existed, please install it first"; return 1
	fi
	if [[ ! -f $(type -p expect) ]]; then
		log "expect is not existed, please install it first"; return 1
	fi
	if [[ ! -f $(type -p mkfs.fat) ]]; then
		log "dosfstools is not existed, please install it first"; return 1
	fi
	return 0
}

# 计算分区容量
calculate_parts_capacities() {
  disk_capacity=$(fdisk -l | grep "${dev_name}:" | awk -F ' ' '{print $7}')
	if [[ $disk_capacity == "" ]]; then
		log "The device $dev_name does not exit"; return 1
	fi
	disk_capacity_aligned=$(((disk_capacity)*512/sector_size_aligned*sector_size_aligned/512))
	# 多减1是为了4K对齐
	part_exchange_end=$((disk_capacity_aligned-2048-1))
  part_exchange_start=$((part_exchange_end-part_exchange_size+1))
  part_root_end=$((part_exchange_start-1))
  return 0
}

# 删除分区
del_part() {
  log "Delete part $2"
  if [[ $(df -h | grep "$2") != '' ]]; then umount "$2"; fi
	part_to_delete_num=${2: -1}
  if ! parted -s "$1" rm "$part_to_delete_num" > /dev/null 2>&1; then return 1; fi
  partprobe
	log "Part $2 delete finish."
	return 0
}

# 创建分区
create_part() {
  log "Create partition $2"
  if [[ "$(parted --script "$dev_name" p 2>&1)" == *"fix the GPT"* ]]; then
    expect <<- END > /dev/null 2>&1
      spawn parted
      expect "*(parted)*"
      send "mkpart\n"
      expect "*Fix/Ignore*"
      send "Fix\n"
      expect "*Partition name?*"
      exit
END
  partprobe
  fi
  if [[ "$3" == *"fat"* ]]; then
    if ! parted -s -a optimal "$1" Unit s mkpart primary fat32 "$4" "$5" >/dev/null 2>&1; then return 1; fi
  else
    if ! parted -s -a optimal "$1" Unit s mkpart primary ext4 "$4" "$5" >/dev/null 2>&1; then return 1; fi
  fi
  partprobe
  return 0
}

# 创建文件系统
make_fs() {
  log "Make file system $1"
  if [[ "$2" == *"fat"* ]]; then
      if ! mkfs.fat "$1" >/dev/null 2>&1; then return 1; fi
  else
      if ! mkfs.ext4 -F "$1" >/dev/null 2>&1; then return 1; fi
  fi
  partprobe
  return 0
}

# 添加标签
add_label() {
  log "Add label to partition $1"
  type="$(blkid | grep "$1" | awk '{ delete vars; for(i = 1; i <= NF; ++i) { n = index($i, "="); if(n) { vars[substr($i, 1, n - 1)] = substr($i, n + 1) } } Var = vars["Var"] } { print vars["TYPE"] }' | cut -d'"' -f2)"
  if [[ "$type" == *"fat"* ]]; then
    if ! fatlabel "$1" "$2" > /dev/null 2>&1; then return 1; fi
  else
    if ! e2label "$1" "$2" > /dev/null 2>&1; then return 1; fi
  fi
  return 0
}

# 重新分配分区容量
resize_part() {
  log "Resize part $1"
  part_to_resize_num=${1: -1}
	expect <<- END > /dev/null 2>&1
	spawn parted $dev_name
	expect "parted"
	send "resizepart $part_to_resize_num\n"
	expect "End"
	send "$2s\n"
	expect "parted"
	send "q\n"
	expect eof
	exit
END
  partprobe
  if ! resize2fs "$1" > /dev/null 2>&1; then return 1; fi
  return 0
}

# 创建交换分区
create_swap_part() {
  # 创建并挂载swap分区
  fallocate -l 8G /swapfile
  fallocate -z -l 8G /swapfile
  chmod 600 /swapfile
  mkswap /swapfile
  swapon /swapfile
  fstab_configuration+="/swapfile   swap   swap   defaults   0   0\n"
  fstab_configuration+="tmpfs   /var/log   tmpfs   rw,mode=0755,size=128M   0   0\n"
}

# 挂载分区
mount_part() {
  log "Mount part $1 to $2"
  mkdir -p "$2"
  UUID="$(blkid | grep "$1" | awk '{ delete vars; for(i = 1; i <= NF; ++i) { n = index($i, "="); if(n) { vars[substr($i, 1, n - 1)] = substr($i, n + 1) } } Var = vars["Var"] } { print vars["UUID"] }' | cut -d'"' -f2)"
  type="$(blkid | grep "$1" | awk '{ delete vars; for(i = 1; i <= NF; ++i) { n = index($i, "="); if(n) { vars[substr($i, 1, n - 1)] = substr($i, n + 1) } } Var = vars["Var"] } { print vars["TYPE"] }' | cut -d'"' -f2)"
  fstab_configuration+="UUID=$UUID   $2  $type   defaults   0   0"
  echo -e "$fstab_configuration" > /etc/fstab
  if ! mount -a; then return 1; fi
}

# 扩容与挂载
expand_and_mount() {
  # 4K对齐参数
  sector_size_aligned=$((2048*512))
  # 设备与分区信息
  dev_name="/dev/mmcblk1"
  part_root="$dev_name""p2"
  part_exchange="$dev_name""p3"
  part_exchange_size=$((50*1024*2))
  # 计算分区容量
  if ! calculate_parts_capacities; then return 1; fi
  # 备份exchange分区
  mkdir -p "$conf_dir"
  cp "$conf" "$conf_dir"
  # 删除exchange分区
  if ! del_part "$dev_name" "$part_exchange"; then return 1; fi
  # 创建新exchange分区
  if ! create_part "$dev_name" "$part_exchange" "fat" "$part_exchange_start" "$part_exchange_end"; then return 1; fi
  # 创建新exchange分区文件系统
  if ! make_fs "$part_exchange" "fat"; then return 1; fi
  # 向新exchange分区添加新标签
  if ! add_label "$part_exchange" "exchange"; then return 1; fi
  # 扩展root分区
  if ! resize_part "$part_root" "$part_root_end"; then return 1; fi
  # 创建扩容分区
  if ! create_swap_part > /dev/null 2>&1; then return 1; fi
  # 挂载exchange分区
  mount_part "$part_exchange" "/exchange"
  # 将备份文件放入新的exchange分区
  cp "$conf_dir"/"$conf_file_name" "/exchange"
  return 0
}

# 设置单个端口的ip，传入参数为：要设置的端口(eth0/eth1/usb0)
set_ip() {
	eval dhcp4=\$"$1"_dhcp4
	eval address=\$"$1"_address
	eval mask=\$"$1"_mask
	eval route=\$"$1"_route
	eval dns_pre=\$"$1"_dns_pre
	eval dns_alter=\$"$1"_dns_alter
	echo "    $1:" >> "$file"
	if [[ $dhcp4 == "yes" ]]; then
		echo "      dhcp4: yes" >> "$file"
		echo "set $1 dhcp4 to yes."
	else
		echo "      dhcp4: no" >> "$file"
		if [[ $address == "" ]]; then
			eval address=\$"$1"_def_ip
		fi
		if [[ $mask == "" ]]; then
			mask=$def_mask
		fi
		echo "      addresses: [$address/$mask]" >> "$file"
		echo "set $1 ip=$address, mask=$mask"
	fi
	echo "set $1 ip finish."
	if [[ $route != "" && $route_flag == "false" ]]; then
		{ echo "      routes:"; echo "        - to: default"; echo "          via: $route"; } >> "$file"
		route_flag=true
		echo "set $1 route=$route."
	fi
	# shellcheck disable=SC2086
	if [[ $dns_pre != "" || $dns_alter != "" ]]; then
		echo "      nameservers:" >> "$file"
		if [[ $dns_pre != "" ]]; then
			echo "        addresses: [$dns_pre]" >> "$file"
			echo "set $1 dns=$dns_pre."
		fi
		if [[ $dns_alter != "" ]]; then
			echo "        addresses: [$dns_alter]" >> "$file"
			echo "set $1 dns=$dns_alter."
		fi
	fi
	echo "" >> "$file"
}

# 设置两个网口和一个typeC口的ip
set_ips() {
	# 路由是否已设置的标志
	route_flag="false"
	path="/etc/netplan"
	file_name="01-netcfg.yaml"
	file=$path/$file_name
	rm $path/*.yaml
	touch $path/$file_name
	echo "network:
  version: 2
  renderer: networkd
  ethernets:" >> $file
	set_ip eth0
	set_ip eth1
	set_ip usb0
}

main() {
  # 脚本相关信息
  file_name=${BASH_SOURCE[0]}
  path=$(cd "$(dirname "$file_name")" && pwd)

  # 网络配置
  # 默认ip和掩码
  eth0_def_ip=192.168.1.100
  eth1_def_ip=192.168.137.100
  usb0_def_ip=192.168.0.2
  def_mask=24

  # 配置文件
  conf_file_name=config.ini
  # 配置文件备份文件夹
  conf_dir=/root/configFile
  # 配置文件目标位置
  conf=/exchange/"$conf_file_name"

  # 扩容
  # 扩展磁盘标志，若为true则执行扩展磁盘操作，执行一次后会改成false
  expand_flag=true
  if [[ $expand_flag == "true" ]]; then
    if ! expand_and_mount; then
      log "sd card expand failed!"
      echo "failed" >> /var/sd_expanded_result_flag
    else
      log "sd card expand success!"
      echo "successful" >> /var/sd_expanded_result_flag
    fi
    sed -i "s/^  expand_flag=true$/  expand_flag=false/" /var/expand.sh
  fi
  swap_flag=$(free -h | grep Swap | cut -d' ' -f14 | cut -d'B' -f1)
  if [ "$swap_flag" == "0" ]; then
    mkswap /swapfile
    swapon /swapfile
  fi

  # 设置ip
  if [ -e $conf ]; then
    chmod +x $conf
    dos2unix $conf
    . $conf
    if [[ $setting_flag == "true" ]] && [[ -e /var/mini_upgraded ]]; then
      log "Begin to set ips"
      set_ips
      sed -i "s/^setting_flag=true$/setting_flag=false/" "$conf"
      log "Finished setting ips"
    fi
  fi
  netplan apply
}

main
