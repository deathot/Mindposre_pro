#!/usr/bin/env bash

# ==============================
# 1. 参数解析与help信息
# 本部分用于对脚本参数进行解析，进而影响脚本的运行过程
# 同时提供help信息帮助用户了解脚本的基本运行命令
# ==============================

set -e

USAGE="Usage: bash $(basename "$0") [-v] \"distribution path\" \"target disk\" [\"download path of dependencies\"] [\"mount path for post process\"].
Script will find the distribution path below parent path, which provides cfg.json and func.sh to finish the whole complete image build process."

while getopts "hv" opt; do
  case "$opt" in
  h)
    printf "%s\\n" "$USAGE"
    exit 2
    ;;
  v)
    FLAG_VERBOSE=true
    ;;
  ?)
    echo "Usage: bash $(basename "$0") [-v] \"distribution path\" \"target disk\" [\"download path of dependencies\"] [\"mount path for post process\"],
 more detailed information please run bash $(basename "$0") -h."
    exit 1
    ;;
  esac
done
shift "$((OPTIND - 1))"

# ==============================
# 2. 环境变量
# 在此处对重要的环境变量进行命令和赋值
# ==============================

# 脚本名称
SCRIPT_NAME=${BASH_SOURCE[0]}
# 获取当前文件父路径
PARENT_PATH=$(cd "$(dirname "$SCRIPT_NAME")" && pwd)
# 配置文件名称
CONFIG_FILE_NAME="cfg.json"
# 函数仓库名称
FUNCTION_FILE_NAME="func.sh"
# 发行版名称代号
distribution_name=$(basename "$(dirname "$(realpath "$PARENT_PATH"/"$1")")")
# 版本号
version=$(basename "$(realpath "$PARENT_PATH"/"$1")")
# 用户目标磁盘
dev_name="$2"
# 用户指定的依赖下载路径
path_download="$3"
# 用户指定后处理挂载路径
path_mount="$4"

# ==============================
# 3. 通用功能函数
# 在此部分将一些通用的功能，或者代码容易重复的部分抽离成函数，减小代码量，同时便于使用
# 包括日志函数，命令辅助函数，依赖下载函数，异常结束处理函数，中断处理函数等
# ==============================

# 日志函数
log() {
  # 打印信息
  echo "[$(date "+%Y-%m-%d %H:%M:%S")] [MINIMAL] $1"
}

# 命令辅助函数
# 注意run_command不能执行带有">>"以及类似符号的命令
# 如echo 123 > test.sh
# 因为">>"会被当作命令而不是重定向符号
run_command() {
  if [ "$FLAG_VERBOSE" == true ]; then
    "$@" || { termination_handler; echo "Command failed: $*"; exit 1; }
  else
    "$@" > /dev/null 2>&1 || { termination_handler; echo "Command failed: $*"; exit 1; }
  fi
}

# 依赖下载函数(common)
# 最小镜像的制作不允许有多个相同软件的安装脚本，由sdtool脚本特性决定
# 打印日志后必须return，否则会将打印的日志信息一起输出回函数结果
download_dependencies() {
  local pattern="$1"
  local index="$2"

  # 检查符合正则表达式的文件是否存在
  local matching_files=()
  for file in "$path_download"/*; do
    if [[ $(basename "$file") =~ $pattern ]] && [[ $(basename "$file") != *"backup"* ]]; then
      matching_files+=("$file")
    fi
  done

  # 文件唯一性检查
  if [ ${#matching_files[@]} -gt 1 ]; then
    log "Multiple files match the pattern $pattern. Please provide unique dependency file."
    exit 1
  elif [ ${#matching_files[@]} -eq 1 ]; then
    echo "${matching_files[0]}"
    return 0
  fi

  # 如果文件不存在，查询配置文件
  local package_url
  package_url="$(jq -r ".source.url[\"$index\"]" "$config_path")"

  # url不存在，报错
  if [ -z "$package_url" ] || [ "$package_url" == "null" ]; then
    log "URL for $index is not provided in the config file."
    exit 1
  fi

  # package_file_backup_name不能local，因为cleanup_temporary_files需要此变量
  local package_file_name
  local package_file_path
  local base
  local ext

  package_file_name=$(basename "$package_url")
  package_file_path="$path_download/$package_file_name"
  base="${package_file_name%.*}"
  ext="${package_file_name##*.}"
  package_file_backup_path="$path_download/$base.backup.$ext"

  # 下载文件
  run_command wget "$package_url" -O "$package_file_backup_path" --no-check-certificate
  run_command mv "$package_file_backup_path" "$package_file_path"

  # 检查文件是否下载成功
  if [ ! -f "$package_file_path" ]; then
    log "Failed to download $package_file_name."
    exit 1
  fi

  echo "$package_file_path"
}

# 异常结束处理函数
termination_handler() {
  if [ "$(type -t accident_handler)" = "function" ]; then
    accident_handler
  fi
  # 回到用户调用脚本的当前路径
  cd "$(pwd)"
  # 关闭报错即退出
  set +e
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

# 临时文件清理函数
# 输入参数: 待删除的路径或者文件 空目录标识
# 先判断是否在path_download下面, 如果不是则直接按入参进行删除
# 当空目录标识为true时, 函数将不会判断目录是否为空, 直接删除
cleanup_temporary_files() {
  set -e
  if [ "$1" == "" ]; then
    return 0
  fi
  if [ ! -e "$1" ]; then
    return 0
  fi
  if [ -f "$1" ] || [ -z "$(ls -A "$1")" ] || [ "$2" == true ]; then
    rm -r "$1"
    return 0
  fi
}

# ==============================
# 4. base脚本函数
# 第3部分的处理函数为通用的func.sh服务，而此部分的函数均为base.sh脚本服务
# 包括权限检查，入参检查，依赖下载等
# ==============================

# 权限检查函数
authority_check_base() {
  set -e
  if [ "$EUID" -ne 0 ]; then
    log "Please use root to run this script."
    exit 1
  fi
  export USER=root
  if [ "$(umask)" != "0022" ]; then
    log "Please check umask value of root because it should be 0022."
    exit 1
  fi
}

# 入参检查函数
parameter_check_base() {
  set -e
  [ -z "$distribution_name" ] && log "Distribution name must be provided." && exit 1
  [ -z "$version" ] &&  log "Version must be provided." && exit 1
  [ -z "$dev_name" ] && log "Device name must be provided." && exit 1
  if ! lsblk "$dev_name" >/dev/null 2>&1; then
    log "Device $dev_name does not exist." && exit 1
  fi
  path_download=${path_download:-"$PARENT_PATH/$distribution_name/$version/download"}
  path_download=${path_download%/}
  run_command mkdir -p "$path_download"
  path_mount=${path_mount:-"$PARENT_PATH/$distribution_name/$version/mnt"}
  path_mount=${path_mount%/}
  run_command mkdir -p "$path_mount"
}

# 依赖安装函数(base)
install_dependencies_base() {
  set -e
  log "Install dependencies for base."
  # 处理配置文件的依赖
  local apt_required
  local apt_installed
  local apt_to_install

  apt_required=("jq")
  # sdtool依赖
  apt_required+=("qemu-user-static" "binfmt-support" "dosfstools" "dump" "parted" "kpartx" "gcc" "g++")
  # 已下载依赖数组
  apt_installed=()
  # 未下载依赖数组
  apt_to_install=()
  for package in "${apt_required[@]}"; do
    if dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q "ok installed"; then
      apt_installed+=("$package")
    else
      apt_to_install+=("$package")
    fi
  done
  if [ ${#apt_installed[@]} -ne 0 ]; then log "These packages have been installed: ${apt_installed[*]}."; fi
  if [ ${#apt_to_install[@]} -ne 0 ]; then
    log "These packages are to install: ${apt_to_install[*]}."
    run_command apt-get update > /dev/null 2>&1
    run_command apt-get -f -y --no-install-recommends install "${apt_to_install[@]}" > /dev/null 2>&1
  else
    log "No packages to install."
  fi
}

# ==============================
# 5. 运行控制
# 此部分为base.sh最重要的部分，用来读取和修改配置文件来决定func.sh的运行过程
# 首先构建要运行的函数的数组，随后根据数组运行相应函数，并修改配置文件中对应的配置
# ==============================

# 配置文件处理函数
# 输入参数: 系统名称 系统版本 依赖下载路径 依赖安装路径
# 下载路径指的是wifi的ko, miniconda, CANN, mxVision等安装脚本目录, 默认为当前脚本路径下的download
# 安装路径指的是Miniconda3, CANN, mxVision等依赖在安装的时候的目录, 默认为/usr/local
process_config_base() {
  set -e
  # 构造变量
  # 函数使能标志
  enabled_flag="y"
  disabled_flag="n"
  # 配置文件和函数定义文件路径
  config_path="$PARENT_PATH/$distribution_name/$version/$CONFIG_FILE_NAME"
  func_path="$PARENT_PATH/$distribution_name/$version/$FUNCTION_FILE_NAME"

  # 获取配置yaml文件所有的key, 并构建函数数组
  log "Build required-functions array before running."
  IFS=" " read -r -a functions_array <<< "$(jq -r --arg enabled_flag "$enabled_flag" '.function | to_entries | map(select(.value == $enabled_flag)) | map(.key) | @sh' "$config_path" | tr -d "'")"
}

# 运行函数
# 根据构建的函数数组运行相关函数
run_base() {
  set -e
  # 引入函数定义文件中的函数
  # shellcheck disable=SC1090
  source "$func_path"
  # 运行成功的函数的计数
  local func_executed_num=0
  # 最后显示信息
  local final_information=""

  # for循环运行所有待运行的函数
  for key in "${functions_array[@]}"; do
    # 运行相应函数
    if ! "$key"; then
      return 1
    fi
    # 修改、配置文件
    jq --arg key "$key" --arg disabled_flag "$disabled_flag" '.function[$key]=$disabled_flag' "$config_path" > temp.json && mv temp.json "$config_path"
    # 运行成功后计数加一
    func_executed_num=$((func_executed_num + 1))
  done
  # 补充最后的显示信息
  final_information+="All the temp files are in \"${path_download}\", and user can remove them according to needs."
  # 计数满足后全部刷新为y以方便下次运行
  if [ "$func_executed_num" -eq "${#functions_array[@]}" ]; then
    for key in "${functions_array[@]}"; do
      jq --arg key "$key" --arg enabled_flag "$enabled_flag" '.function[$key]=$enabled_flag' "$config_path" > temp.json && mv temp.json "$config_path"
    done
  fi
  # 显示最后信息
  log "$final_information"
  return 0
}

# ==============================
# 6. 主函数
# 在此处将对应的函数和标志进行trap，同时运行各项base函数
# ==============================

# 主函数
main() {
  set -e
  # 将中断处理函数赋予SIGINT标志
  trap interrupt_handler SIGINT
  if ! authority_check_base || ! parameter_check_base || ! install_dependencies_base || ! process_config_base || ! run_base; then
    return 1
  fi
  # 回到用户调用脚本的当前路径
  cd "$(pwd)"
  # 关闭报错即退出
  set +e
}

if ! main; then
  log "Minimal image build failed!"
else
  log "Minimal image build successful!"
fi
