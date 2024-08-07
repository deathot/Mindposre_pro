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
    return 0
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
{
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
