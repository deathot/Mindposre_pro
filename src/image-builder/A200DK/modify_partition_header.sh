#!/bin/bash

# ************************writePartitionHeader**************************************
# Description:  write partirion header
# ******************************************************************************
function writePartitionHeader()
{
    #sector 512
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
