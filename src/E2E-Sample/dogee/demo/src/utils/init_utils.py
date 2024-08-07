import subprocess

from src.utils.constant import STM32_NAME,ESP32_NAME,FIND_PORT_CODE
from src.utils.logger import logger_instance as log


def get_port(device_name):
    usb_list = subprocess.check_output(
        FIND_PORT_CODE, shell=True, executable='/bin/bash').decode().split('\n')

    # get all serial port list
    usb_port_dict = {value: key for info in usb_list if info !=
                     '' for key, value in [info.split(' - ')]}

    tty_port = usb_port_dict.get(device_name, None)
    log.info(f"{device_name}'s port is {tty_port}")
    return tty_port
