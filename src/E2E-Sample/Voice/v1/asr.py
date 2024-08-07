#!/usr/bin/env python3
#coding=utf-8
import smbus2
import time

# Speech recognition module address
i2c_addr = 0x0f
# Entry add address
asr_add_word_addr = 0x01
# Recognition mode setting address, the value is 0-2, 
# 0: cyclic recognition mode 1: password mode, 2: button mode, the default is cyclic detection
asr_mode_addr = 0x02
# RGB lamp setting address, need to send two bits, the first one is directly the lamp number 1: blue 2: red 3: green
# The second byte is brightness 0-255, the larger the value, the higher the brightness
asr_rgb_addr = 0x03   
# Identification sensitivity setting address, sensitivity can be set to 0x00-0x7f, the higher the value, the easier it is to detect but the easier it is to misjudge
# It is recommended to set the value to 0x40-0x55, the default value is 0x40
asr_rec_gain_addr = 0x04
# Clear the operation address of the power-off cache, clear the cache area information before entering the information
asr_clear_addr = 0x05
# Used in key mode, set the startup recognition mode
asr_key_flag = 0x06
# Used to set whether to turn on the recognition result prompt sound
asr_voice_flag = 0x07
# Recognition result storage address
asr_result = 0x08
# Buzzer control register, 1 bit is on, 0 bit is off
asr_buzzer = 0x09
# Check the number of entries
asr_num_cleck = 0x0a
# firmware version number
asr_vession = 0x0b
# Busy and busy flag
asr_busy = 0x0c

# Write entry
def AsrAddWords(idnum, str, bus):
    global i2c_addr
    global asr_add_word_addr
    words = []
    words.append(asr_add_word_addr)
    words.append(len(str) + 2)
    words.append(idnum)
    for  alond_word in str:
        words.append(ord(alond_word))
    words.append(0)
    for date in words:
        bus.write_byte(i2c_addr, date)
        time.sleep(0.03)

# Set RGB
def RGBSet(R,G,B, bus):
    global i2c_addr
    global asr_rgb_addr
    date = []
    date.append(R)
    date.append(G)
    date.append(B)
    bus.write_i2c_block_data(i2c_addr, asr_rgb_addr, date)

# Read result
def I2CReadByte(reg, bus):
    global i2c_addr
    bus.write_byte(i2c_addr, reg)
    time.sleep(0.05)
    Read_result = bus.read_byte(i2c_addr)
    return Read_result

# Wait busy
def Busy_Wait(bus):
    busy = 255
    while busy != 0:
        busy = I2CReadByte(asr_busy, bus)
        # print(asr_busy)

def init_voice_detector():
    bus = smbus2.SMBus(7)
    # Clear the power-down buffer area
    bus.write_byte_data(i2c_addr, asr_clear_addr, 0x40)
    # Wait for the module to be free
    Busy_Wait(bus)
    print("Cache cleared")
    # Set to loop mode
    bus.write_byte_data(i2c_addr, asr_mode_addr, 0x00)
    Busy_Wait(bus)

    AsrAddWords(0, "xiao ya", bus)
    Busy_Wait(bus)
    AsrAddWords(11, "kai deng", bus)
    Busy_Wait(bus)
    AsrAddWords(12, "guan deng", bus)
    Busy_Wait(bus)

    # Set the sensitivity, the recommended value is 0x40-0x55
    bus.write_byte_data(i2c_addr, asr_rec_gain_addr, 0x40)
    # Set switch sound
    bus.write_byte_data(i2c_addr, asr_voice_flag, 1)
    # buzzer
    bus.write_byte_data(i2c_addr, asr_buzzer, 1)
    RGBSet(255, 255, 255, bus)
    time.sleep(1)
    RGBSet(0, 0, 0, bus)
    # buzzer
    bus.write_byte_data(i2c_addr, asr_buzzer, 0)
    return bus
