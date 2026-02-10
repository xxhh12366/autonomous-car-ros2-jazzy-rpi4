#!/usr/bin/env python3
# coding=UTF-8
# ------------------------------------------------------------------ #
# @file    : servo_driver.py
# @platform: Python
# @board   : Jetson Nano
# @brief   : 舵机（SCS20-360T）控制程序 - Consolidated servo driver
# 
# @author  : WeiJiaHao
# @date    : 2023/2
# @web     : http://www.relaxingtechnology.com/
# @e-mail  : relaxingtech@qq.com
# Copyright (C) 2023 Relaxing Technology Chongqing Co.,Ltd. All rights reserved.
# ------------------------------------------------------------------ #
import os
from scservo_sdk import *  # Uses SCServo SDK library

# Control table address
ADDR_SCS_TORQUE_ENABLE = 40
ADDR_SCS_GOAL_ACC = 41
ADDR_SCS_GOAL_POSITION = 42
ADDR_SCS_GOAL_SPEED = 46
ADDR_SCS_PRESENT_POSITION = 56

# Default setting
SCS_ID = 1  # SCServo ID : 1
BAUDRATE = 1000000  # SCServo default baudrate : 1000000
DEVICENAME = '/dev/servo'  # 舵机控制板串口号
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

SCS_MINIMUM_POSITION_VALUE = 800
SCS_MAXIMUM_POSITION_VALUE = 2200
SCS_MOVING_STATUS_THRESHOLD = 20  # SCServo moving status threshold
SCS_MOVING_SPEED = 0  # SCServo moving speed
SCS_MOVING_ACC = 50  # SCServo moving acc
protocol_end = 0  # SCServo bit end(STS/SMS=0, SCS=1)

scs_goal_position = 1500  # Goal position
scs_present_position_speed = 0  # Global variable for position/speed

# 初始化端口等项
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(protocol_end)

# Initialize port only if this module is imported (not when run as script)
try:
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)
    
    # Write SCServo acc
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS_ID, ADDR_SCS_GOAL_ACC, SCS_MOVING_ACC)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))
    
    # Write SCServo speed
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID, ADDR_SCS_GOAL_SPEED, SCS_MOVING_SPEED)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))
except Exception as e:
    print(f"Servo initialization warning: {e}")


def servo_angle_write(target_position):
    """
    写入舵机目标角度位置（比起旧版本多加了上下界的判断，保护舵机）
    
    :param target_position: Target position value
    """
    target_position = SCS_MAXIMUM_POSITION_VALUE if target_position > SCS_MAXIMUM_POSITION_VALUE else target_position
    target_position = SCS_MINIMUM_POSITION_VALUE if target_position < SCS_MINIMUM_POSITION_VALUE else target_position
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID, ADDR_SCS_GOAL_POSITION,
                                                              target_position)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))


def servo_angle_read():
    """
    读取舵机当前位置（分离位置和速度）
    
    :return: Current servo position
    """
    global scs_present_position_speed
    scs_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, SCS_ID,
                                                                                         ADDR_SCS_PRESENT_POSITION)
    if scs_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print(packetHandler.getRxPacketError(scs_error))

    scs_present_position = SCS_LOWORD(scs_present_position_speed)
    scs_present_speed = SCS_HIWORD(scs_present_position_speed)
    # print("[ID:%03d] GoalPos:%03d PresPos:%03d PresSpd:%03d"
    #       % (SCS_ID, scs_goal_position, scs_present_position, SCS_TOHOST(scs_present_speed, 15)))
    return scs_present_position


if __name__ == '__main__':
    # Test code
    servo_angle_write(scs_goal_position)
    while True:
        current_pos = servo_angle_read()
        print(f"Current position: {current_pos}")
        if not (abs(scs_goal_position - scs_present_position_speed) > SCS_MOVING_STATUS_THRESHOLD):
            break
    portHandler.closePort()                                    