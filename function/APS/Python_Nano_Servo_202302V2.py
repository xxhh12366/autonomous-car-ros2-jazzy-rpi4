#!/usr/bin/env python3
#coding=UTF-8
# ------------------------------------------------------------------ #
# @file    : Python_Nano_Servo_202302V2.py
# @platform: Python
# @board   : Jetson Nano
# @brief   : 舵机（SCS20-360T）控制程序
# 
# @author  : WeiJiaHao
# @date    : 2023/2s
# @web     : http ://www.relaxingtechnology.com/
# @e-mail  : relaxingtech@qq.com
# Copyright (C) 2023 Relaxing Technology Chongqing Co.,Ltd. All rights reserved.
# ------------------------------------------------------------------ #
import os
from scservo_sdk import *  # Uses SCServo SDK library
import serial

# Control table address
ADDR_SCS_TORQUE_ENABLE = 40
ADDR_SCS_GOAL_ACC = 41
ADDR_SCS_GOAL_POSITION = 42
ADDR_SCS_GOAL_SPEED = 46
ADDR_SCS_PRESENT_POSITION = 56

# Default setting
SCS_ID = 1  # SCServo ID : 1
BAUDRATE = 1000000  # SCServo default baudrate : 1000000
DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

SCS_MINIMUM_POSITION_VALUE = 800
SCS_MAXIMUM_POSITION_VALUE = 2200
SCS_MOVING_STATUS_THRESHOLD = 20  # SCServo moving status threshold
SCS_MOVING_SPEED = 0  # SCServo moving speed
SCS_MOVING_ACC = 50  # SCServo moving acc
protocol_end = 0  # SCServo bit end(STS/SMS=0, SCS=1)

scs_goal_position = 1550  # Goal position
# 初始化端口等项
portHandler = PortHandler(DEVICENAME)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)
# portHandler = serial.Serial(DEVICENAME, BAUDRATE)
# print(portHandler.is_open)
packetHandler = PacketHandler(protocol_end)

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

# 比起旧版本多加了上下界的判断
def servo_angle_write(target_position):
    target_position = SCS_MAXIMUM_POSITION_VALUE if target_position > SCS_MAXIMUM_POSITION_VALUE else target_position
    target_position = SCS_MINIMUM_POSITION_VALUE if target_position < SCS_MINIMUM_POSITION_VALUE else target_position
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID, ADDR_SCS_GOAL_POSITION,
                                                              target_position)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))


def servo_angle_read():
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
    servo_angle_write(scs_goal_position)
    scs_present_position_speed = 0
    while True:
        print(servo_angle_read())
        if not (abs(scs_goal_position - scs_present_position_speed) > SCS_MOVING_STATUS_THRESHOLD):
            break
    portHandler.closePort()
