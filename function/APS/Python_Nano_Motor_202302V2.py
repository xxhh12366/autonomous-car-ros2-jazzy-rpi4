#!/usr/bin/env python3
#coding=UTF-8
# ------------------------------------------------------------------ #
# @file    : Python_Nano_Motor_202302V2.py
# @platform: Python
# @board   : Jetson Nano
# @brief   : 电机（3650 21.5T）控制程序
# 
# @author  : WeiJiaHao
# @date    : 2023/2
# @web     : http ://www.relaxingtechnology.com/
# @e-mail  : relaxingtech@qq.com
# Copyright (C) 2023 Relaxing Technology Chongqing Co.,Ltd. All rights reserved.
# ------------------------------------------------------------------ #
import pyvesc
from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
import serial
import time

serialport = '/dev/ttyUSB1'
Trans_Ratio = 6.287  # 传动比
Wheel_Radius = 0.032  # 轮胎半径，m


def get_values_example(SetMode):
    with serial.Serial(serialport, baudrate=115200, timeout=0.01) as ser:
        ser.write(pyvesc.encode(SetMode))
        ser.write(pyvesc.encode_request(GetValues))
        (response, consumed) = pyvesc.decode(ser.read(78))
        if consumed == 78:
            # print("rpm=",response.rpm)
            # velocity =printSpeed(response.rpm)
            return response.rpm / Trans_Ratio * (2 * 3.14 * Wheel_Radius) / 60 * 100    # cm/s
        else:
            # print('error')
            return 0


if __name__ == "__main__":
    # SetRPM_Values = 900   
    # SetMode = SetRPM(SetRPM_Values)
    SetDutyCycle_Values = 0.07
    SetMode = SetDutyCycle(SetDutyCycle_Values)

    while True:
        print(get_values_example(SetMode))
    get_values_example(SetDutyCycle(0))
