                                                                #!/usr/bin/env python3
# coding=UTF-8
# ------------------------------------------------------------------ #
# @file    : motor_driver.py
# @platform: Python
# @board   : Jetson Nano
# @brief   : 电机（3650 21.5T）控制程序 - Consolidated motor driver
# 
# @author  : WeiJiaHao
# @date    : 2023/2
# @web     : http://www.relaxingtechnology.com/
# @e-mail  : relaxingtech@qq.com
# Copyright (C) 2023 Relaxing Technology Chongqing Co.,Ltd. All rights reserved.
# ------------------------------------------------------------------ #
import pyvesc
from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
import serial

# Default configuration - can be overridden when calling functions
serialport = '/dev/motor'  # 电机-电调串口号
Trans_Ratio = 6.287  # 传动比
Wheel_Radius = 0.032  # 轮胎半径，m


def printSpeed(rpm):
    """
    打印车速

    :param rpm: response.rpm, 电机转速
    :return: velocity in cm/s
    """
    velocity = rpm / Trans_Ratio * (2 * 3.14 * Wheel_Radius) / 60 * 100
    print("小车的速度为%.6fcm/s" % velocity)
    return velocity


def get_values_example(SetMode, port=None):
    """
    从VESC读取电机转速（RPM），然后计算车轮的线速度（单位：厘米/秒）
    
    :param SetMode: SetDutyCycle or SetRPM message
    :param port: Serial port, defaults to serialport if None
    :return: velocity in cm/s, or 0 on error
    """
    port = port or serialport
    try:
        with serial.Serial(port, baudrate=115200, timeout=0.01) as ser:
            # 将 SetMode消息编码后发送给VESC，然后请求VESC返回当前的电机状态（包括转速）
            ser.write(pyvesc.encode(SetMode))
            ser.write(pyvesc.encode_request(GetValues))
            (response, consumed) = pyvesc.decode(ser.read(78))
            if consumed == 78:
                # 计算并返回车轮的线速度；否则返回错误信息
                return response.rpm / Trans_Ratio * (2 * 3.14 * Wheel_Radius) / 60 * 100  # cm/s
            else:
                return 0
    except Exception as e:
        print(f"Motor driver error: {e}")
        return 0


if __name__ == "__main__":
    # Test code
    SetDutyCycle_Values = 0.07
    SetMode = SetDutyCycle(SetDutyCycle_Values)
    
    try:
        while True:
            speed = get_values_example(SetMode)
            print(f"Current speed: {speed} cm/s")
    except KeyboardInterrupt:
        # Stop motor on exit
        get_values_example(SetDutyCycle(0))
        print("\nMotor stopped")                                    