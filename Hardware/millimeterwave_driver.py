# #!/usr/bin/env python3
# coding=UTF-8
# ------------------------------------------------------------------ #
# @file    : millimeterwave_driver.py
# @platform: Python
# @board   : Jetson Nano
# @brief   : 毫米波雷达（HLR12）驱动程序 - Consolidated MMWR driver
# 
# Jetson Nano 8号引脚是UART_TX，10号引脚是UART_RX
# 毫米波HLR12通讯协议
# 读数据
# 地址码（2字节）数据长度（1字节）指令码（1字节）检测距离（2字节）速度数据（2字节）信号强度（2字节）手势识别（1字节）雷达关闭提示（1字节）和校验（1字节）
# 地址码：0x55 0xA5
# 数据长度：0x0A，从数据长度到雷达关闭提示的总字节数
# 指令码：0xD1 - 开关雷达指令，0xD2 - 设置波特率，0xD3 - 查询微波检测信息
# 距离、速度：两个字节为高八位、低八位，默认先发送高八位，速度有正负，正数表示远离，负数表示接近
# 手势识别：1表识别到手势（摆手），0表未识别到手势
# 
# @author  : WeiJiaHao
# @date    : 2023/2
# @web     : http://www.relaxingtechnology.com/
# @e-mail  : relaxingtech@qq.com
# Copyright (C) 2023 Relaxing Technology Chongqing Co.,Ltd. All rights reserved.
# ------------------------------------------------------------------ #
import serial

# 声明端口号与比特率
MMWR_PORT = "/dev/ttyTHS1"
MMWR_BAUD_RATE = 115200


def openMMWPort(port=MMWR_PORT, baudRate=MMWR_BAUD_RATE) -> serial.Serial:
    """
    打开串口
    
    :param port: 串口设备路径
    :param baudRate: 波特率
    :return uart: serial.Serial, 串口类
    """
    try:
        uart = serial.Serial(port, baudRate, timeout=5)
    except Exception as error:
        print("Failed to open serial port\n", error)
        exit(0)
    else:
        return uart


def checkData(bytesData: bytes) -> bool:
    """
    使用不进位累加校验和来验证数据

    :param bytesData: 要校验的字节数据
    :return result: bool, True -> 校验成功, False -> 校验失败
    """
    checkSum = 0
    for bit in bytesData:
        checkSum = checkSum + bit

    # 在这里校验
    return bytesData[-1] == (checkSum - bytesData[-1])


def analyseData(bytesData: bytes):
    """
    解析毫米波雷达数据的函数，从原始字节数据中提取距离和速度信息

    :param bytesData: bytes, 串口读取的字节数据
    :return distance: int, 距离 (cm)
    :return speed: int, 相对速度 (cm/s)
    """
    distance = ((bytesData[4] << 8) | bytesData[5])  # 高八位，低八位组合
    speedData_untreated = (bytesData[6] << 8) | bytesData[7]

    if speedData_untreated & 0x8000 == 0x8000:  # 速度为负
        speed = -((speedData_untreated - 1) ^ 0xFFFF)
    else:  # 速度为正
        speed = speedData_untreated

    return distance, speed


def printData(distance: int, speed: int):
    """
    打印距离和速度

    :param distance: int, 距离
    :param speed: int, 相对速度
    """
    print("distance = ", distance, "cm", end="\t")
    print("relative_speed = ", speed, "cm/s")


def MMWDetection(uart: serial.Serial):
    """
    毫米波检测函数，需放置在循环中执行

    :param uart: serial.Serial, 串口对象
    :return distance: int, 距离 (cm)
    :return speed: int, 速度 (cm/s)
    """
    try:
        distance = 0
        speed = 0
        while True:
            if uart.is_open and uart.in_waiting:  # 未检测到数据时串口没有字节缓存
                uartData = uart.read(uart.in_waiting)
                if checkData(uartData):  # 校验数据
                    distance, speed = analyseData(uartData)  # 解析数据
                    # printData(distance, speed)  # 可选：打印数据
                    break
                else:
                    print("Data validation failed")  # 数据校验失败
        return distance, speed
    except KeyboardInterrupt:
        uart.close()
        exit(0)


if __name__ == "__main__":
    Uart = openMMWPort(port=MMWR_PORT, baudRate=MMWR_BAUD_RATE)  # 打开串口
    print("毫米波雷达驱动程序启动...")
    try:
        while True:
            distance, speed = MMWDetection(Uart)
            if distance > 0:
                printData(distance, speed)
    except KeyboardInterrupt:
        Uart.close()
        print("\n串口已关闭")