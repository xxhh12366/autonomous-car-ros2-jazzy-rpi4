
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PID 电机转速控制器

功能说明：
本程序实现基于VESC的电机转速PID闭环控制。
通过串口与VESC电调通信，读取当前转速，使用PID算法计算输出，
调整电机占空比，使转速稳定在目标值。

技术参数：
- 串口: /dev/ttyS0
- 波特率: 115200
- 目标转速: 2000 RPM
- PID参数: Kp=0.8, Ki=1.0, Kd=0.01
- 输出限制: ±3000 RPM

控制流程：
1. 设置初始占空比
2. 发送VESC控制指令
3. 读取当前电机转速
4. 计算转速误差
5. PID控制器计算输出
6. 转换为占空比并应用
7. 循环执行

作者: xxhh12366
日期: 2023
"""

import pyvesc
from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
import serial
from simple_pid import PID

# 串口配置
serialport = '/dev/ttyS0'  # VESC串口地址

# 目标转速设定 (RPM)
Target_rpm = 2000

# PID控制器初始化
# PID(Kp, Ki, Kd, setpoint)
# Kp: 比例系数，决定响应速度
# Ki: 积分系数，消除稳态误差
# Kd: 微分系数，减少超调
pid = PID(0.8, 1, 0.01, setpoint=0)

# PID输出限制，防止输出过大
pid.output_limits = (-Target_rpm*1.5, Target_rpm*1.5)


def get_values_example(SetMode):
    """
    与VESC电调通信，设置控制模式并读取当前转速
    
    参数:
        SetMode: VESC控制指令（SetDutyCycle或SetRPM对象）
    
    返回:
        float: 当前电机转速(RPM)，通信失败返回'error'
    
    通信流程:
        1. 打开串口连接 (波特率115200, 超时0.01s)
        2. 发送控制模式指令
        3. 发送GetValues请求
        4. 读取78字节响应数据
        5. 解码并返回RPM值
    """
    with serial.Serial(serialport, baudrate=115200, timeout=0.01) as ser:
        # 发送设置指令（占空比或转速）
        ser.write(pyvesc.encode(SetMode))
        
        # 请求获取电机状态信息
        ser.write(pyvesc.encode_request(GetValues))
        
        # 读取并解码响应数据
        (response, consumed) = pyvesc.decode(ser.read(78))
        
        # 检查是否成功读取完整数据包(78字节)
        if consumed == 78:
            # 返回当前转速值
            return response.rpm
        else:
            # 通信失败
            return 'error'



                    
if __name__ == "__main__":
    """
    主控制循环
    
    控制逻辑：
    1. 初始化占空比为0.05 (5%)
    2. 进入无限循环
    3. 读取当前转速
    4. 计算转速误差 (当前转速 - 目标转速)
    5. PID计算输出调整量
    6. 将PID输出转换为占空比
    7. 应用新的占空比
    
    占空比计算公式：
    Duty_PID = RPM_PID/15200 + 1/152
    - 15200: 转速到占空比的转换系数
    - 1/152: 占空比偏置量
    """
    
    # 初始占空比设置为5%
    SetDutyCycle_Values = 0.05
    SetMode = SetDutyCycle(SetDutyCycle_Values)
    Duty_PID = SetDutyCycle_Values
    
    print("=" * 60)
    print("PID电机转速控制器启动")
    print(f"目标转速: {Target_rpm} RPM")
    print(f"PID参数: Kp=0.8, Ki=1.0, Kd=0.01")
    print(f"初始占空比: {SetDutyCycle_Values * 100}%")
    print("=" * 60)
    
    # 主控制循环
    while True:
        # 创建占空比控制指令
        SetMode = SetDutyCycle(Duty_PID)
        
        # 获取当前电机转速
        RPM_NOW = get_values_example(SetMode)
        
        # 如果通信成功
        if RPM_NOW != 'error':
            # 计算转速误差（实际转速 - 目标转速）
            RPM_ERR = RPM_NOW - Target_rpm
            
            # PID控制器计算输出
            # pid(RPM_ERR)返回调整量，加上当前转速得到期望转速
            RPM_PID = pid(RPM_ERR) + RPM_NOW
            
            # 将期望转速转换为占空比
            # 转换公式: Duty = RPM/15200 + 1/152
            Duty_PID = RPM_PID / 15200 + 1 / 152
            
            # 输出调试信息
            print("RPM_NOW = ", RPM_NOW, "RPM_ERR = ", RPM_ERR, "RPM_PID = ", RPM_PID)
            print("Duty_PID = ", Duty_PID)
        else:
            # 通信失败，跳过本次循环
            print("通信错误: 无法读取电机转速")
            pass                                    