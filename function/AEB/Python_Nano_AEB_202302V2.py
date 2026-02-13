#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AEB - 自动紧急制动系统 (Automatic Emergency Braking)

功能说明：
使用毫米波雷达检测前方障碍物距离，当距离小于设定阈值时自动紧急刹车。
这是一项重要的主动安全功能，可以有效避免或减轻碰撞事故。

工作原理：
1. 持续监测毫米波雷达数据
2. 获取前方障碍物距离和相对速度
3. 判断距离是否小于安全阈值(30cm)
4. 若距离过近，立即停止电机(占空比设为0)
5. 若距离安全，保持正常行驶(占空比0.1)

硬件配置：
- 毫米波雷达串口: /dev/ttyTHS1
- 波特率: 115200
- 电机控制: VESC (通过Motor模块)

安全参数：
- 距离阈值: 30cm (可根据车速调整)
- 电机占空比: 0.1 (正常行驶)
- 紧急制动: 占空比设为0

作者: xxhh12366
日期: 2023/02
版权所有 (C) 2023 Relaxing Technology Chongqing Co.,Ltd.
"""

from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
import Python_Nano_Motor_202302V2 as Motor
import Python_Nano_MMWR_202302V2 as MMWR


# 距离阈值设定 (单位: cm)
# 当检测到障碍物距离小于此值时触发紧急制动
DISTANCE_THRESHOLD = 30

# 正常行驶时的电机占空比 (范围: 0-1)
MOTOR_DUTY_CYCLE = 0.1

# 毫米波雷达串口配置
MMWR_PORT = "/dev/ttyTHS1"   # 毫米波雷达串口地址
MMWR_BAUD_RATE = 115200      # 毫米波雷达波特率


def AEB():
    """
    AEB主控制函数
    
    功能流程：
    1. 打开毫米波雷达串口通信
    2. 进入主循环，持续监测前方障碍物
    3. 根据检测距离决定是否紧急制动
    
    触发条件：
    - 距离 > 0 且 距离 <= 30cm: 触发紧急制动
    - 其他情况: 保持正常行驶
    
    返回：
    无返回值，检测到障碍物紧急制动后程序退出
    """
    # 打开毫米波雷达串口
    Uart = MMWR.openMMWPort(MMWR_PORT, MMWR_BAUD_RATE)
    
    print("=" * 60)
    print("AEB 自动紧急制动系统启动")
    print(f"距离阈值: {DISTANCE_THRESHOLD} cm")
    print(f"正常占空比: {MOTOR_DUTY_CYCLE}")
    print("=" * 60)

    while True:
        # 获取毫米波雷达检测数据
        # distance: 障碍物距离 (cm)
        # speed: 相对速度 (可用于预判碰撞风险)
        distance, speed = MMWR.MMWDetection(Uart)
        
        # 判断是否需要紧急制动
        # 条件: 距离有效(>0) 且 距离小于等于阈值
        if 0 < distance <= DISTANCE_THRESHOLD:
            print(f"警告! 检测到障碍物距离: {distance}cm")
            print("触发紧急制动!")
            
            # 紧急停车: 将电机占空比设为0
            Motor.get_values_example(SetDutyCycle(0))
            
            # 制动完成，退出程序
            exit(0)
        else:
            # 距离安全，保持正常行驶
            Motor.get_values_example(SetDutyCycle(MOTOR_DUTY_CYCLE))
            
            # 输出当前状态（可选，用于调试）
            if distance > 0:
                print(f"前方距离: {distance}cm - 安全")


if __name__ == "__main__":
    """
    程序入口
    启动AEB自动紧急制动系统
    """
    try:
        AEB()
    except KeyboardInterrupt:
        # 用户手动中断程序 (Ctrl+C)
        print("\n程序已停止")
        exit(0)
    except Exception as e:
        # 捕获其他异常
        print(f"错误: {e}")
        exit(1)
