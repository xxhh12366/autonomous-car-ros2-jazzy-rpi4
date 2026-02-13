#!/usr/bin/env python3
#coding=UTF-8
# ------------------------------------------------------------------ #
# @file    : Python_Nano_APS_202304v2.py
# @platform: Python
# @board   : Jetson Nano
# @brief   : APS
# 
#            超参数：
#            - IP: str, 表示Python_Nano_Servo4APS_[versions].py与该程序通讯的IP地址
#                   在终端使用ifconfig指令查看IP地址
#            - DEFAULT_DUTY: float, 电机默认占空比
#            - DEFAULT_POSITION: int, 舵机默认转角位置
#            - WHEEL_BASE: int or float, 小车轴距
#            - ULTRASONIC_THRESHOLD: int or float, 超声波检测阈值
#                   小于该阈值被认为存在障碍物
#            - PARKING_DIRECTION: str, "right"或"left", 表示停车位在小车右边或左边
#            - PARKING_LENGTH_THRESHOLD: int or float, 表示水平车位于垂直车位的长度阈值
#                   小于该阈值被认为是垂直停车车位；大于该阈值被认为是水平停车位
#
#            function:
#            - levelParking(): 调整水平泊车轨迹
#            - verticalParking(): 调整垂直泊车轨迹
#            - runModel(): 轨迹控制的基本模块，控制电机舵机运行直至一定时间或转过一定角度
#
# @author  : Zhonglw
# @date    : 2023/5
# @web     : http ://www.relaxingtechnology.com/
# @e-mail  : relaxingtech@qq.com
# Copyright (C) 2023 Relaxing Technology Chongqing Co.,Ltd. All rights reserved.
# ------------------------------------------------------------------ #
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'Hardware'))

from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
from motor_controller import MotorController
from servo_controller import ServoController
from config import MOTOR_CONFIG, SERVO_CONFIG
import Python_Nano_Ultrasonic_202305v2 as Ultra
import time
import socket #网络通信
import struct #二进制数据打包/解包
import math

# Initialize motor and servo controllers
motor = MotorController(
    serial_port=MOTOR_CONFIG['serial_port'],
    baudrate=MOTOR_CONFIG['baudrate'],
    trans_ratio=MOTOR_CONFIG['trans_ratio'],
    wheel_radius=MOTOR_CONFIG['wheel_radius'],
    timeout=MOTOR_CONFIG['timeout']
)

servo = ServoController(
    device_name=SERVO_CONFIG['device_name'],
    servo_id=SERVO_CONFIG['servo_id'],
    baudrate=SERVO_CONFIG['baudrate'],
    min_position=SERVO_CONFIG['min_position'],
    max_position=SERVO_CONFIG['max_position'],
    moving_speed=SERVO_CONFIG['moving_speed'],
    moving_acc=SERVO_CONFIG['moving_acc'],
    protocol_end=SERVO_CONFIG['protocol_end']
)


IP = '10.10.100.43'
DEFAULT_DUTY = 0.06
DEFAULT_POSITION = 1500
WHEEL_BASE = 25     # cm
ULTRASONIC_THRESHOLD = 45   # cm
PARKING_DIRECTION = "left"     # right or left
PARKING_LENGTH_THRESHOLD = 55 # cm

Step = 1
LastData = 0

"""
类功能概述：
UltraMF是 Ultra.UltraObj的子类，增加了中值滤波功能：
原始功能：超声波测距
新增功能：对多次测量结果进行排序，取中位数，消除异常值

"""

class UltraMF(Ultra.UltraObj):
    def __init__(self, filterLength: int, trigPin, echoPin):
        # 继承
        super(UltraMF, self).__init__(trigPin, echoPin, mode=Ultra.BOARD, unit="cm")
        
        self.origialData = [0.0] * int(filterLength)
        self.listLen = filterLength

    def detection(self):
        distance = Ultra.UltraObj.detection(self)
        return self._filter(distance)

    def _filter(self, data: float):
        self.origialData = self.origialData[1:]
        self.origialData.append(data)

        self.filterData = UltraMF._quicksort(self.origialData)  # 排序

        number = self.filterData[self.listLen // 2]
        if self.listLen % 2 == 1:
            return float(number)
        else:
            return float(0.5 * (number + self.filterData[self.listLen // 2 - 1]))

    @staticmethod   # 静态方法
    def _quicksort(arr: list):
        if len(arr) <= 1:
            return arr
        else:
            pivot = arr[len(arr) // 2]
            left = [x for x in arr if x < pivot]
            mid = [x for x in arr if x == pivot]
            right = [x for x in arr if x > pivot]
        return UltraMF._quicksort(left) + mid + UltraMF._quicksort(right)


# 从图像处理中获取舵机角度位置
def servoPositionSocket():
    socketLink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    socketLink.bind((IP, 5000))
    return int(struct.unpack('d', socketLink.recvfrom(2048)[0])[0])


# 电机、舵机控制
def run(position=DEFAULT_POSITION, duty=DEFAULT_DUTY, stopFlag=False):
    duty = 0 if stopFlag else duty
    servo.write_position(int(position))
    velocity = motor.get_velocity(SetDutyCycle(duty))
    if stopFlag:
        time.sleep(1)
    current_position, _ = servo.read_position()
    return current_position, velocity


# 检测停车位
def checkParking(startTime):
    global LastData
    atPosition, atVelocity = DEFAULT_POSITION, 0
    obstacleFlag, detectionFlag = False, False
    obstacleDis = []

    while True:
        position = servoPositionSocket()

        distance = RightUltra.detection() if PARKING_DIRECTION == "right" else LeftUltra.detection()

        if 5 < distance <= ULTRASONIC_THRESHOLD and not obstacleFlag:
            LastData = [distance, atVelocity, time.time()]
            detectionFlag = True

        if distance > ULTRASONIC_THRESHOLD and detectionFlag:
            obstacleDis.append(LastData)
            obstacleFlag = True

        if obstacleFlag and distance <= ULTRASONIC_THRESHOLD:
            obstacleDis.append([distance, atVelocity, time.time()])  
            break

        atPosition, atVelocity = run(position)

        if (time.time() - startTime) * atVelocity > 600:
            raise KeyboardInterrupt
        
    run(DEFAULT_POSITION, 0)
    return obstacleDis


def runModel(position=DEFAULT_POSITION, duty=DEFAULT_DUTY, **kwargs):
    """
    控制模块，控制电机舵机运行直至一定时间或转过一定角度

    :param position: 舵机角度位置，默认值DEFAULT_POSITION
    :param duty: 电机占空比，默认DEFAULT_DUTY
    :**kwargs: 运行结束条件，time或alpha
    
        time: 运行一定时间后结束，单位s
        alpha: 转过一定角度后结束，角度制
        
        Examples:
        >>> runModel(position=900, duty=0.08, time=2)
        >>> 舵机角度位置为900，电机占空比为0.08，运行2秒

        >>> runModel(duty=0.07, alpha=60)
        >>> 舵机角度位置为默认值DEFAULT_POSITION，电机占空比为0.07，小车转过60度后结束运行

    """
    global Step
    print(f"\n---------- step {Step} ----------")

    atPosition, atVelocity = run(position, duty=0, stopFlag=True)
    vel, pos = [atVelocity], [atPosition]
    startTime = time.time()

    while True:
        aveVelocity = sum(vel) / len(vel)
        aveTheta = math.radians(sum(pos) / len(pos) / 20 - 75)
        atAlpha = abs(aveVelocity) * (time.time() - startTime) * math.tan(abs(aveTheta)) / WHEEL_BASE
        
        if "alpha" in kwargs.keys():
            if atAlpha >= math.radians(kwargs["alpha"]):
                break
        elif "time" in kwargs.keys():
            if time.time() - startTime >= kwargs["time"]:
                break
        atPosition, atVelocity = run(position, duty)
        
        vel.append(atVelocity)
        pos.append(atPosition)
    run(stopFlag=True)
    Step += 1


# 垂直泊车
def verticalParking():
    if PARKING_DIRECTION == "right":
        # runModel(duty=- (DEFAULT_DUTY + 0.01), time=0.2)
        runModel(position=800, alpha=30)
        runModel(position=2200, duty=- (DEFAULT_DUTY + 0.01), alpha=90)
        runModel(duty=- (DEFAULT_DUTY + 0.01), time=0.4)
    
    elif PARKING_DIRECTION == "left":
        runModel(position=2200, alpha=30)
        runModel(position=800, duty=- (DEFAULT_DUTY + 0.01), alpha=80)
        runModel(duty=- (DEFAULT_DUTY + 0.01), time=0.8)


# 水平泊车
def levelParking():
    if PARKING_DIRECTION == "right":
        runModel(duty=DEFAULT_DUTY + 0.01, time=0.8)
        runModel(position=2200, duty=- (DEFAULT_DUTY + 0.01), alpha=60)
        runModel(duty=- (DEFAULT_DUTY + 0.01), time=0.6)
        runModel(position=800, duty=- (DEFAULT_DUTY + 0.01), alpha=30)
        runModel(position=2200, duty=DEFAULT_DUTY + 0.01, alpha=15)
        runModel(duty=- (DEFAULT_DUTY + 0.01), time=0.4)

    if PARKING_DIRECTION == "left":
        runModel(duty=DEFAULT_DUTY + 0.01, time=0.7)
        runModel(position=800, duty=- (DEFAULT_DUTY + 0.01), alpha=70)
        runModel(duty=(DEFAULT_DUTY + 0.01), time=0.7)
        runModel(position=2200, duty=- (DEFAULT_DUTY + 0.01), alpha=60)
        runModel(position=800, duty=DEFAULT_DUTY + 0.01, alpha=15)
        runModel(duty=- (DEFAULT_DUTY + 0.01), time=0.4)


if __name__ == "__main__":
    RightUltra = UltraMF(5, Ultra.RIGHT_TRIG_PIN, Ultra.RIGHT_ECHO_PIN)
    LeftUltra = UltraMF(5, Ultra.LEFT_TRIG_PIN, Ultra.LEFT_ECHO_PIN)

    servo.write_position(DEFAULT_POSITION)
    time.sleep(1)

    try:
        servoPositionSocket()   # 等待视觉程序启动

        # 检测停车位，建议使用纸箱为障碍物
        startTime = time.time()
        while True:
            obstacleDis = checkParking(startTime)

            aveDistance = sum([row[0] for row in obstacleDis]) / len(obstacleDis)   # 平均检测距离
            aveVelocity = sum([row[1] for row in obstacleDis]) / len(obstacleDis)   # 平均车速
            
            parkingLength = aveVelocity * (obstacleDis[-1][2] - obstacleDis[-2][2])
            print("parking lehgth = ", parkingLength)
            if 10 <= parkingLength <= PARKING_LENGTH_THRESHOLD:
                print("Vertical parking space detected.")
                parkingType = 1
                break
            elif PARKING_LENGTH_THRESHOLD <= parkingLength <= 140:
                print("Horizontal parking space detected.")
                parkingType = 0
                break

            if (time.time() - startTime) * aveVelocity > 600:
                print("No parking space detected.")
                raise KeyboardInterrupt
            print("--------------------")
        
        time.sleep(1)

        if parkingType == 0:    # 水平
            levelParking()

        elif parkingType == 1:  # 垂直
            verticalParking()
        
    except KeyboardInterrupt:
        exit(0)
