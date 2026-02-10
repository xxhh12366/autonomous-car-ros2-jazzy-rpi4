import sys
import os
# Add Hardware directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from Hardware import servo_driver as Servo
from Hardware import motor_driver as Motor
from Hardware.millimeterwave_driver import openMMWPort, MMWDetection
from pyvesc.VESC.messages import SetDutyCycle
from Python_Nano_Ultrasonic_202302V1 import ultrasonicDetection
import socket
import struct
import time

IP = '192.168.3.242'                 # 主控板ip地址
FRONT_THRESHOLD = 65                # 前方毫米波距离阈值
RIGHT_MUTATION_THRESHOLD = 20       # 右方超声波距离阈值
MOTOR_DUTY_CYCLE = 0.07             # 执行LKS时电机占空比
TURN_DUTY_CYCLE = 0.07              # 转弯时电机占空比
TURN_LEFT_ANGLE_POSITION = 1000      # 左转时舵机控制输入
TURN_RIGHT_ANGLE_POSITION = 2200    # 右转时舵机控制输入
TURN_LEFT_COUNT = 38                # 左转延时计次
TURN_RIGHT_COUNT = 38               # 右转延时计次

TurnLeftCount, TurnRightCount = 0, 0
TurnLeftFlag, TurnRightFlag = True, False


# 卡尔曼滤波
class KF:
    x_prior, x_post = 0, 0  # 状态矩阵
    P_prior, P_post = 1, 1  # 协方差矩阵
    Q = 0.002               # 预测噪声
    R = 0.05                # 观测噪声
    K = 0                   # 卡尔曼系数

    def __init__(self, Q=None, R=None):
        # private
        self._xPrior = KF.x_prior
        self._xPost = KF.x_post
        self._PPrior = KF.P_prior
        self._PPost = KF.P_post
        self._Q = KF.Q if Q is None else Q  # 预测噪声
        self._R = KF.R if R is None else R  # 观测噪声
        self._K = KF.K

    def filter(self, data):
        """
        滤波，需放置在循环中执行

        :param data: 观测值，即传感器测量数据
        :return self._PPost: 卡尔曼后验估计
        :return data: 滤波前原始数据
        """
        # 预测
        self._xPrior = self._xPost
        self._PPrior = self._PPost + self._Q
        # 更新
        self._K = self._PPrior / (self._PPrior + self._R)
        self._xPost = self._xPrior + self._K * (data - self._xPrior)
        self._PPost = (1 - self._K) * self._PPrior

        return self._xPost, data


def transportCondition(angle: int): return [angle, True] if 1300 < angle < 1700 else [angle, False]
# 读取与处理传感器数据
def filteringMMWData(port): return MMWDetection(port)
def filteringUltrasonicData(kf: KF): return kf.filter(ultrasonicDetection())


def run(motorControl, serveControl):
    Motor.get_values_example(SetDutyCycle(motorControl))
    Servo.servo_angle_write(serveControl)


def PCSMain():
    """
    PCS主程序

    :return: None
    """
    global TurnLeftFlag, TurnRightFlag, TurnLeftCount, TurnRightCount
    socketLink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    socketLink.bind((IP, 5000))
    MMWPort = openMMWPort()
    ultra_kf = KF(Q=0.004, R=0.06)

    try:
        while True:
            # 读取舵机角度
            angle4LKS, directionFlag = transportCondition(int(struct.unpack('d', socketLink.recvfrom(2048)[0])[0]))
            # 获取传感器读数
            frontDistance, _ = filteringMMWData(MMWPort)
            rightDistance_KF, rightDistance = filteringUltrasonicData(ultra_kf)

            if directionFlag and TurnLeftFlag and 0 < frontDistance < FRONT_THRESHOLD:  # 左转
                # if abs(1500 - Python_Servo_RTech_202204V1.servo_angle_read()) <= 200:
                while TurnLeftCount <= TURN_LEFT_COUNT:
                    run(TURN_DUTY_CYCLE, TURN_LEFT_ANGLE_POSITION)
                    TurnLeftCount = TurnLeftCount + 1
                    print("turn left", "\tfront = ", frontDistance, "\tright = ", rightDistance_KF, "\t")

                TurnLeftFlag, TurnLeftCount = False, 0  # 准备直行LKS

            elif directionFlag and TurnRightFlag and rightDistance_KF > RIGHT_MUTATION_THRESHOLD:  # 右转
                # if abs(1500 - Python_Servo_RTech_202204V1.servo_angle_read()) <= 200:
                while TurnRightCount <= TURN_RIGHT_COUNT:
                    run(TURN_DUTY_CYCLE, TURN_RIGHT_ANGLE_POSITION)
                    TurnRightCount = TurnRightCount + 1
                    print("turn right", "\tfront = ", frontDistance, "\tright = ", rightDistance_KF, "\t")
            
                TurnLeftFlag, TurnRightFlag, TurnRightCount = True, False, 0    # 准备直行LKS
                
            elif not TurnLeftFlag:   # LKS
                for i in range(26):
                    run(MOTOR_DUTY_CYCLE, int(struct.unpack('d', socketLink.recvfrom(2048)[0])[0]))
                    print("LKS --", "\tfront = ", frontDistance, "\tright = ", rightDistance_KF, "\tservo: ", angle4LKS)

                TurnRightFlag = True    # 准备右转
                    
            else:   # LKS
                run(MOTOR_DUTY_CYCLE, angle4LKS)
                print("LKS --", "\tfront = ", frontDistance, "\tright = ", rightDistance_KF, "\tservo: ", angle4LKS)

                # TurnLeftFlag, TurnRightFlag = True, False   # 准备下一次PCS
            time.sleep(0.03)

    except KeyboardInterrupt:
        socketLink.close()
        MMWPort.close()
        del ultra_kf


if __name__ == "__main__":
    PCSMain()
