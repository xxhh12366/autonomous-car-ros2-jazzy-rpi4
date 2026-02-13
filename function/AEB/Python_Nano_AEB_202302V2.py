import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'Hardware'))

from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
from motor_controller import MotorController
from config import MOTOR_CONFIG
import Python_Nano_MMWR_202302V2 as MMWR

# Initialize motor controller
motor = MotorController(
    serial_port=MOTOR_CONFIG['serial_port'],
    baudrate=MOTOR_CONFIG['baudrate'],
    trans_ratio=MOTOR_CONFIG['trans_ratio'],
    wheel_radius=MOTOR_CONFIG['wheel_radius'],
    timeout=MOTOR_CONFIG['timeout']
)


DISTANCE_THRESHOLD = 30     # AEB设定阈值
MOTOR_DUTY_CYCLE = 0.1     # 电机占空比

MMWR_PORT = "/dev/ttyTHS1"   # 毫米波串口
MMWR_BAUD_RATE = 115200      # 毫米波波特率


def AEB():
    Uart = MMWR.openMMWPort(MMWR_PORT, MMWR_BAUD_RATE)  # 打开串口

    while True:
        distance, speed = MMWR.MMWDetection(Uart)
        if 0 < distance <= DISTANCE_THRESHOLD:
            print("start stop")
            motor.get_velocity(SetDutyCycle(0))
            exit(0)
        else:
            motor.get_velocity(SetDutyCycle(MOTOR_DUTY_CYCLE))


if __name__ == "__main__":
    AEB()
