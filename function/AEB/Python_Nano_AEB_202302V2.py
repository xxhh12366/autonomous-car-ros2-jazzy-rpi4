from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
import Python_Nano_Motor_202302V2 as Motor
import Python_Nano_MMWR_202302V2 as MMWR


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
            Motor.get_values_example(SetDutyCycle(0))
            exit(0)
        else:
            Motor.get_values_example(SetDutyCycle(MOTOR_DUTY_CYCLE))


if __name__ == "__main__":
    AEB()
