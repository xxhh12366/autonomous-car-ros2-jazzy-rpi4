#要执行ACC功能的话，这个是main文件
"""
这是一个距离-速度闭环控制系统：
感知：用毫米波雷达测量前方障碍物距离
决策：根据距离决定加速、减速、保持或停止
控制：将速度指令转换为电机占空比，发送给VESC电机控制器
"""


import Python_Nano_Motor_202302V2 as Motor
import Python_Nano_MMWR_202302V3 as MMW
from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM

#这里改port
MMW_PORT = "/dev/ttyTHS1"
MMW_BAUD_RATE = 115200
TRANS_RATIO = 6.287  # 传动比为6.287
WHEEL_RADIUS = 0.032  # 半径为0.032米
STOP_DISTANCE = 35  # 停止距离
SLOW_DOWN_DISTANCE = 60  # 减速距离
SPEED_UP_DISTANCE = 80  # 加速距离
TO_FOLLOW_TIME = 8  # 变化至跟随模式的时间

DEFAULT_DUTY = 0.08  # 默认占空比
MAX_DUTY = 0.1  # 最大占空比

duty = DEFAULT_DUTY


def speed2rpm(speed): return int(speed * 0.01 * TRANS_RATIO * 60 / (2 * 3.14 * WHEEL_RADIUS))


def rpm2duty(rpm): return (abs(rpm) - 100) * 0.11 / 1200 + 0.01 if rpm > 0 else -((abs(rpm) - 100) * 0.11 / 1200 + 0.01)


"""
距离(cm)    控制策略
┌─────────────────────────────────────┐
│ 0-20      STOP_DISTANCE        停止  │
│ 20-50     SLOW_DOWN_DISTANCE   减速  │
│ 50-200    SPEED_UP_DISTANCE    加速  │
│ 其他      DEFAULT_DUTY         跟随  │
└─────────────────────────────────────┘
"""

def ACCMain():
    global duty
    port = MMW.openMMWPort(MMW_PORT, MMW_BAUD_RATE)  # 打开串口

    while True:
        distance, _ = MMW.MMWDetection(port)

        if 0 < distance < 200:
            if distance < STOP_DISTANCE:  # 停止
                duty = 0

            elif distance < SLOW_DOWN_DISTANCE:  # 减速
                correctionSpeed = (distance - SLOW_DOWN_DISTANCE) / TO_FOLLOW_TIME
                duty = duty + rpm2duty(speed2rpm(correctionSpeed))

            elif distance > SPEED_UP_DISTANCE:  # 加速
                correctionSpeed = (distance - SPEED_UP_DISTANCE) / TO_FOLLOW_TIME
                duty = duty + rpm2duty(speed2rpm(correctionSpeed))

            else:  # 跟随
                duty = DEFAULT_DUTY

        else:
            duty = DEFAULT_DUTY

        # 设定好duty的上下限
        duty = 0 if duty < 0 else duty
        duty = MAX_DUTY if duty > MAX_DUTY else duty
        Motor.get_values_example(SetDutyCycle(duty))    #目前这个的作用就只是发送控制命令给电机
        print("distance: ", distance, "\tduty = %f" % duty, "\t")


if __name__ == "__main__":
    ACCMain()
