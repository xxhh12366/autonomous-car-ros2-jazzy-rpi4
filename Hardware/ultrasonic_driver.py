#                                                                #Python_Nano_Ultrasonic_202302V1.py
# #!/usr/bin/env python3
# # coding:utf-8
# #import Jetson.GPIO as GPIO          
# import RPi.GPIO as GPIO     #换成树莓派的GPIO库   
# import time

# ultrasonicTrigPin = 19    # 超声波触发引脚——tx——指的是超声波板上的tx
# ultrasonicEchoPin = 26    # 超声波信号回响引脚——rx

# GPIO.setmode(GPIO.BOARD)                            # 使用物理编码，及开发板上正面的引脚编号
# GPIO.setup(ultrasonicTrigPin, GPIO.OUT)             # 设置触发引脚为输出模式
# GPIO.setup(ultrasonicEchoPin, GPIO.IN)              # 设置信号引脚为输出模式
# EchoRisingEdgeTime, EchoFallingEdgeTime = 0.0, 0.0  # 上升沿和下降沿的触发时间


# def triggerPulse():
#     """
#     输出10us的脉冲

#     :return:
#     """

#     GPIO.output(ultrasonicTrigPin, GPIO.HIGH)
#     time.sleep(0.0001)
#     GPIO.output(ultrasonicTrigPin, GPIO.LOW)


# def ultrasonicDetection() -> float:
#     """
#     超声波测距函数，需放置在循环中执行

#     :return: float, 距离，单位cm

#         0 -> 超出检测范围
#     """
#     global EchoRisingEdgeTime, EchoFallingEdgeTime
    
#     triggerPulse()  # 输出10us的触发脉冲做为超声波的控制信号
#     timeOutCount = 1000
#     while GPIO.input(ultrasonicEchoPin) == GPIO.LOW and timeOutCount > 0:
#         timeOutCount -= 1
#     EchoRisingEdgeTime = time.time()
#     # print(EchoRisingEdgeTime, timeOutCount)

#     timeOutCount = 1000
#     while GPIO.input(ultrasonicEchoPin) == GPIO.HIGH and timeOutCount > 0:
#         timeOutCount -= 1
#     EchoFallingEdgeTime = time.time()
#     # print(EchoFallingEdgeTime, timeOutCount)
#     # 计算距离
#     distance = (EchoFallingEdgeTime - EchoRisingEdgeTime) * 340 / 2 * 100  # 单位cm，
    
#     EchoRisingEdgeTime, EchoFallingEdgeTime = 0, 0

#     return distance


# if __name__ == "__main__":
#     while True:
#         Distance = ultrasonicDetection()
#         print("distance: ", Distance)
#         time.sleep(0.05)                                    

#!/usr/bin/env python3
# coding:utf-8
import RPi.GPIO as GPIO
import time

# 引脚定义（右边）（物理引脚编号）
ultrasonicTrigPin = 35  #左边38
ultrasonicEchoPin = 37  #36

def init_ultrasonic():
    """初始化GPIO设置"""
    GPIO.setmode(GPIO.BOARD)      # 使用物理引脚编号
    GPIO.setwarnings(False)       # 屏蔽GPIO警告
    GPIO.setup(ultrasonicTrigPin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ultrasonicEchoPin, GPIO.IN)
    time.sleep(0.5)               # 等待传感器稳定

def get_distance() -> float:
    """
    超声波测距核心函数
    返回：距离(cm)，如果超时则返回 -1.0
    """
    # 1. 发送 10us 的触发脉冲
    GPIO.output(ultrasonicTrigPin, GPIO.HIGH)
    time.sleep(0.00001) # 10微秒
    GPIO.output(ultrasonicTrigPin, GPIO.LOW)

    # 2. 记录 Echo 引脚变高（上升沿）的时刻
    t_start = time.time()
    t_timeout = t_start + 0.04 # 40ms 超时（对应约 7 米）
    
    while GPIO.input(ultrasonicEchoPin) == GPIO.LOW:
        t_start = time.time()
        if t_start > t_timeout:
            return -1.0 # 超时返回

    # 3. 记录 Echo 引脚变低（下降沿）的时刻
    t_stop = time.time()
    while GPIO.input(ultrasonicEchoPin) == GPIO.HIGH:
        t_stop = time.time()
        if t_stop > t_timeout:
            return -1.0

    # 4. 计算距离
    # 声速 343m/s，往返距离所以除以 2
    duration = t_stop - t_start
    distance = (duration * 34300) / 2

    # 5. 过滤掉异常数据（HC-SR04 最小量程约 2cm，最大约 400cm）
    if 2 <= distance <= 400:
        return distance
    else:
        return -1.0

if __name__ == "__main__":
    try:
        init_ultrasonic()
        print("超声波模块已启动，开始测距... (Ctrl+C 停止)")
        
        while True:
            dist = get_distance()
            if dist != -1.0:
                print(f"当前距离: {dist:.2f} cm")
            else:
                print("警告: 超出量程或传感器未响应")
                
            # 注意：超声波发射需要间隔，频率太快会产生余震干扰测距
            time.sleep(0.5) 
            
    except KeyboardInterrupt:
        print("\n程序停止中...")
        GPIO.cleanup()