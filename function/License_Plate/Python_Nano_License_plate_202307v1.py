#!/usr/bin/env python3
#coding=UTF-8
# ------------------------------------------------------------------ #
# @file    : Python_Nano_License_plate_202307v1.py
# @platform: Python
# @board   : Jetson Nano
# @brief   : License Plate
#
# @author  : Kuangxy
# @date    : 2023/7
# @web     : http ://www.relaxingtechnology.com/
# @e-mail  : relaxingtech@qq.com
# Copyright (C) 2023 Relaxing Technology Chongqing Co.,Ltd. All rights reserved.
# ------------------------------------------------------------------ #
import sys
import os
# Add Hardware directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

import cv2
import hyperlpr
import numpy as np
import time
from PIL import ImageFont, ImageDraw, Image
from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
from Hardware import motor_driver as Motor

plate_length_threshold=80    # 车牌长度阈值
MOTOR_DUTY_CYCLE = 0.06     # 电机占空比

cap = cv2.VideoCapture(0)

# 设置图像分辨率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)


# 加载SimSun字体
font_path = '/home/rtech/Desktop/License_Plate/simsun.ttc'  # 替换为SimSun字体文件的实际路径
font = ImageFont.truetype(font_path, size=30)


while True:
    # 读取摄像头的帧
    ret, frame = cap.read()

    # 使用hyperlpr进行车牌识别
    results = hyperlpr.HyperLPR_plate_recognition(frame)

    # 将OpenCV图像转换为PIL图像
    pil_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_image)

    Motor.get_values_example(SetDutyCycle(MOTOR_DUTY_CYCLE))

    # 遍历识别结果
    for result in results:
        # 提取车牌号码
        plate_number = result[0]

        # 在图像上绘制车牌号码
        draw.text((10, 50), plate_number, font=font, fill=(0, 255, 0))

        # x, y, w, h = cv2.boundingRect(result[2])
        # plate_length = w
        plate_length = result[2][2]-result[2][0]
        print(results)
        # print(result[2][0])
        # print(result[2][2])
        if plate_length > plate_length_threshold:
            print("start stop")
            Motor.get_values_example(SetDutyCycle(0))

        else:
            Motor.get_values_example(SetDutyCycle(MOTOR_DUTY_CYCLE))

        print("车牌号码:", plate_number)

    # 将PIL图像转换回OpenCV图像
    frame = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)

    # 显示带有车牌号码的图像
    cv2.imshow("车牌识别", frame)

    time.sleep(0.1)

    # 检查是否按下 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头并关闭所有窗口
cap.release()
cv2.destroyAllWindows()
