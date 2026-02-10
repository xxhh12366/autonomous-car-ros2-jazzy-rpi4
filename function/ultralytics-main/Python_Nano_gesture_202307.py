#!/usr/bin/env python3
#coding=UTF-8
# ------------------------------------------------------------------ #
# @file    : Python_Nano_gesture_202307.py
# @platform: Python
# @board   : Jetson Nano
# @brief   : Gesture
#
# @author  : Wangfz
# @date    : 2023/7
# @web     : http ://www.relaxingtechnology.com/
# @e-mail  : relaxingtech@qq.com
# Copyright (C) 2023 Relaxing Technology Chongqing Co.,Ltd. All rights reserved.
# ------------------------------------------------------------------ #
import sys
import os
# Add Hardware directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from ultralytics import YOLO
import cv2
from Hardware import servo_driver as Servo
from Hardware import motor_driver as Motor
from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM

model = YOLO("best.pt")


TURN_LEFT_ANGLE_POSITION = 1000     # 左转时舵机控制输入
TURN_RIGHT_ANGLE_POSITION = 2200    # 右转时舵机控制输入
STRAIGHT_ANGLE_POSITION = 1500      # 直行时舵机控制输入
MOTOR_DUTY_CYCLE = 0.07             # 执行LKS时电机占空比
TURN_DUTY_CYCLE = 0.07              # 转弯时电机占空比
SLOWDOWN_DUTY_CYCLE = 0.05          # 减速时电机占空比
ACCELERATE_DUTY_CYCLE = 0.09        # 加速时电机占空比
STOP_DUTY_CYCLE = 0                 # 停止时电机占空比



# Display preds. Accepts all YOLO predict arguments
video_path = 0
cap = cv2.VideoCapture(video_path)



def run(motorControl, serveControl):
    Motor.get_values_example(SetDutyCycle(motorControl))
    Servo.servo_angle_write(serveControl)

while cap.isOpened():
    success,frame = cap.read()

    if success:
        
        results = model(source=frame)
        # annotated_fram = results[0].plot()
        # cv2.imshow("yolov8",annotated_fram)

        for result in results:
            boxes = result.boxes.cpu()
            for i, box in enumerate(boxes):
                if (box.conf[0] > 0.5):
                    cls = int(box.cls[0])

                    # print(cls)

                    if cls == 0:
                          run(TURN_DUTY_CYCLE, TURN_RIGHT_ANGLE_POSITION)
                    elif cls == 1:
                          run(TURN_DUTY_CYCLE, TURN_LEFT_ANGLE_POSITION)
                    elif cls == 2:
                          run(ACCELERATE_DUTY_CYCLE, STRAIGHT_ANGLE_POSITION)
                    elif cls == 3:
                          run(SLOWDOWN_DUTY_CYCLE, STRAIGHT_ANGLE_POSITION)
                    elif cls == 4:
                          run(STOP_DUTY_CYCLE, STRAIGHT_ANGLE_POSITION)                

            

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        

cap.release()
cv2.destroyAllWindows()
