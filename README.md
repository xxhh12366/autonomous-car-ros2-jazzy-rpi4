# Autonomous Car ROS2 Jazzy - Raspberry Pi 4

自动驾驶小车项目 - 基于 ROS2 Jazzy 和 Raspberry Pi 4

## 项目结构 / Project Structure

```
autonomous-car-ros2-jazzy-rpi4/
├── Hardware/                    # 硬件驱动模块（中心化）
│   ├── motor_driver.py         # 电机驱动（VESC + 3650 21.5T motor）
│   ├── servo_driver.py         # 舵机驱动（SCS20-360T）
│   ├── millimeterwave_driver.py # 毫米波雷达驱动（HLR12）
│   ├── camera_driver.py        # 摄像头驱动（ROS2）
│   ├── imu_driver.py           # IMU 传感器驱动
│   ├── gps_driver.py           # GPS 传感器驱动
│   ├── ultrasonic_driver.py    # 超声波传感器驱动
│   └── __init__.py             # Python 包初始化文件
│
├── control/                     # 控制算法模块
│   └── pid_controller.py       # PID 控制器
│
├── function/                    # 功能模块
│   ├── ACC/                    # 自适应巡航控制（Adaptive Cruise Control）
│   ├── AEB/                    # 自动紧急制动（Automatic Emergency Braking）
│   ├── APS/                    # 自动泊车系统（Automatic Parking System）
│   ├── LKS_Hough/             # 车道保持系统 - Hough变换
│   ├── License_Plate/         # 车牌识别
│   ├── PCS/                   # 前向碰撞预警系统
│   ├── Zebra/                 # 斑马线检测与制动
│   ├── traffic_lights/        # 交通信号灯识别
│   ├── location/              # 定位与导航
│   └── ultralytics-main/      # YOLO 目标检测
│
├── rplidar_ros/                # RPLidar 激光雷达 ROS2 包
├── main.py                     # 主程序入口
└── start_lidar.sh             # 启动激光雷达脚本
```

## 硬件驱动说明 / Hardware Driver Documentation

### 电机驱动 (Motor Driver)
- **文件**: `Hardware/motor_driver.py`
- **硬件**: VESC 电调 + 3650 21.5T 无刷电机
- **接口**: `/dev/motor` (串口)
- **功能**: 电机速度控制、转速读取、速度计算

### 舵机驱动 (Servo Driver)
- **文件**: `Hardware/servo_driver.py`
- **硬件**: SCS20-360T 舵机
- **接口**: `/dev/servo` (串口)
- **功能**: 舵机角度控制、位置读取、保护限位

### 毫米波雷达驱动 (Millimeter Wave Radar Driver)
- **文件**: `Hardware/millimeterwave_driver.py`
- **硬件**: HLR12 毫米波雷达
- **接口**: `/dev/ttyTHS1` (UART)
- **功能**: 距离检测、速度测量、数据校验

## 使用方法 / Usage

### 导入硬件驱动
```python
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from Hardware import motor_driver as Motor
from Hardware import servo_driver as Servo
from Hardware import millimeterwave_driver as MMWR
```

### 使用电机驱动
```python
from pyvesc.VESC.messages import SetDutyCycle

# 设置电机占空比
duty_cycle = 0.07
speed = Motor.get_values_example(SetDutyCycle(duty_cycle))
print(f"Current speed: {speed} cm/s")
```

### 使用舵机驱动
```python
# 设置舵机角度
target_position = 1500  # 中间位置
Servo.servo_angle_write(target_position)

# 读取舵机当前位置
current_position = Servo.servo_angle_read()
```

### 使用毫米波雷达
```python
# 打开串口
uart = MMWR.openMMWPort("/dev/ttyTHS1", 115200)

# 检测距离和速度
distance, speed = MMWR.MMWDetection(uart)
print(f"Distance: {distance} cm, Speed: {speed} cm/s")
```

## 功能模块说明 / Function Modules

- **ACC** (自适应巡航控制): 保持与前车安全距离，自动调节车速
- **AEB** (自动紧急制动): 检测前方障碍物，紧急情况自动刹车
- **APS** (自动泊车系统): 自动识别停车位并完成泊车
- **LKS** (车道保持系统): 通过视觉识别车道线，保持车辆在车道中央
- **PCS** (前向碰撞预警): 预测碰撞风险并提前警告
- **Zebra** (斑马线检测): 识别斑马线并执行减速停车

## 代码组织改进 / Code Organization Improvements

### 之前的问题
- 硬件驱动代码重复，分散在多个功能模块中
- 不同版本的驱动代码不一致，难以维护
- 功能模块耦合度高，不利于复用

### 现在的优势
- ✅ 硬件驱动统一管理在 `Hardware/` 目录
- ✅ 所有功能模块通过导入使用共享驱动
- ✅ 代码复用性高，易于维护和更新
- ✅ 清晰的模块化结构

## 开发说明 / Development Notes

1. 所有硬件驱动应放在 `Hardware/` 目录
2. 功能模块通过导入方式使用硬件驱动
3. 不要在功能模块中复制驱动代码
4. 保持代码结构清晰和模块化

## 作者 / Authors
Relaxing Technology Chongqing Co.,Ltd.
- Website: http://www.relaxingtechnology.com/
- Email: relaxingtech@qq.com

## 版权 / Copyright
Copyright (C) 2023 Relaxing Technology Chongqing Co.,Ltd. All rights reserved.
