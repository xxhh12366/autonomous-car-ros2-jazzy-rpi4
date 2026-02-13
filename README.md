# 自动驾驶小车 - ROS2 Jazzy (树莓派4)

## 项目简介

这是一个基于 ROS2 Jazzy 和树莓派4的自动驾驶小车项目。该项目集成了多种传感器和智能驾驶辅助功能，实现了自主导航、车道保持、自动泊车等功能。

## 主要功能模块

### 1. 车道保持系统 (LKS - Lane Keeping System)
- **路径**: `function/LKS_Hough/`
- **功能**: 使用霍夫变换进行车道线检测，实时调整舵机角度保持车道居中
- **技术**: OpenCV图像处理、Canny边缘检测、霍夫变换

### 2. 自适应巡航控制 (ACC - Adaptive Cruise Control)
- **路径**: `function/ACC/`
- **功能**: 根据前方车辆距离自动调节车速，保持安全距离

### 3. 自动紧急制动 (AEB - Automatic Emergency Braking)
- **路径**: `function/AEB/`
- **功能**: 当检测到前方障碍物距离小于阈值时自动紧急刹车
- **传感器**: 毫米波雷达 (MMWR)

### 4. 自动泊车系统 (APS - Automatic Parking System)
- **路径**: `function/APS/`
- **功能**: 
  - 自动检测停车位（垂直/水平）
  - 规划泊车轨迹
  - 自动完成泊车动作
- **传感器**: 超声波传感器、摄像头

### 5. 碰撞避免系统 (PCS - Pre-Collision System)
- **路径**: `function/PCS/`
- **功能**: 预测碰撞风险并采取规避措施

### 6. 斑马线识别
- **路径**: `function/Zebra/`
- **功能**: 识别斑马线并自动停车

### 7. 车牌识别
- **路径**: `function/License_Plate/`
- **功能**: 识别和读取车牌号码

### 8. 交通信号灯识别
- **路径**: `function/traffic_lights/`
- **功能**: 识别交通信号灯状态并作出相应响应

### 9. 定位系统
- **路径**: `function/location/`
- **功能**: 使用激光雷达(LIDAR)和GPS进行精确定位

## 硬件驱动

所有硬件驱动位于 `Hardware/` 目录下：

- **camera_driver.py**: 摄像头驱动
- **gps_driver.py**: GPS模块驱动
- **imu_driver.py**: 惯性测量单元(IMU)驱动
- **millimeterwave_driver.py**: 毫米波雷达驱动
- **motor_driver.py**: 电机驱动 (基于VESC)
- **servo_driver.py**: 舵机驱动
- **ultrasonic_driver.py**: 超声波传感器驱动
- **tcp_comm_server.py / tcp_comm_client.py**: TCP网络通信
- **udp_comm_server.py / udp_comm_client.py**: UDP网络通信

## 控制系统

### PID控制器
- **路径**: `control/pid_controller.py`
- **功能**: 实现电机转速的PID闭环控制
- **参数**: 
  - Kp = 0.8 (比例系数)
  - Ki = 1.0 (积分系数)
  - Kd = 0.01 (微分系数)
  - 目标转速: 2000 RPM

## 传感器配置

- **激光雷达**: RPLidar A1/A2/A3/S2/S3 系列
- **摄像头**: USB摄像头 (320x240)
- **毫米波雷达**: 串口通信，波特率115200
- **超声波传感器**: 左右双传感器配置
- **IMU**: 惯性测量单元
- **GPS**: 定位模块

## 系统要求

- **硬件**: 树莓派 4
- **操作系统**: Ubuntu 24.04 (或兼容系统)
- **ROS版本**: ROS2 Jazzy
- **Python版本**: Python 3.x

## 依赖库

```bash
# ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Python依赖
pip install pyvesc
pip install simple-pid
pip install opencv-python
pip install numpy
pip install pyserial
```

## 安装和使用

### 1. 克隆仓库

```bash
cd ~/xt_ws/src/
git clone https://github.com/xxhh12366/autonomous-car-ros2-jazzy-rpi4.git
```

### 2. 设置环境

```bash
cd autonomous-car-ros2-jazzy-rpi4
python3 -m venv .venv_car
source .venv_car/bin/activate
```

### 3. 安装依赖

```bash
pip install -r requirements.txt  # 如果存在
```

### 4. 启动激光雷达

```bash
chmod +x start_lidar.sh
./start_lidar.sh
```

### 5. 运行PID控制器

```bash
python control/pid_controller.py
```

### 6. 运行特定功能

```bash
# 车道保持系统
python function/LKS_Hough/Python_Nano_LKS_202302V1.py

# 自动泊车系统
python function/APS/Python_Nano_APS_202304v2.py

# 自动紧急制动
python function/AEB/Python_Nano_AEB_202302V2.py
```

## 项目结构

```
autonomous-car-ros2-jazzy-rpi4/
├── control/                    # 控制算法
│   └── pid_controller.py      # PID控制器
├── function/                   # 功能模块
│   ├── ACC/                   # 自适应巡航
│   ├── AEB/                   # 自动紧急制动
│   ├── APS/                   # 自动泊车
│   ├── LKS_Hough/            # 车道保持
│   ├── License_Plate/        # 车牌识别
│   ├── PCS/                  # 碰撞避免
│   ├── Zebra/                # 斑马线识别
│   ├── location/             # 定位系统
│   └── traffic_lights/       # 交通灯识别
├── Hardware/                  # 硬件驱动
│   ├── camera_driver.py
│   ├── gps_driver.py
│   ├── imu_driver.py
│   ├── motor_driver.py
│   ├── servo_driver.py
│   └── ...
├── rplidar_ros/              # 激光雷达ROS包
├── main.py                   # 主程序入口
└── start_lidar.sh           # 激光雷达启动脚本
```

## 串口配置

- **电机控制 (VESC)**: `/dev/ttyS0` (波特率: 115200)
- **毫米波雷达**: `/dev/ttyTHS1` (波特率: 115200)
- **激光雷达**: `/dev/ttyUSB1`

## 注意事项

1. 运行前确保所有串口权限正确设置：
   ```bash
   sudo chmod 666 /dev/ttyS0
   sudo chmod 666 /dev/ttyTHS1
   sudo chmod 666 /dev/ttyUSB1
   ```

2. 确保摄像头已正确连接并可访问

3. 各功能模块独立运行，根据需要选择启动

4. 建议在安全环境下测试各项功能

## 开发团队

- **作者**: xxhh12366
- **邮箱**: BPJY@outlook.com
- **技术支持**: Relaxing Technology Chongqing Co.,Ltd.

## 许可证

Copyright (C) 2023 Relaxing Technology Chongqing Co.,Ltd. All rights reserved.

## 更新日志

- 2023/05: 自动泊车系统优化
- 2023/04: 新增APS功能
- 2023/03: 车道保持系统实现
- 2023/02: 基础框架搭建
