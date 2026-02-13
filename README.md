# Autonomous Car - ROS2 Jazzy on Raspberry Pi 4

[English](#english) | [中文](#chinese)

---

<a name="english"></a>
## English

### Project Overview

This project implements an autonomous car system using ROS2 Jazzy on Raspberry Pi 4. The system integrates multiple sensors and implements various Advanced Driver Assistance Systems (ADAS) features, including adaptive cruise control, automatic emergency braking, lane keeping, automated parking, and more.

### Features

#### ADAS Functions
- **ACC (Adaptive Cruise Control)**: Distance-speed closed-loop control system using millimeter-wave radar
- **AEB (Automatic Emergency Braking)**: Collision avoidance system with automatic braking
- **APS (Automatic Parking System)**: Automated parallel and perpendicular parking
- **LKS (Lane Keeping System)**: Lane detection and keeping using Hough transform
- **PCS (Pedestrian Collision System)**: Pedestrian detection and collision avoidance
- **Zebra Crossing Detection**: Detection and automatic braking at zebra crossings
- **License Plate Recognition**: Vehicle license plate detection and recognition

### Hardware Components

#### Sensors
- **Camera**: USB camera for vision-based features (OpenCV + cv_bridge)
- **LiDAR**: RPLiDAR A1 for 2D laser scanning and mapping
- **Millimeter-Wave Radar**: Distance and speed detection for ACC/AEB
- **Ultrasonic Sensors**: Multiple sensors for parking assistance and obstacle detection
- **IMU (Inertial Measurement Unit)**: 9-axis sensor for orientation and motion tracking
- **GPS**: Location tracking and navigation

#### Actuators
- **Motor**: VESC-based BLDC motor controller
  - Serial communication via `/dev/ttyUSB2`
  - RPM and duty cycle control
  - PID-based speed control
- **Servo**: SCServo SDK-based steering control
  - Position and speed control
  - Real-time feedback

#### Communication
- **TCP/UDP**: Network communication for distributed system components
- **Serial**: Hardware device communication (motor, sensors)

### System Architecture

```
autonomous-car-ros2-jazzy-rpi4/
├── Hardware/              # Hardware drivers
│   ├── camera_driver.py        # ROS2 camera publisher
│   ├── motor_driver.py         # VESC motor controller
│   ├── servo_driver.py         # SCServo controller
│   ├── imu_driver.py          # IMU sensor driver
│   ├── gps_driver.py          # GPS sensor driver
│   ├── ultrasonic_driver.py   # Ultrasonic sensor array
│   ├── millimeterwave_driver.py # MMW radar driver
│   ├── pyvesc/            # PyVESC library for motor control
│   └── scservo_sdk/       # SCServo SDK for servo control
├── control/               # Control algorithms
│   └── pid_controller.py       # PID controller implementation
├── function/              # ADAS feature implementations
│   ├── ACC/               # Adaptive cruise control
│   ├── AEB/               # Automatic emergency braking
│   ├── APS/               # Automatic parking system
│   ├── LKS_Hough/         # Lane keeping (Hough transform)
│   ├── PCS/               # Pedestrian collision system
│   ├── Zebra/             # Zebra crossing detection
│   ├── License_Plate/     # License plate recognition
│   ├── location/          # Localization and mapping
│   │   ├── base_controller/   # ROS2 base controller
│   │   └── rplidar_ros/       # RPLiDAR ROS2 package
│   ├── traffic_lights/    # Traffic light detection (YOLOv5)
│   └── ultralytics-main/  # YOLOv8 for object detection
├── rplidar_ros/           # RPLiDAR ROS2 package
├── main.py                # Main entry point
└── start_lidar.sh         # LiDAR launch script
```

### Requirements

#### Hardware
- Raspberry Pi 4 (4GB+ RAM recommended)
- USB Camera
- RPLiDAR A1
- VESC Motor Controller
- SCServo Motor
- Millimeter-Wave Radar
- Ultrasonic Sensors (4-8 sensors)
- IMU Sensor
- GPS Module

#### Software
- Ubuntu 22.04 or later (for ROS2 Jazzy)
- ROS2 Jazzy
- Python 3.10+
- OpenCV (cv2)
- cv_bridge (ROS2 package)
- PyVESC
- SCServo SDK
- simple-pid

### Installation

1. **Install ROS2 Jazzy**
   ```bash
   # Follow official ROS2 Jazzy installation guide
   # https://docs.ros.org/en/jazzy/Installation.html
   ```

2. **Clone the Repository**
   ```bash
   cd ~/
   git clone https://github.com/xxhh12366/autonomous-car-ros2-jazzy-rpi4.git
   cd autonomous-car-ros2-jazzy-rpi4
   ```

3. **Install Python Dependencies**
   ```bash
   pip install opencv-python numpy simple-pid pyserial
   ```

4. **Build ROS2 Packages**
   ```bash
   # Source ROS2
   source /opt/ros/jazzy/setup.bash
   
   # Build workspace
   colcon build
   source install/setup.bash
   ```

5. **Configure Serial Ports**
   ```bash
   # Grant permission to serial ports
   sudo chmod 666 /dev/ttyUSB2  # Motor controller
   sudo chmod 666 /dev/ttyUSB1  # LiDAR
   sudo chmod 666 /dev/ttyTHS1  # Millimeter-wave radar
   ```

### Usage

#### Launch LiDAR
```bash
./start_lidar.sh
# Or manually:
ros2 launch rplidar_ros rplidar.launch.py
```

#### Run ADAS Features

**Adaptive Cruise Control (ACC)**
```bash
cd function/ACC
python3 Python_Nano_ACC_202302V2.py
```

**Automatic Emergency Braking (AEB)**
```bash
cd function/AEB
python3 Python_Nano_AEB_202302V2.py
```

**Automatic Parking System (APS)**
```bash
cd function/APS
python3 Python_Nano_APS_202304v2.py
```

**Lane Keeping System (LKS)**
```bash
cd function/LKS_Hough
python3 Python_Nano_LKS_202302V1.py
```

**Pedestrian Collision System (PCS)**
```bash
cd function/PCS
python3 Python_Nano_PCS_202302v1.py
```

**Zebra Crossing Detection**
```bash
cd function/Zebra
python3 Python_Nano_ZebraBrake_202302V1.py
```

**License Plate Recognition**
```bash
cd function/License_Plate
python3 Python_Nano_License_plate_202307v1.py
```

#### Run Camera Node
```bash
# In ROS2 workspace
cd Hardware
python3 camera_driver.py
```

### Configuration

#### Motor Configuration
Edit `Hardware/motor_driver.py`:
```python
serialport = '/dev/ttyUSB2'  # Motor serial port
Target_rpm = 2000            # Target RPM
```

#### Servo Configuration
Edit servo parameters in function files:
```python
DEFAULT_POSITION = 1500      # Center position
MAX_ANGLE = 2000            # Maximum angle
MIN_ANGLE = 1000            # Minimum angle
```

#### ACC Parameters
Edit `function/ACC/Python_Nano_ACC_202302V2.py`:
```python
STOP_DISTANCE = 35          # Stop distance (cm)
SLOW_DOWN_DISTANCE = 60     # Slow down distance (cm)
SPEED_UP_DISTANCE = 80      # Speed up distance (cm)
DEFAULT_DUTY = 0.08         # Default motor duty cycle
```

#### APS Parameters
Edit `function/APS/Python_Nano_APS_202304v2.py`:
```python
PARKING_DIRECTION = "left"          # Parking side: "left" or "right"
ULTRASONIC_THRESHOLD = 45           # Detection threshold (cm)
PARKING_LENGTH_THRESHOLD = 55       # Parking space length threshold (cm)
```

### Technical Details

#### Control System
- **PID Control**: Motor speed control with PID algorithm
- **Sensor Fusion**: Multi-sensor data fusion for robust perception
- **Real-time Processing**: Optimized for real-time performance on Raspberry Pi 4

#### Communication Protocols
- **ROS2 Topics**: Publisher-subscriber pattern for sensor data
- **Serial Communication**: UART for motor/sensor control
- **Network Communication**: TCP/UDP for distributed components

#### Vision Processing
- **OpenCV**: Image processing and computer vision algorithms
- **Hough Transform**: Lane detection
- **YOLO**: Object detection for pedestrians, vehicles, and traffic lights
- **cv_bridge**: ROS-OpenCV image conversion

### Troubleshooting

#### Serial Port Issues
```bash
# Check available ports
ls -l /dev/tty*

# Grant permissions (example for motor controller on /dev/ttyUSB2)
sudo chmod 666 /dev/ttyUSB2
```

#### Camera Not Detected
```bash
# Check camera
ls /dev/video*

# Test camera
python3 -c "import cv2; cap = cv2.VideoCapture(0); print(cap.isOpened())"
```

#### ROS2 Build Errors
```bash
# Clean build
rm -rf build install log
colcon build --symlink-install
```

### Contributing

Contributions are welcome! Please follow these guidelines:
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

### License

This project is developed for educational and research purposes.

### Contact

- Author: xxhh12366
- Email: BPJY@outlook.com
- Repository: https://github.com/xxhh12366/autonomous-car-ros2-jazzy-rpi4

---

<a name="chinese"></a>
## 中文

### 项目概述

本项目在树莓派4上使用ROS2 Jazzy实现了一个自动驾驶小车系统。系统集成了多种传感器，实现了多种高级驾驶辅助系统（ADAS）功能，包括自适应巡航控制、自动紧急制动、车道保持、自动泊车等。

### 功能特性

#### ADAS功能
- **ACC（自适应巡航控制）**：使用毫米波雷达的距离-速度闭环控制系统
- **AEB（自动紧急制动）**：具有自动制动功能的碰撞避免系统
- **APS（自动泊车系统）**：自动平行和垂直泊车
- **LKS（车道保持系统）**：使用霍夫变换进行车道检测和保持
- **PCS（行人碰撞系统）**：行人检测和碰撞避免
- **斑马线检测**：斑马线检测和自动制动
- **车牌识别**：车辆车牌检测和识别

### 硬件组件

#### 传感器
- **摄像头**：USB摄像头用于视觉功能（OpenCV + cv_bridge）
- **激光雷达**：RPLiDAR A1用于2D激光扫描和建图
- **毫米波雷达**：用于ACC/AEB的距离和速度检测
- **超声波传感器**：多个传感器用于泊车辅助和障碍物检测
- **IMU（惯性测量单元）**：9轴传感器用于方向和运动跟踪
- **GPS**：位置跟踪和导航

#### 执行器
- **电机**：基于VESC的无刷直流电机控制器
  - 通过`/dev/ttyUSB2`串口通信
  - RPM和占空比控制
  - 基于PID的速度控制
- **舵机**：基于SCServo SDK的转向控制
  - 位置和速度控制
  - 实时反馈

#### 通信
- **TCP/UDP**：分布式系统组件的网络通信
- **串口**：硬件设备通信（电机、传感器）

### 系统架构

```
autonomous-car-ros2-jazzy-rpi4/
├── Hardware/              # 硬件驱动
│   ├── camera_driver.py        # ROS2摄像头发布器
│   ├── motor_driver.py         # VESC电机控制器
│   ├── servo_driver.py         # SCServo控制器
│   ├── imu_driver.py          # IMU传感器驱动
│   ├── gps_driver.py          # GPS传感器驱动
│   ├── ultrasonic_driver.py   # 超声波传感器阵列
│   ├── millimeterwave_driver.py # 毫米波雷达驱动
│   ├── pyvesc/            # 电机控制的PyVESC库
│   └── scservo_sdk/       # 舵机控制的SCServo SDK
├── control/               # 控制算法
│   └── pid_controller.py       # PID控制器实现
├── function/              # ADAS功能实现
│   ├── ACC/               # 自适应巡航控制
│   ├── AEB/               # 自动紧急制动
│   ├── APS/               # 自动泊车系统
│   ├── LKS_Hough/         # 车道保持（霍夫变换）
│   ├── PCS/               # 行人碰撞系统
│   ├── Zebra/             # 斑马线检测
│   ├── License_Plate/     # 车牌识别
│   ├── location/          # 定位和建图
│   │   ├── base_controller/   # ROS2基础控制器
│   │   └── rplidar_ros/       # RPLiDAR ROS2包
│   ├── traffic_lights/    # 交通灯检测（YOLOv5）
│   └── ultralytics-main/  # 物体检测的YOLOv8
├── rplidar_ros/           # RPLiDAR ROS2包
├── main.py                # 主入口点
└── start_lidar.sh         # 激光雷达启动脚本
```

### 系统要求

#### 硬件
- 树莓派4（推荐4GB+内存）
- USB摄像头
- RPLiDAR A1
- VESC电机控制器
- SCServo舵机
- 毫米波雷达
- 超声波传感器（4-8个传感器）
- IMU传感器
- GPS模块

#### 软件
- Ubuntu 22.04或更高版本（用于ROS2 Jazzy）
- ROS2 Jazzy
- Python 3.10+
- OpenCV (cv2)
- cv_bridge (ROS2包)
- PyVESC
- SCServo SDK
- simple-pid

### 安装步骤

1. **安装ROS2 Jazzy**
   ```bash
   # 遵循官方ROS2 Jazzy安装指南
   # https://docs.ros.org/en/jazzy/Installation.html
   ```

2. **克隆仓库**
   ```bash
   cd ~/
   git clone https://github.com/xxhh12366/autonomous-car-ros2-jazzy-rpi4.git
   cd autonomous-car-ros2-jazzy-rpi4
   ```

3. **安装Python依赖**
   ```bash
   pip install opencv-python numpy simple-pid pyserial
   ```

4. **编译ROS2包**
   ```bash
   # Source ROS2
   source /opt/ros/jazzy/setup.bash
   
   # 编译工作空间
   colcon build
   source install/setup.bash
   ```

5. **配置串口**
   ```bash
   # 授予串口权限
   sudo chmod 666 /dev/ttyUSB2  # 电机控制器
   sudo chmod 666 /dev/ttyUSB1  # 激光雷达
   sudo chmod 666 /dev/ttyTHS1  # 毫米波雷达
   ```

### 使用方法

#### 启动激光雷达
```bash
./start_lidar.sh
# 或手动启动：
ros2 launch rplidar_ros rplidar.launch.py
```

#### 运行ADAS功能

**自适应巡航控制（ACC）**
```bash
cd function/ACC
python3 Python_Nano_ACC_202302V2.py
```

**自动紧急制动（AEB）**
```bash
cd function/AEB
python3 Python_Nano_AEB_202302V2.py
```

**自动泊车系统（APS）**
```bash
cd function/APS
python3 Python_Nano_APS_202304v2.py
```

**车道保持系统（LKS）**
```bash
cd function/LKS_Hough
python3 Python_Nano_LKS_202302V1.py
```

**行人碰撞系统（PCS）**
```bash
cd function/PCS
python3 Python_Nano_PCS_202302v1.py
```

**斑马线检测**
```bash
cd function/Zebra
python3 Python_Nano_ZebraBrake_202302V1.py
```

**车牌识别**
```bash
cd function/License_Plate
python3 Python_Nano_License_plate_202307v1.py
```

#### 运行摄像头节点
```bash
# 在ROS2工作空间中
cd Hardware
python3 camera_driver.py
```

### 配置

#### 电机配置
编辑`Hardware/motor_driver.py`：
```python
serialport = '/dev/ttyUSB2'  # 电机串口
Target_rpm = 2000            # 目标RPM
```

#### 舵机配置
在功能文件中编辑舵机参数：
```python
DEFAULT_POSITION = 1500      # 中心位置
MAX_ANGLE = 2000            # 最大角度
MIN_ANGLE = 1000            # 最小角度
```

#### ACC参数
编辑`function/ACC/Python_Nano_ACC_202302V2.py`：
```python
STOP_DISTANCE = 35          # 停止距离（厘米）
SLOW_DOWN_DISTANCE = 60     # 减速距离（厘米）
SPEED_UP_DISTANCE = 80      # 加速距离（厘米）
DEFAULT_DUTY = 0.08         # 默认电机占空比
```

#### APS参数
编辑`function/APS/Python_Nano_APS_202304v2.py`：
```python
PARKING_DIRECTION = "left"          # 泊车方向："left"或"right"
ULTRASONIC_THRESHOLD = 45           # 检测阈值（厘米）
PARKING_LENGTH_THRESHOLD = 55       # 车位长度阈值（厘米）
```

### 技术细节

#### 控制系统
- **PID控制**：使用PID算法进行电机速度控制
- **传感器融合**：多传感器数据融合以实现可靠感知
- **实时处理**：针对树莓派4的实时性能优化

#### 通信协议
- **ROS2话题**：传感器数据的发布-订阅模式
- **串口通信**：用于电机/传感器控制的UART
- **网络通信**：分布式组件的TCP/UDP

#### 视觉处理
- **OpenCV**：图像处理和计算机视觉算法
- **霍夫变换**：车道检测
- **YOLO**：行人、车辆和交通灯的物体检测
- **cv_bridge**：ROS-OpenCV图像转换

### 故障排除

#### 串口问题
```bash
# 检查可用端口
ls -l /dev/tty*

# 授予权限（示例：电机控制器在 /dev/ttyUSB2）
sudo chmod 666 /dev/ttyUSB2
```

#### 摄像头未检测到
```bash
# 检查摄像头
ls /dev/video*

# 测试摄像头
python3 -c "import cv2; cap = cv2.VideoCapture(0); print(cap.isOpened())"
```

#### ROS2编译错误
```bash
# 清理编译
rm -rf build install log
colcon build --symlink-install
```

### 贡献

欢迎贡献！请遵循以下准则：
1. Fork仓库
2. 创建功能分支
3. 提交更改
4. 推送到分支
5. 创建Pull Request

### 许可证

本项目用于教育和研究目的。

### 联系方式

- 作者：xxhh12366
- 邮箱：BPJY@outlook.com
- 仓库：https://github.com/xxhh12366/autonomous-car-ros2-jazzy-rpi4
