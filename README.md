# Autonomous Car - ROS2 Jazzy on Raspberry Pi 4

[English](#english) | [中文](#chinese)

---

<a name="english"></a>
## English

### 🚗 Project Overview

This is a Level 2 autonomous vehicle platform built with **ROS2 Jazzy** running on **Raspberry Pi 4**. The project implements multiple Advanced Driver Assistance Systems (ADAS) features including adaptive cruise control, emergency braking, lane keeping, automated parking, and more.

### 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Application Layer                        │
│  ACC | AEB | LKS | PCS | APS | Zebra | License Plate | ...  │
├─────────────────────────────────────────────────────────────┤
│                      Control Layer                           │
│            PID Controller | Path Planning                    │
├─────────────────────────────────────────────────────────────┤
│                    Hardware Driver Layer                     │
│  Motor | Servo | Camera | IMU | GPS | Radar | Ultrasonic    │
├─────────────────────────────────────────────────────────────┤
│                   Communication Layer                        │
│         ROS2 Topics | Serial | TCP/UDP | VESC Protocol       │
└─────────────────────────────────────────────────────────────┘
```

### 🔧 Hardware Requirements

#### Core Components
- **Computing Platform**: Raspberry Pi 4 (4GB+ RAM recommended)
- **Operating System**: Ubuntu 22.04 with ROS2 Jazzy
- **Motor Controller**: VESC (Vehicle Electronic Speed Controller) for brushless motors
- **Servo Controller**: SCServo for steering control

#### Sensors
| Sensor | Model/Type | Interface | Purpose |
|--------|------------|-----------|---------|
| Camera | USB/CSI Camera | `/dev/video0` | Lane detection, object recognition |
| LIDAR | RPLidar A1/A2/A3/S1/S2/S3/C1/T1 | USB Serial | Environment mapping |
| IMU | 9-axis (Gyro+Accel+Mag) | Serial (921600 baud) | Orientation sensing |
| GPS | GNSS Receiver | Serial (9600 baud) | Position tracking |
| Millimeter-Wave Radar | Custom MMW Radar | `/dev/ttyTHS1` or `/dev/ttyUSB3` | Distance & speed detection |
| Ultrasonic | HC-SR04 | GPIO (35 trigger, 37 echo) | Short-range obstacle detection |

#### Actuators
- **Motor**: Brushless DC motor with VESC controller (serial `/dev/motor`, 115200 baud)
- **Servo**: SCServo (ID=1, serial `/dev/ttyUSB0`, 1000000 baud, position range 500-2500)

### 📦 Software Dependencies

#### ROS2 Packages
```bash
sudo apt update
sudo apt install ros-jazzy-desktop
sudo apt install python3-pip
```

#### Python Dependencies
```bash
pip3 install opencv-python
pip3 install cv-bridge
pip3 install numpy
pip3 install pyserial
pip3 install simple-pid
pip3 install ultralytics  # for YOLO models
```

#### Custom Libraries
- `pyvesc`: VESC motor controller communication (included in `Hardware/pyvesc/`)
- `scservo_sdk`: SCServo control library (included in `Hardware/scservo_sdk/`)

### 🎯 Features

#### 1. **ACC (Adaptive Cruise Control)** 📍 `function/Python_Nano_ACC_202302V2.py`
Automatically maintains safe distance from the vehicle ahead using millimeter-wave radar.

**Control Logic:**
- **Distance < 35cm**: Stop
- **35-60cm**: Slow down
- **60-80cm**: Follow mode
- **> 80cm**: Speed up to default speed

**Key Parameters:**
```python
STOP_DISTANCE = 35        # cm
SLOW_DOWN_DISTANCE = 60   # cm
SPEED_UP_DISTANCE = 80    # cm
DEFAULT_DUTY = 0.08       # Motor duty cycle
MAX_DUTY = 0.1
```

#### 2. **AEB (Automatic Emergency Braking)** 📍 `function/Python_Nano_AEB_202302V2.py`
Emergency braking system that stops the vehicle when an obstacle is detected within threshold distance.

**Parameters:**
```python
DISTANCE_THRESHOLD = 30   # cm, emergency brake distance
MOTOR_DUTY_CYCLE = 0.1    # Normal driving duty cycle
```

#### 3. **LKS (Lane Keeping System)** 📍 `function/LKS_Hough/`
Uses Hough transform for lane line detection and servo control to keep the vehicle centered in the lane.

**Processing Pipeline:**
1. Image preprocessing (Gaussian blur, edge detection)
2. ROI (Region of Interest) masking
3. Hough line detection
4. Lane line angle calculation
5. Servo angle adjustment

#### 4. **PCS (Parking Collision System)** 📍 `function/PCS/`
Integrated system combining millimeter-wave radar, ultrasonic sensors, and lane detection for parking assistance.

**Features:**
- Front/rear obstacle detection
- Safe distance monitoring
- Collision warning

#### 5. **APS (Automated Parking System)** 📍 `function/APS/`
Fully automated parking with two modes:

**Parking Modes:**
- **Level Parking** (平行停车): Parallel parking mode
- **Vertical Parking** (垂直停车): Perpendicular parking mode

**Configuration:**
```python
WHEEL_BASE = 25           # cm, vehicle wheelbase
PARKING_SPACE_WIDTH = 50  # cm
```

#### 6. **Zebra Crossing Detection** 📍 `function/Zebra/`
Detects zebra crossings using computer vision and controls vehicle to stop appropriately.

#### 7. **License Plate Recognition** 📍 `function/License_Plate/`
YOLO-based license plate detection and recognition system.

#### 8. **Traffic Light Detection** 📍 `function/traffic_lights/`
YOLO-based traffic light detection for autonomous navigation.

### 🚀 Quick Start

#### 1. Hardware Setup
```bash
# Connect hardware components:
# - Motor VESC to /dev/motor (or create symlink)
# - Servo to /dev/ttyUSB0
# - MMW Radar to /dev/ttyTHS1 or /dev/ttyUSB3
# - Camera to /dev/video0
# - IMU, GPS, and other sensors as per hardware guide
```

#### 2. Launch Camera Node (ROS2)
```bash
# Terminal 1: Camera publisher
cd /home/runner/work/autonomous-car-ros2-jazzy-rpi4/autonomous-car-ros2-jazzy-rpi4
python3 Hardware/camera_driver.py
```

#### 3. Launch LIDAR Node
```bash
# Terminal 2: LIDAR
cd /home/runner/work/autonomous-car-ros2-jazzy-rpi4/autonomous-car-ros2-jazzy-rpi4
./start_lidar.sh
```

#### 4. Run Autonomous Features

**Option A: Run ACC (Adaptive Cruise Control)**
```bash
cd function
python3 Python_Nano_ACC_202302V2.py
```

**Option B: Run AEB (Emergency Braking)**
```bash
cd function
python3 Python_Nano_AEB_202302V2.py
```

**Option C: Run Lane Keeping**
```bash
cd function/LKS_Hough
python3 Python_Nano_LKS_202302V1.py
```

**Option D: Run Automated Parking**
```bash
cd function/APS
python3 Python_Nano_APS_202304v2.py
```

### 📖 Hardware Driver API

#### Motor Control
```python
from Hardware import motor_driver as Motor
from pyvesc.VESC.messages import SetDutyCycle, SetRPM

# Set motor duty cycle (0.0 to 1.0)
Motor.get_values_example(SetDutyCycle(0.08))

# Set motor RPM directly
Motor.get_values_example(SetRPM(900))

# Get current speed
current_speed = Motor.get_values_example(SetDutyCycle(0.08))
# Returns speed in cm/s
```

#### Servo Control
```python
from Hardware import servo_driver

# Set servo position (500-2500)
servo_driver.servo_angle_write(1500)  # Center position

# Read current servo position
servo_driver.servo_angle_read()
```

#### Camera (ROS2 Node)
```python
# Subscribe to camera topic
ros2 topic echo /camera/image_raw
```

#### Ultrasonic Sensor
```python
from Hardware import ultrasonic_driver as Ultrasonic

# Get distance in cm
distance = Ultrasonic.get_distance()
print(f"Distance: {distance} cm")
```

#### IMU
```python
from Hardware import imu_driver as IMU

# Get IMU data
data = IMU.get_imu_data()
# Returns: acceleration, gyro, magnetic field, Euler angles
```

#### GPS
```python
from Hardware import gps_driver as GPS

# Get GPS coordinates
lat, lon, altitude, satellites = GPS.get_gps_data()
```

#### Millimeter-Wave Radar
```python
from Hardware import millimeterwave_driver as MMW

# Open radar port
port = MMW.openMMWPort("/dev/ttyTHS1", 115200)

# Get distance and speed
distance, speed = MMW.MMWDetection(port)
# distance in cm, speed in cm/s
```

### ⚙️ Configuration

#### Serial Port Configuration
Create symbolic links for consistent device naming:
```bash
# Motor VESC
sudo ln -s /dev/ttyACM0 /dev/motor

# Servo
sudo ln -s /dev/ttyUSB0 /dev/servo

# MMW Radar
sudo ln -s /dev/ttyTHS1 /dev/mmw_radar
```

#### Vehicle Parameters
Edit the constants in feature modules as needed:
- `TRANS_RATIO = 6.287` - Gear transmission ratio
- `WHEEL_RADIUS = 0.032` - Wheel radius in meters
- `WHEEL_BASE = 25` - Vehicle wheelbase in cm

### 🔍 PID Controller
The PID controller (`control/pid_controller.py`) is used for closed-loop motor speed control:

```python
from simple_pid import PID

pid = PID(0.8, 1, 0.01, setpoint=0)
pid.output_limits = (-3000, 3000)

# In control loop
rpm_error = current_rpm - target_rpm
rpm_correction = pid(rpm_error)
```

### 📂 Project Structure

```
autonomous-car-ros2-jazzy-rpi4/
├── Hardware/                    # Hardware driver layer
│   ├── camera_driver.py         # ROS2 camera publisher
│   ├── motor_driver.py          # VESC motor control
│   ├── servo_driver.py          # SCServo control
│   ├── imu_driver.py            # 9-axis IMU
│   ├── gps_driver.py            # GPS/GNSS
│   ├── ultrasonic_driver.py     # HC-SR04 ultrasonic
│   ├── millimeterwave_driver.py # MMW radar
│   ├── pyvesc/                  # VESC protocol library
│   └── scservo_sdk/             # SCServo SDK
├── control/                     # Control algorithms
│   └── pid_controller.py        # PID speed control
├── function/                    # Autonomous features
│   ├── Python_Nano_ACC_202302V2.py      # Adaptive Cruise Control
│   ├── Python_Nano_AEB_202302V2.py      # Emergency Braking
│   ├── LKS_Hough/               # Lane Keeping System
│   ├── APS/                     # Automated Parking System
│   ├── PCS/                     # Parking Collision System
│   ├── Zebra/                   # Zebra crossing detection
│   ├── License_Plate/           # License plate recognition
│   ├── traffic_lights/          # Traffic light detection
│   └── ultralytics-main/        # YOLOv8 models
├── rplidar_ros/                 # RPLidar ROS2 driver
├── start_lidar.sh               # LIDAR launch script
└── main.py                      # Main entry point (to be implemented)
```

### 🛠️ Troubleshooting

#### Serial Port Permission Issues
```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS*
# Logout and login again
```

#### Camera Not Found
```bash
# Check camera device
ls -l /dev/video*
v4l2-ctl --list-devices

# Test camera
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera Error')"
```

#### ROS2 Environment
```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 📝 Development Guidelines

#### Adding New Features
1. Create a new directory under `function/`
2. Follow the naming convention: `Python_Nano_<Feature>_<YYYYMM><Version>.py`
3. Import required hardware drivers from `Hardware/`
4. Add configuration constants at the top of the file
5. Document control logic with comments

#### Code Style
- Use Chinese comments for Chinese developers (current style)
- Add English docstrings for functions
- Follow PEP 8 Python style guide
- Use descriptive variable names

### 🤝 Contributing

Contributions are welcome! Please follow these steps:
1. Fork the repository
2. Create a feature branch
3. Add comprehensive documentation
4. Test on hardware if possible
5. Submit a pull request

### 📄 License

[Add license information]

### 👥 Authors

- xxhh12366 - Original author

### 🙏 Acknowledgments

- ROS2 Jazzy community
- RPLidar ROS2 driver
- Ultralytics YOLOv8
- VESC project
- SCServo SDK

---

<a name="chinese"></a>
## 中文

### 🚗 项目概述

这是一个基于**树莓派4**和**ROS2 Jazzy**构建的L2级自动驾驶车辆平台。该项目实现了多种高级驾驶辅助系统（ADAS）功能，包括自适应巡航控制、紧急制动、车道保持、自动泊车等。

### 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                      应用层                                  │
│  ACC | AEB | LKS | PCS | APS | 斑马线 | 车牌识别 | ...      │
├─────────────────────────────────────────────────────────────┤
│                      控制层                                  │
│            PID控制器 | 路径规划                              │
├─────────────────────────────────────────────────────────────┤
│                    硬件驱动层                                │
│  电机 | 舵机 | 摄像头 | IMU | GPS | 雷达 | 超声波           │
├─────────────────────────────────────────────────────────────┤
│                   通信层                                     │
│         ROS2话题 | 串口 | TCP/UDP | VESC协议                 │
└─────────────────────────────────────────────────────────────┘
```

### 🔧 硬件要求

#### 核心组件
- **计算平台**：树莓派4（建议4GB+内存）
- **操作系统**：Ubuntu 22.04 with ROS2 Jazzy
- **电机控制器**：VESC（车辆电子调速器）用于无刷电机
- **舵机控制器**：SCServo用于转向控制

#### 传感器
| 传感器 | 型号/类型 | 接口 | 用途 |
|--------|-----------|------|------|
| 摄像头 | USB/CSI相机 | `/dev/video0` | 车道检测、物体识别 |
| 激光雷达 | RPLidar A1/A2/A3/S1/S2/S3/C1/T1 | USB串口 | 环境建图 |
| IMU | 9轴（陀螺仪+加速度+磁力计） | 串口 (921600波特率) | 姿态感知 |
| GPS | GNSS接收器 | 串口 (9600波特率) | 位置追踪 |
| 毫米波雷达 | 定制毫米波雷达 | `/dev/ttyTHS1` 或 `/dev/ttyUSB3` | 距离和速度检测 |
| 超声波 | HC-SR04 | GPIO (35触发, 37回响) | 短距离障碍物检测 |

#### 执行器
- **电机**：无刷直流电机配VESC控制器（串口 `/dev/motor`，115200波特率）
- **舵机**：SCServo（ID=1，串口 `/dev/ttyUSB0`，1000000波特率，位置范围500-2500）

### 📦 软件依赖

#### ROS2包
```bash
sudo apt update
sudo apt install ros-jazzy-desktop
sudo apt install python3-pip
```

#### Python依赖
```bash
pip3 install opencv-python
pip3 install cv-bridge
pip3 install numpy
pip3 install pyserial
pip3 install simple-pid
pip3 install ultralytics  # YOLO模型
```

#### 自定义库
- `pyvesc`：VESC电机控制器通信（包含在 `Hardware/pyvesc/`）
- `scservo_sdk`：SCServo控制库（包含在 `Hardware/scservo_sdk/`）

### 🎯 功能特性

#### 1. **ACC（自适应巡航控制）** 📍 `function/Python_Nano_ACC_202302V2.py`
使用毫米波雷达自动保持与前车的安全距离。

**控制逻辑：**
- **距离 < 35cm**：停止
- **35-60cm**：减速
- **60-80cm**：跟随模式
- **> 80cm**：加速到默认速度

**关键参数：**
```python
STOP_DISTANCE = 35        # 厘米
SLOW_DOWN_DISTANCE = 60   # 厘米
SPEED_UP_DISTANCE = 80    # 厘米
DEFAULT_DUTY = 0.08       # 电机占空比
MAX_DUTY = 0.1
```

#### 2. **AEB（自动紧急制动）** 📍 `function/Python_Nano_AEB_202302V2.py`
当检测到阈值距离内有障碍物时，紧急制动系统会停止车辆。

**参数：**
```python
DISTANCE_THRESHOLD = 30   # 厘米，紧急制动距离
MOTOR_DUTY_CYCLE = 0.1    # 正常行驶占空比
```

#### 3. **LKS（车道保持系统）** 📍 `function/LKS_Hough/`
使用霍夫变换进行车道线检测，并通过舵机控制保持车辆在车道中央。

**处理流程：**
1. 图像预处理（高斯模糊、边缘检测）
2. ROI（感兴趣区域）掩码
3. 霍夫直线检测
4. 车道线角度计算
5. 舵机角度调整

#### 4. **PCS（泊车碰撞系统）** 📍 `function/PCS/`
结合毫米波雷达、超声波传感器和车道检测的集成系统，用于泊车辅助。

**功能：**
- 前后障碍物检测
- 安全距离监控
- 碰撞预警

#### 5. **APS（自动泊车系统）** 📍 `function/APS/`
具有两种模式的全自动泊车：

**泊车模式：**
- **平行停车**：平行泊车模式
- **垂直停车**：垂直泊车模式

**配置：**
```python
WHEEL_BASE = 25           # 厘米，车辆轴距
PARKING_SPACE_WIDTH = 50  # 厘米
```

#### 6. **斑马线检测** 📍 `function/Zebra/`
使用计算机视觉检测斑马线，并控制车辆适当停车。

#### 7. **车牌识别** 📍 `function/License_Plate/`
基于YOLO的车牌检测和识别系统。

#### 8. **交通信号灯检测** 📍 `function/traffic_lights/`
基于YOLO的交通信号灯检测，用于自主导航。

### 🚀 快速开始

#### 1. 硬件设置
```bash
# 连接硬件组件：
# - 电机VESC到 /dev/motor（或创建符号链接）
# - 舵机到 /dev/ttyUSB0
# - 毫米波雷达到 /dev/ttyTHS1 或 /dev/ttyUSB3
# - 摄像头到 /dev/video0
# - IMU、GPS和其他传感器根据硬件指南连接
```

#### 2. 启动摄像头节点（ROS2）
```bash
# 终端1：摄像头发布器
cd /home/runner/work/autonomous-car-ros2-jazzy-rpi4/autonomous-car-ros2-jazzy-rpi4
python3 Hardware/camera_driver.py
```

#### 3. 启动激光雷达节点
```bash
# 终端2：激光雷达
cd /home/runner/work/autonomous-car-ros2-jazzy-rpi4/autonomous-car-ros2-jazzy-rpi4
./start_lidar.sh
```

#### 4. 运行自动驾驶功能

**选项A：运行ACC（自适应巡航控制）**
```bash
cd function
python3 Python_Nano_ACC_202302V2.py
```

**选项B：运行AEB（紧急制动）**
```bash
cd function
python3 Python_Nano_AEB_202302V2.py
```

**选项C：运行车道保持**
```bash
cd function/LKS_Hough
python3 Python_Nano_LKS_202302V1.py
```

**选项D：运行自动泊车**
```bash
cd function/APS
python3 Python_Nano_APS_202304v2.py
```

### 📖 硬件驱动API

#### 电机控制
```python
from Hardware import motor_driver as Motor
from pyvesc.VESC.messages import SetDutyCycle, SetRPM

# 设置电机占空比（0.0到1.0）
Motor.get_values_example(SetDutyCycle(0.08))

# 直接设置电机RPM
Motor.get_values_example(SetRPM(900))

# 获取当前速度
current_speed = Motor.get_values_example(SetDutyCycle(0.08))
# 返回速度，单位：厘米/秒
```

#### 舵机控制
```python
from Hardware import servo_driver

# 设置舵机位置（500-2500）
servo_driver.servo_angle_write(1500)  # 中心位置

# 读取当前舵机位置
servo_driver.servo_angle_read()
```

#### 摄像头（ROS2节点）
```python
# 订阅摄像头话题
ros2 topic echo /camera/image_raw
```

#### 超声波传感器
```python
from Hardware import ultrasonic_driver as Ultrasonic

# 获取距离（厘米）
distance = Ultrasonic.get_distance()
print(f"距离：{distance} 厘米")
```

#### IMU
```python
from Hardware import imu_driver as IMU

# 获取IMU数据
data = IMU.get_imu_data()
# 返回：加速度、陀螺仪、磁场、欧拉角
```

#### GPS
```python
from Hardware import gps_driver as GPS

# 获取GPS坐标
lat, lon, altitude, satellites = GPS.get_gps_data()
```

#### 毫米波雷达
```python
from Hardware import millimeterwave_driver as MMW

# 打开雷达端口
port = MMW.openMMWPort("/dev/ttyTHS1", 115200)

# 获取距离和速度
distance, speed = MMW.MMWDetection(port)
# distance单位：厘米，speed单位：厘米/秒
```

### ⚙️ 配置

#### 串口配置
为设备命名创建符号链接：
```bash
# 电机VESC
sudo ln -s /dev/ttyACM0 /dev/motor

# 舵机
sudo ln -s /dev/ttyUSB0 /dev/servo

# 毫米波雷达
sudo ln -s /dev/ttyTHS1 /dev/mmw_radar
```

#### 车辆参数
根据需要编辑功能模块中的常量：
- `TRANS_RATIO = 6.287` - 齿轮传动比
- `WHEEL_RADIUS = 0.032` - 车轮半径（米）
- `WHEEL_BASE = 25` - 车辆轴距（厘米）

### 🔍 PID控制器
PID控制器（`control/pid_controller.py`）用于闭环电机速度控制：

```python
from simple_pid import PID

pid = PID(0.8, 1, 0.01, setpoint=0)
pid.output_limits = (-3000, 3000)

# 在控制循环中
rpm_error = current_rpm - target_rpm
rpm_correction = pid(rpm_error)
```

### 📂 项目结构

```
autonomous-car-ros2-jazzy-rpi4/
├── Hardware/                    # 硬件驱动层
│   ├── camera_driver.py         # ROS2摄像头发布器
│   ├── motor_driver.py          # VESC电机控制
│   ├── servo_driver.py          # SCServo控制
│   ├── imu_driver.py            # 9轴IMU
│   ├── gps_driver.py            # GPS/GNSS
│   ├── ultrasonic_driver.py     # HC-SR04超声波
│   ├── millimeterwave_driver.py # 毫米波雷达
│   ├── pyvesc/                  # VESC协议库
│   └── scservo_sdk/             # SCServo SDK
├── control/                     # 控制算法
│   └── pid_controller.py        # PID速度控制
├── function/                    # 自动驾驶功能
│   ├── Python_Nano_ACC_202302V2.py      # 自适应巡航控制
│   ├── Python_Nano_AEB_202302V2.py      # 紧急制动
│   ├── LKS_Hough/               # 车道保持系统
│   ├── APS/                     # 自动泊车系统
│   ├── PCS/                     # 泊车碰撞系统
│   ├── Zebra/                   # 斑马线检测
│   ├── License_Plate/           # 车牌识别
│   ├── traffic_lights/          # 交通信号灯检测
│   └── ultralytics-main/        # YOLOv8模型
├── rplidar_ros/                 # RPLidar ROS2驱动
├── start_lidar.sh               # 激光雷达启动脚本
└── main.py                      # 主入口点（待实现）
```

### 🛠️ 故障排除

#### 串口权限问题
```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS*
# 注销后重新登录
```

#### 摄像头未找到
```bash
# 检查摄像头设备
ls -l /dev/video*
v4l2-ctl --list-devices

# 测试摄像头
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('摄像头正常' if cap.isOpened() else '摄像头错误')"
```

#### ROS2环境
```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 📝 开发指南

#### 添加新功能
1. 在 `function/` 下创建新目录
2. 遵循命名规范：`Python_Nano_<功能>_<YYYYMM><版本>.py`
3. 从 `Hardware/` 导入所需的硬件驱动
4. 在文件顶部添加配置常量
5. 用注释记录控制逻辑

#### 代码风格
- 为中文开发者使用中文注释（当前风格）
- 为函数添加英文文档字符串
- 遵循PEP 8 Python风格指南
- 使用描述性变量名

### 🤝 贡献

欢迎贡献！请遵循以下步骤：
1. Fork仓库
2. 创建功能分支
3. 添加全面的文档
4. 如果可能在硬件上测试
5. 提交Pull Request

### 📄 许可证

[添加许可证信息]

### 👥 作者

- xxhh12366 - 原作者

### 🙏 致谢

- ROS2 Jazzy社区
- RPLidar ROS2驱动
- Ultralytics YOLOv8
- VESC项目
- SCServo SDK

---

**Note**: This project is under active development. Some features may require calibration and tuning for your specific hardware setup.

**注意**：此项目正在积极开发中。某些功能可能需要针对您的特定硬件设置进行校准和调整。
