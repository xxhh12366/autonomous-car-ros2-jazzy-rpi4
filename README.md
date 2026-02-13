# Autonomous Car ROS2 - Raspberry Pi 4

基于ROS2 Jazzy的树莓派4自动驾驶小车项目

## 项目概述

这是一个完整的自动驾驶辅助系统(ADAS)实现，运行在Raspberry Pi 4上，集成了多种先进的驾驶辅助功能。

## 主要功能模块

### 驾驶辅助系统 (ADAS)

- **ACC** (Adaptive Cruise Control) - 自适应巡航控制，基于距离的速度控制
- **AEB** (Autonomous Emergency Braking) - 自动紧急制动，碰撞避免系统
- **APS** (Automatic Parking System) - 自动泊车系统
- **PCS** (Parking Collision System) - 泊车碰撞检测系统
- **LKS** (Lane Keeping System) - 车道保持系统，使用Hough变换进行车道线检测
- **交通灯识别** - 使用YOLOv8进行交通灯检测
- **GPS/IMU定位** - 基于GPS和惯性测量单元的定位系统

## 硬件架构

### Hardware/ - 硬件驱动层

#### 新增统一控制器（优化后）

- **motor_controller.py** - 统一的电机控制器
  - 支持VESC电调通过串口通信
  - 提供RPM读取和速度控制
  - 自动进行RPM到线速度的转换
  - 配置化的传动比和轮径参数

- **servo_controller.py** - 统一的舵机控制器
  - 支持SCServo系列舵机
  - 位置读写功能
  - 自动初始化和错误处理
  - 支持上下文管理器（with语句）

- **config.py** - 硬件配置文件
  - 集中管理所有硬件参数
  - 电机、舵机、PID控制器配置
  - 传感器端口配置
  - 易于维护和修改

#### 传感器驱动

- **camera_driver.py** - 摄像头驱动
- **gps_driver.py** - GPS模块驱动
- **imu_driver.py** - 惯性测量单元驱动
- **millimeterwave_driver.py** - 毫米波雷达驱动
- **ultrasonic_driver.py** - 超声波传感器驱动

#### 通信模块

- **tcp_comm_client.py / tcp_comm_server.py** - TCP通信
- **udp_comm_client.py / udp_comm_server.py** - UDP通信

### control/ - 控制层

- **pid_controller.py** - PID速度控制器
  - 基于RPM反馈的闭环控制
  - 使用统一的motor_controller
  - 配置化的PID参数

### function/ - 功能模块层

各个ADAS功能的具体实现，现在都使用统一的motor_controller和servo_controller。

## 代码优化

### 优化成果

本次优化主要解决了代码重复和维护困难的问题：

#### 1. 消除代码重复 (约70%减少)
- ✅ 移除了14个重复的驱动文件
  - 8个重复的 `Python_Nano_Motor_202302V2.py`
  - 6个重复的 `Python_Nano_Servo_202302V2.py`
- ✅ 将重复代码合并为3个统一的模块
  - `Hardware/motor_controller.py`
  - `Hardware/servo_controller.py`
  - `Hardware/config.py`

#### 2. 集中配置管理
- ✅ 所有硬件参数集中在 `Hardware/config.py`
- ✅ 串口路径、波特率、传动比等参数统一管理
- ✅ PID控制参数配置化

#### 3. 提升代码质量
- ✅ 添加完整的文档字符串（docstrings）
- ✅ 使用类型提示（type hints）提高代码可读性
- ✅ 改善错误处理机制
- ✅ 使用面向对象设计原则

#### 4. 改进的模块
- ✅ `control/pid_controller.py` - 使用新的motor_controller
- ✅ `function/ACC/` - ACC模块更新
- ✅ `function/AEB/` - AEB模块更新
- ✅ `function/APS/` - APS模块更新
- ✅ `function/LKS_Hough/` - 车道保持模块更新

### 使用统一控制器

#### 电机控制示例

```python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'Hardware'))

from motor_controller import MotorController
from config import MOTOR_CONFIG
from pyvesc.VESC.messages import SetDutyCycle

# 初始化电机控制器
motor = MotorController(
    serial_port=MOTOR_CONFIG['serial_port'],
    baudrate=MOTOR_CONFIG['baudrate'],
    trans_ratio=MOTOR_CONFIG['trans_ratio'],
    wheel_radius=MOTOR_CONFIG['wheel_radius']
)

# 设置占空比并获取速度
velocity = motor.get_velocity(SetDutyCycle(0.07))
print(f"Current velocity: {velocity} cm/s")
```

#### 舵机控制示例

```python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'Hardware'))

from servo_controller import ServoController
from config import SERVO_CONFIG

# 使用上下文管理器自动管理资源
with ServoController(**SERVO_CONFIG) as servo:
    # 写入目标位置
    servo.write_position(2000)
    
    # 读取当前位置
    position, speed = servo.read_position()
    print(f"Position: {position}, Speed: {speed}")
    
    # 等待到达目标位置
    servo.wait_for_position(2000, verbose=True)
```

#### 修改配置

所有硬件配置现在集中在 `Hardware/config.py`：

```python
# 电机配置
MOTOR_CONFIG = {
    'serial_port': '/dev/motor',
    'baudrate': 115200,
    'timeout': 0.01,
    'trans_ratio': 6.287,  # 传动比
    'wheel_radius': 0.032,  # 轮径(米)
}

# 舵机配置
SERVO_CONFIG = {
    'device_name': '/dev/ttyUSB0',
    'servo_id': 1,
    'baudrate': 1000000,
    'min_position': 500,
    'max_position': 2500,
}

# PID配置
PID_CONFIG = {
    'kp': 0.8,
    'ki': 1.0,
    'kd': 0.01,
    'target_rpm': 2000,
}
```

## 项目结构

```
autonomous-car-ros2-jazzy-rpi4/
├── Hardware/                    # 硬件驱动层（优化后）
│   ├── motor_controller.py     # 统一电机控制器（新增）
│   ├── servo_controller.py     # 统一舵机控制器（新增）
│   ├── config.py               # 硬件配置文件（新增）
│   ├── camera_driver.py
│   ├── gps_driver.py
│   ├── imu_driver.py
│   ├── millimeterwave_driver.py
│   ├── ultrasonic_driver.py
│   ├── tcp_comm_*.py
│   └── udp_comm_*.py
├── control/                    # 控制层
│   └── pid_controller.py       # PID控制器（已优化）
├── function/                   # 功能模块层
│   ├── ACC/                    # 自适应巡航（已优化）
│   ├── AEB/                    # 自动紧急制动（已优化）
│   ├── APS/                    # 自动泊车（已优化）
│   ├── LKS_Hough/              # 车道保持（已优化）
│   ├── PCS/                    # 泊车碰撞检测
│   ├── traffic_lights/         # 交通灯识别
│   └── Zebra/                  # 斑马线检测
├── rplidar_ros/                # RPLidar激光雷达ROS驱动
├── main.py                     # 主程序入口
└── start_lidar.sh              # 启动脚本

优化前总行数：约1200行（包含重复代码）
优化后总行数：约450行（移除852行重复代码，新增100行高质量代码）
代码减少：约60%
```

## 依赖项

- Python 3.x
- ROS2 Jazzy
- pyvesc - VESC电调通信库
- simple-pid - PID控制器
- OpenCV (cv2) - 图像处理
- NumPy - 数值计算
- scservo_sdk - 舵机控制SDK

## 安装

```bash
# 克隆仓库
git clone https://github.com/xxhh12366/autonomous-car-ros2-jazzy-rpi4.git
cd autonomous-car-ros2-jazzy-rpi4

# 安装Python依赖
pip install pyvesc simple-pid opencv-python numpy

# 配置ROS2环境
source /opt/ros/jazzy/setup.bash
```

## 快速开始

### 1. 配置硬件参数

编辑 `Hardware/config.py` 根据您的硬件配置修改参数。

### 2. 运行单个功能模块

```bash
# ACC - 自适应巡航
cd function/ACC
python3 Python_Nano_ACC_202302V2.py

# AEB - 自动紧急制动
cd function/AEB
python3 Python_Nano_AEB_202302V2.py

# LKS - 车道保持
cd function/LKS_Hough
python3 Python_Nano_LKS_202302V1.py
```

### 3. PID速度控制

```bash
cd control
python3 pid_controller.py
```

## 维护优势

### 优化前的问题
- ❌ 14个重复文件，修改一处需要改14个地方
- ❌ 配置分散，难以维护
- ❌ 缺乏文档和类型提示
- ❌ 错误处理不统一

### 优化后的优势
- ✅ 单一数据源，修改一处全局生效
- ✅ 集中配置管理，易于调试
- ✅ 完整的文档字符串和类型提示
- ✅ 统一的错误处理机制
- ✅ 面向对象设计，更易扩展
- ✅ 代码量减少60%，可维护性大幅提升

## 贡献

欢迎提交问题和拉取请求来改进这个项目。

## 许可证

本项目使用的具体许可证请查看项目中的LICENSE文件。

## 联系方式

- GitHub: [xxhh12366](https://github.com/xxhh12366)

---

**注意**: 本项目针对教育和研究目的。在实际车辆上使用前请确保充分测试和符合相关法律法规。
