# 迁移指南 - 使用统一控制器

本指南说明如何将旧的motor/servo代码迁移到新的统一控制器。

## 为什么需要迁移

优化前的问题：
- 每个功能模块都有自己的motor和servo文件副本
- 修改一个bug需要在14个地方重复修改
- 配置分散，难以维护
- 代码重复率高达70%

优化后的优势：
- 单一数据源，只需修改一处
- 集中配置管理
- 更好的错误处理
- 完整的文档和类型提示

## 迁移步骤

### 1. 更新导入语句

#### 旧代码（电机）
```python
import Python_Nano_Motor_202302V2 as Motor
```

#### 新代码（电机）
```python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'Hardware'))

from motor_controller import MotorController
from config import MOTOR_CONFIG

# 初始化电机控制器
motor = MotorController(
    serial_port=MOTOR_CONFIG['serial_port'],
    baudrate=MOTOR_CONFIG['baudrate'],
    trans_ratio=MOTOR_CONFIG['trans_ratio'],
    wheel_radius=MOTOR_CONFIG['wheel_radius'],
    timeout=MOTOR_CONFIG['timeout']
)
```

#### 旧代码（舵机）
```python
import Python_Nano_Servo_202302V2 as Servo
```

#### 新代码（舵机）
```python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'Hardware'))

from servo_controller import ServoController
from config import SERVO_CONFIG

# 初始化舵机控制器
servo = ServoController(
    device_name=SERVO_CONFIG['device_name'],
    servo_id=SERVO_CONFIG['servo_id'],
    baudrate=SERVO_CONFIG['baudrate'],
    min_position=SERVO_CONFIG['min_position'],
    max_position=SERVO_CONFIG['max_position'],
    moving_speed=SERVO_CONFIG['moving_speed'],
    moving_acc=SERVO_CONFIG['moving_acc'],
    protocol_end=SERVO_CONFIG['protocol_end']
)
```

### 2. 更新函数调用

#### 电机函数映射

| 旧函数 | 新函数 | 说明 |
|--------|--------|------|
| `Motor.get_values_example(SetMode)` | `motor.get_velocity(SetMode)` | 获取速度（cm/s） |
| | `motor.get_rpm(SetMode)` | 获取RPM |
| | `motor.rpm_to_velocity(rpm)` | RPM转速度 |

#### 舵机函数映射

| 旧函数 | 新函数 | 说明 |
|--------|--------|------|
| `Servo.servo_angle_write(position)` | `servo.write_position(position)` | 写入位置 |
| `Servo.servo_angle_read()` | `servo.read_position()` | 读取位置和速度 |
| | `servo.wait_for_position(goal, verbose)` | 等待到达位置 |

### 3. 代码示例

#### 示例1：ACC模块迁移

**旧代码:**
```python
import Python_Nano_Motor_202302V2 as Motor
from pyvesc.VESC.messages import SetDutyCycle

duty = 0.08
Motor.get_values_example(SetDutyCycle(duty))
```

**新代码:**
```python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'Hardware'))

from motor_controller import MotorController
from config import MOTOR_CONFIG
from pyvesc.VESC.messages import SetDutyCycle

# 初始化
motor = MotorController(**MOTOR_CONFIG)

# 使用
duty = 0.08
velocity = motor.get_velocity(SetDutyCycle(duty))
print(f"Velocity: {velocity} cm/s")
```

#### 示例2：APS模块迁移（电机+舵机）

**旧代码:**
```python
import Python_Nano_Motor_202302V2 as Motor
import Python_Nano_Servo_202302V2 as Servo

Servo.servo_angle_write(1500)
velocity = Motor.get_values_example(SetDutyCycle(0.06))
position = Servo.servo_angle_read()
```

**新代码:**
```python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'Hardware'))

from motor_controller import MotorController
from servo_controller import ServoController
from config import MOTOR_CONFIG, SERVO_CONFIG
from pyvesc.VESC.messages import SetDutyCycle

# 初始化
motor = MotorController(**MOTOR_CONFIG)
servo = ServoController(**SERVO_CONFIG)

# 使用
servo.write_position(1500)
velocity = motor.get_velocity(SetDutyCycle(0.06))
position, speed = servo.read_position()
```

#### 示例3：LKS车道保持迁移

**旧代码:**
```python
import Python_Nano_Servo_202302V2 as Servo

uart = 1500
Servo.servo_angle_write(uart)
```

**新代码:**
```python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'Hardware'))

from servo_controller import ServoController
from config import SERVO_CONFIG

# 初始化
servo = ServoController(**SERVO_CONFIG)

# 使用
uart = 1500
servo.write_position(uart)
```

### 4. 修改配置参数

如果您的硬件配置与默认值不同，请修改 `Hardware/config.py`：

```python
# 修改电机串口
MOTOR_CONFIG = {
    'serial_port': '/dev/ttyS0',  # 改为您的串口
    'baudrate': 115200,
    'trans_ratio': 6.287,
    'wheel_radius': 0.032,
    'timeout': 0.01,
}

# 修改舵机配置
SERVO_CONFIG = {
    'device_name': '/dev/ttyUSB0',  # 改为您的串口
    'servo_id': 1,
    'baudrate': 1000000,
    # ... 其他参数
}
```

### 5. 删除旧文件

迁移完成后，删除模块中的重复文件：

```bash
# 删除电机文件
rm Python_Nano_Motor_202302V2.py

# 删除舵机文件
rm Python_Nano_Servo_202302V2.py
```

## 已迁移的模块

以下模块已经完成迁移，可作为参考：

✅ `control/pid_controller.py`
✅ `function/ACC/Python_Nano_ACC_202302V2.py`
✅ `function/AEB/Python_Nano_AEB_202302V2.py`
✅ `function/APS/Python_Nano_APS_202304v2.py`
✅ `function/LKS_Hough/Python_Nano_LKS_202302V1.py`

## 待迁移的模块

如果以下模块中还有代码使用旧的motor/servo文件，请参考本指南进行迁移：

- `function/PCS/` - 泊车碰撞检测
- `function/Zebra/` - 斑马线检测
- `function/License_Plate/` - 车牌识别
- `function/traffic_lights/` - 交通灯识别
- `function/ultralytics-main/` - YOLO相关

## 常见问题

### Q: 为什么servo.read_position()返回两个值？
A: 新的控制器同时返回位置和速度：`position, speed = servo.read_position()`

### Q: 如何处理错误返回值？
A: 新控制器在失败时返回 `'error'` 字符串，或者 `None` 值。请检查返回值类型。

### Q: 可以临时使用不同的串口吗？
A: 可以，创建控制器时传入不同参数：
```python
motor = MotorController(serial_port='/dev/ttyUSB2')
```

### Q: 旧代码还能用吗？
A: 由于删除了重复文件，旧代码将无法找到导入。必须迁移到新控制器。

## 获取帮助

如果迁移过程中遇到问题：
1. 参考已迁移模块的代码
2. 查看 `Hardware/motor_controller.py` 和 `servo_controller.py` 的文档字符串
3. 检查 `Hardware/config.py` 中的配置参数

---

通过完成迁移，您的代码将：
- ✅ 更易维护
- ✅ 更少错误
- ✅ 更好的性能
- ✅ 更清晰的结构
