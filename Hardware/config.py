"""
Hardware Configuration for Autonomous Car
Centralized configuration for all hardware parameters
"""

# Motor Configuration
MOTOR_CONFIG = {
    'serial_port': '/dev/motor',
    'baudrate': 115200,
    'timeout': 0.01,
    'trans_ratio': 6.287,  # Gear transmission ratio
    'wheel_radius': 0.032,  # Wheel radius in meters
}

# Alternative motor port configurations
MOTOR_PORTS = {
    'default': '/dev/motor',
    'usb2': '/dev/ttyUSB2',
    'ttyS0': '/dev/ttyS0',
}

# Servo Configuration
SERVO_CONFIG = {
    'device_name': '/dev/ttyUSB0',
    'servo_id': 1,
    'baudrate': 1000000,
    'min_position': 500,     # Full range: 500-2500 (default for SCServo)
    'max_position': 2500,    # Adjust based on your servo's safe range
    'moving_speed': 0,  # 0 = max speed
    'moving_acc': 0,    # 0 = max acceleration
    'protocol_end': 0,  # STS/SMS=0, SCS=1
}

# PID Controller Configuration
PID_CONFIG = {
    'kp': 0.8,
    'ki': 1.0,
    'kd': 0.01,
    'target_rpm': 2000,
    'output_limit_multiplier': 1.5,
}

# Ultrasonic Sensor Configuration
ULTRASONIC_CONFIG = {
    'trigger_pin': 23,
    'echo_pin': 24,
}

# Camera Configuration
CAMERA_CONFIG = {
    'device_index': 0,
    'width': 640,
    'height': 480,
    'fps': 30,
}

# GPS Configuration
GPS_CONFIG = {
    'serial_port': '/dev/ttyUSB1',
    'baudrate': 9600,
}

# IMU Configuration
IMU_CONFIG = {
    'i2c_bus': 1,
    'address': 0x68,
}

# Millimeter Wave Radar Configuration
RADAR_CONFIG = {
    'serial_port': '/dev/ttyUSB3',
    'baudrate': 115200,
}
