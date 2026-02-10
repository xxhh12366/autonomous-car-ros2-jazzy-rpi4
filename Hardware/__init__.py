"""
Hardware Package
================

Centralized hardware drivers for the autonomous car project.

Available drivers:
- motor_driver: VESC motor controller driver (3650 21.5T motor)
- servo_driver: SCServo SCS20-360T servo controller driver
- millimeterwave_driver: HLR12 millimeter wave radar driver
- camera_driver: ROS2 camera publisher driver
- imu_driver: IMU sensor driver
- gps_driver: GPS sensor driver
- ultrasonic_driver: Ultrasonic sensor driver

Usage:
    from Hardware import motor_driver
    from Hardware import servo_driver
    from Hardware import millimeterwave_driver
"""

__version__ = '1.0.0'
__author__ = 'Relaxing Technology Chongqing Co.,Ltd.'

# Import main driver modules for easier access
try:
    from . import motor_driver
    from . import servo_driver
    from . import millimeterwave_driver
    from . import camera_driver
    from . import imu_driver
    from . import gps_driver
    from . import ultrasonic_driver
except ImportError as e:
    # Some dependencies may not be available in all environments
    pass
