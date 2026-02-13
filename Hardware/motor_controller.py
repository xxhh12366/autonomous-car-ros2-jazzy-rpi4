"""
Unified Motor Controller for VESC-based motors
Consolidates motor control functionality previously duplicated across multiple modules
"""
import math
import pyvesc
from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
import serial
from typing import Union


class MotorController:
    """
    Controller for VESC motor ESC via serial communication
    
    Attributes:
        serial_port: Serial port device path (default: '/dev/motor')
        baudrate: Serial communication baudrate (default: 115200)
        trans_ratio: Gear transmission ratio (default: 6.287)
        wheel_radius: Wheel radius in meters (default: 0.032)
    """
    
    def __init__(self, 
                 serial_port: str = '/dev/motor',
                 baudrate: int = 115200,
                 trans_ratio: float = 6.287,
                 wheel_radius: float = 0.032,
                 timeout: float = 0.01):
        """
        Initialize motor controller
        
        Args:
            serial_port: Path to serial device
            baudrate: Serial communication speed
            trans_ratio: Gear transmission ratio
            wheel_radius: Wheel radius in meters
            timeout: Serial read timeout in seconds
        """
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.trans_ratio = trans_ratio
        self.wheel_radius = wheel_radius
        self.timeout = timeout
    
    def get_rpm(self, set_mode) -> Union[float, str]:
        """
        Get current motor RPM
        
        Args:
            set_mode: VESC set mode command (SetDutyCycle or SetRPM)
            
        Returns:
            Current motor RPM or 'error' if communication fails
        """
        try:
            with serial.Serial(self.serial_port, baudrate=self.baudrate, timeout=self.timeout) as ser:
                ser.write(pyvesc.encode(set_mode))
                ser.write(pyvesc.encode_request(GetValues))
                (response, consumed) = pyvesc.decode(ser.read(78))
                if consumed == 78:
                    return response.rpm
                else:
                    return 'error'
        except Exception as e:
            print(f"Motor communication error: {e}")
            return 'error'
    
    def get_velocity(self, set_mode) -> Union[float, str]:
        """
        Get current wheel velocity in cm/s
        
        Args:
            set_mode: VESC set mode command (SetDutyCycle or SetRPM)
            
        Returns:
            Current wheel velocity in cm/s or 'error' if communication fails
        """
        try:
            with serial.Serial(self.serial_port, baudrate=self.baudrate, timeout=self.timeout) as ser:
                ser.write(pyvesc.encode(set_mode))
                ser.write(pyvesc.encode_request(GetValues))
                (response, consumed) = pyvesc.decode(ser.read(78))
                if consumed == 78:
                    # Calculate velocity: rpm -> wheel speed in cm/s
                    # Formula: velocity = (rpm / trans_ratio) * (2 * pi * wheel_radius) / 60 * 100
                    velocity = response.rpm / self.trans_ratio * (2 * math.pi * self.wheel_radius) / 60 * 100
                    return velocity
                else:
                    return 'error'
        except Exception as e:
            print(f"Motor communication error: {e}")
            return 'error'
    
    def rpm_to_velocity(self, rpm: float) -> float:
        """
        Convert motor RPM to wheel velocity in cm/s
        
        Args:
            rpm: Motor RPM
            
        Returns:
            Wheel velocity in cm/s
        """
        velocity = rpm / self.trans_ratio * (2 * math.pi * self.wheel_radius) / 60 * 100
        return velocity
    
    def print_velocity(self, rpm: float) -> float:
        """
        Print and return wheel velocity
        
        Args:
            rpm: Motor RPM
            
        Returns:
            Wheel velocity in cm/s
        """
        velocity = self.rpm_to_velocity(rpm)
        print("小车的速度为%.6fcm/s" % velocity)
        return velocity


if __name__ == "__main__":
    # Test motor controller
    motor = MotorController()
    
    # Test with duty cycle mode
    SetDutyCycle_Values = 0.07
    SetMode = SetDutyCycle(SetDutyCycle_Values)
    
    print("Testing motor controller...")
    velocity = motor.get_velocity(SetMode)
    print(f"Current velocity: {velocity} cm/s")
