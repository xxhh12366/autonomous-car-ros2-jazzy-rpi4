"""
Unified Servo Controller for SCServo motors
Consolidates servo control functionality previously duplicated across multiple modules
"""
import os
import sys

# Add scservo_sdk to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'scservo_sdk'))

from scservo_sdk import *
from typing import Tuple, Optional


class ServoController:
    """
    Controller for SCServo motors via serial communication
    
    Attributes:
        device_name: Serial port device path (default: '/dev/ttyUSB0')
        servo_id: Servo motor ID (default: 1)
        baudrate: Serial communication baudrate (default: 1000000)
        min_position: Minimum position value (default: 500)
        max_position: Maximum position value (default: 2500)
        moving_speed: Servo moving speed (default: 0)
        moving_acc: Servo moving acceleration (default: 0)
    """
    
    # Control table addresses
    ADDR_SCS_TORQUE_ENABLE = 40
    ADDR_SCS_GOAL_ACC = 41
    ADDR_SCS_GOAL_POSITION = 42
    ADDR_SCS_GOAL_SPEED = 46
    ADDR_SCS_PRESENT_POSITION = 56
    
    # Default thresholds
    SCS_MOVING_STATUS_THRESHOLD = 20
    
    def __init__(self,
                 device_name: str = '/dev/ttyUSB0',
                 servo_id: int = 1,
                 baudrate: int = 1000000,
                 min_position: int = 500,
                 max_position: int = 2500,
                 moving_speed: int = 0,
                 moving_acc: int = 0,
                 protocol_end: int = 0):
        """
        Initialize servo controller
        
        Args:
            device_name: Path to serial device
            servo_id: Servo motor ID
            baudrate: Serial communication speed
            min_position: Minimum position value
            max_position: Maximum position value
            moving_speed: Default moving speed (0 = max speed)
            moving_acc: Default acceleration (0 = max acceleration)
            protocol_end: Protocol bit end (STS/SMS=0, SCS=1)
        """
        self.device_name = device_name
        self.servo_id = servo_id
        self.baudrate = baudrate
        self.min_position = min_position
        self.max_position = max_position
        self.moving_speed = moving_speed
        self.moving_acc = moving_acc
        self.protocol_end = protocol_end
        
        # Initialize port and packet handlers
        self.port_handler = PortHandler(device_name)
        self.packet_handler = PacketHandler(protocol_end)
        
        # Open port and set baudrate
        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port {device_name}")
        
        if not self.port_handler.setBaudRate(baudrate):
            raise RuntimeError(f"Failed to set baudrate to {baudrate}")
        
        # Initialize servo settings
        self._initialize_servo()
    
    def _initialize_servo(self):
        """Initialize servo acceleration and speed settings"""
        # Write SCServo acc
        scs_comm_result, scs_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.servo_id, self.ADDR_SCS_GOAL_ACC, self.moving_acc
        )
        if scs_comm_result != COMM_SUCCESS:
            print(f"ACC init error: {self.packet_handler.getTxRxResult(scs_comm_result)}")
        elif scs_error != 0:
            print(f"ACC init error: {self.packet_handler.getRxPacketError(scs_error)}")
        
        # Write SCServo speed
        scs_comm_result, scs_error = self.packet_handler.write2ByteTxRx(
            self.port_handler, self.servo_id, self.ADDR_SCS_GOAL_SPEED, self.moving_speed
        )
        if scs_comm_result != COMM_SUCCESS:
            print(f"Speed init error: {self.packet_handler.getTxRxResult(scs_comm_result)}")
        elif scs_error != 0:
            print(f"Speed init error: {self.packet_handler.getRxPacketError(scs_error)}")
    
    def write_position(self, target_position: int) -> bool:
        """
        Write target position to servo
        
        Args:
            target_position: Target position value (should be within min/max range)
            
        Returns:
            True if successful, False otherwise
        """
        # Clamp position to valid range
        target_position = max(self.min_position, min(self.max_position, target_position))
        
        scs_comm_result, scs_error = self.packet_handler.write2ByteTxRx(
            self.port_handler, self.servo_id, self.ADDR_SCS_GOAL_POSITION, target_position
        )
        
        if scs_comm_result != COMM_SUCCESS:
            print(f"Position write error: {self.packet_handler.getTxRxResult(scs_comm_result)}")
            return False
        elif scs_error != 0:
            print(f"Position write error: {self.packet_handler.getRxPacketError(scs_error)}")
            return False
        
        return True
    
    def read_position(self) -> Tuple[Optional[int], Optional[int]]:
        """
        Read current servo position and speed
        
        Returns:
            Tuple of (position, speed) or (None, None) if read fails
        """
        scs_present_position_speed, scs_comm_result, scs_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.servo_id, self.ADDR_SCS_PRESENT_POSITION
        )
        
        if scs_comm_result != COMM_SUCCESS:
            print(f"Position read error: {self.packet_handler.getTxRxResult(scs_comm_result)}")
            return None, None
        elif scs_error != 0:
            print(f"Position read error: {self.packet_handler.getRxPacketError(scs_error)}")
            return None, None
        
        scs_present_position = SCS_LOWORD(scs_present_position_speed)
        scs_present_speed = SCS_HIWORD(scs_present_position_speed)
        
        return scs_present_position, SCS_TOHOST(scs_present_speed, 15)
    
    def wait_for_position(self, goal_position: int, verbose: bool = False) -> bool:
        """
        Wait for servo to reach target position
        
        Args:
            goal_position: Target position
            verbose: Print status updates
            
        Returns:
            True when position reached
        """
        while True:
            position, speed = self.read_position()
            if position is None:
                continue
                
            if verbose:
                print(f"[ID:{self.servo_id:03d}] GoalPos:{goal_position:03d} PresPos:{position:03d} PresSpd:{speed}")
            
            if abs(goal_position - position) <= self.SCS_MOVING_STATUS_THRESHOLD:
                return True
    
    def close(self):
        """Close the serial port"""
        self.port_handler.closePort()
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()


if __name__ == '__main__':
    # Test servo controller
    print("Testing servo controller...")
    
    try:
        with ServoController() as servo:
            # Test position control
            target_position = 2000
            print(f"Moving to position {target_position}")
            servo.write_position(target_position)
            servo.wait_for_position(target_position, verbose=True)
            print("Position reached!")
    except Exception as e:
        print(f"Servo test error: {e}")
