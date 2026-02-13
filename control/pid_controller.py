import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Hardware'))

from pyvesc.VESC.messages import SetDutyCycle
from simple_pid import PID
from motor_controller import MotorController
from config import MOTOR_CONFIG, PID_CONFIG

# Initialize motor controller with configuration
motor = MotorController(
    serial_port=MOTOR_CONFIG.get('serial_port', '/dev/ttyS0'),
    baudrate=MOTOR_CONFIG['baudrate'],
    trans_ratio=MOTOR_CONFIG['trans_ratio'],
    wheel_radius=MOTOR_CONFIG['wheel_radius'],
    timeout=MOTOR_CONFIG['timeout']
)

# Initialize PID controller with configuration
Target_rpm = PID_CONFIG['target_rpm']
pid = PID(
    PID_CONFIG['kp'], 
    PID_CONFIG['ki'], 
    PID_CONFIG['kd'], 
    setpoint=0
)
pid.output_limits = (
    -Target_rpm * PID_CONFIG['output_limit_multiplier'], 
    Target_rpm * PID_CONFIG['output_limit_multiplier']
)




def get_values_example(SetMode):
    """
    Get motor RPM using unified motor controller
    
    Args:
        SetMode: VESC command mode
        
    Returns:
        Motor RPM or 'error' if communication fails
    """
    return motor.get_rpm(SetMode)


if __name__ == "__main__":
    SetDutyCycle_Values = 0.05
    SetMode = SetDutyCycle(SetDutyCycle_Values)
    Duty_PID = SetDutyCycle_Values
    
    try:
        while True:
            SetMode = SetDutyCycle(Duty_PID)
            RPM_NOW = get_values_example(SetMode)
            if RPM_NOW != 'error':
                RPM_ERR = RPM_NOW - Target_rpm
                RPM_PID = pid(RPM_ERR) + RPM_NOW
                Duty_PID = RPM_PID/15200 + 1/152
                print("RPM_NOW = ", RPM_NOW, "RPM_ERR = ", RPM_ERR, "RPM_PID = ", RPM_PID)   
                print("Duty_PID = ", Duty_PID)
            else:
                pass
    except KeyboardInterrupt:
        print("\nPID controller stopped")                                    