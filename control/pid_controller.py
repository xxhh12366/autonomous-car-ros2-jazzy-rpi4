
import pyvesc
from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
import serial
from simple_pid import PID

serialport = '/dev/ttyS0'
Target_rpm = 2000
pid = PID(0.8, 1, 0.01, setpoint = 0)
pid.output_limits = (-Target_rpm*1.5, Target_rpm*1.5)


def get_values_example(SetMode):
    with serial.Serial(serialport, baudrate=115200, timeout=0.01) as ser:
        ser.write(pyvesc.encode(SetMode))        
        ser.write(pyvesc.encode_request(GetValues))
        (response, consumed) = pyvesc.decode(ser.read(78))
        if consumed ==  78:
            #print(response, consumed)
            #print(response.rpm)
            return response.rpm
        else:
            return 'error'



                    
if __name__ == "__main__":
    SetDutyCycle_Values = 0.05
    SetMode = SetDutyCycle(SetDutyCycle_Values)
    Duty_PID = SetDutyCycle_Values
    while True:
        SetMode = SetDutyCycle(Duty_PID)
        RPM_NOW = get_values_example(SetMode)
        if RPM_NOW != 'error':
            RPM_ERR = RPM_NOW - Target_rpm
            RPM_PID = pid(RPM_ERR) + RPM_NOW
            Duty_PID =  RPM_PID/15200 + 1/152
            print("RPM_NOW = ",RPM_NOW,"RPM_ERR = ",RPM_ERR,"RPM_PID = ",RPM_PID)   
            print("Duty_PID = ",Duty_PID)
        else:
            pass                                    