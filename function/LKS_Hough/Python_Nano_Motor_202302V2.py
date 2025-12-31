import pyvesc
from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
import serial

serialport = '/dev/motor'
Trans_Ratio = 6.287  # 传动比为6.1
Wheel_Radius = 0.032  # 半径为0.032米


def printSpeed(rpm):
    """
    打印车速

    :param rpm: response.rpm, 电机转速
    :return:
    """

    velocity = rpm / Trans_Ratio * (2 * 3.14 * Wheel_Radius) / 60 * 100
    print("小车的速度为%.6fcm/s" % velocity)
    return velocity


def get_values_example(SetMode):
    with serial.Serial(serialport, baudrate=115200, timeout=0.01) as ser:
        ser.write(pyvesc.encode(SetMode))
        ser.write(pyvesc.encode_request(GetValues))
        (response, consumed) = pyvesc.decode(ser.read(78))
        if consumed == 78:
            # print("rpm=",response.rpm)
            # velocity =printSpeed(response.rpm)
            return response.rpm / Trans_Ratio * (2 * 3.14 * Wheel_Radius) / 60 * 100
        else:
            return 'error'


if __name__ == "__main__":
    # SetRPM_Values = 900   
    # SetMode = SetRPM(SetRPM_Values)
    SetDutyCycle_Values = 0.07
    SetMode = SetDutyCycle(SetDutyCycle_Values)

    while True:
        get_values_example(SetMode)
