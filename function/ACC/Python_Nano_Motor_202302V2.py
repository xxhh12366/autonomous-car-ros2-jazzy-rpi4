import pyvesc
from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
import serial

#这里改port
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

#从VESC读取电机转速（RPM），然后计算车轮的线速度（单位：厘米/秒）。
def get_values_example(SetMode):
    with serial.Serial(serialport, baudrate=115200, timeout=0.01) as ser:
        #将 SetMode消息编码后发送给VESC，然后请求VESC返回当前的电机状态（包括转速）。如果成功接收到数据，就计算并返回车轮的线速度；否则返回错误信息。
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
