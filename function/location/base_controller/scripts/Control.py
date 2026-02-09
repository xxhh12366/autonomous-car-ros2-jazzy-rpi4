#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from http.client import OK
from numpy import rate
import rospy
from geometry_msgs.msg import Twist
from scservo_sdk import *                    # Uses SCServo SDK library
import pyvesc
from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
import serial
from base_controller.msg import speed_turn
import math
# Control table address
ADDR_SCS_TORQUE_ENABLE     = 40
ADDR_SCS_GOAL_ACC          = 41
ADDR_SCS_GOAL_POSITION     = 42
ADDR_SCS_GOAL_SPEED        = 46
ADDR_SCS_PRESENT_POSITION  = 56

# Default setting
SCS_ID                      = 1                 # SCServo ID : 1
BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

SCS_MINIMUM_POSITION_VALUE  = 1000         # SCServo will rotate between this value
SCS_MAXIMUM_POSITION_VALUE  = 2000        # and this value (note that the SCServo would not move when the position value is out of movable range. Check e-manual about the range of the SCServo you use.)
SCS_MOVING_STATUS_THRESHOLD = 0          # SCServo moving status threshold
SCS_MOVING_SPEED            = 0           # SCServo moving speed
SCS_MOVING_ACC              = 0           # SCServo moving acc
protocol_end                = 0           # SCServo bit end(STS/SMS=0, SCS=1)

scs_goal_position = 1500  # Goal position

# 初始化端口等项
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(protocol_end)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# 舵机初始化和使能
scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS_ID, ADDR_SCS_TORQUE_ENABLE, 1)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

# 设置舵机速度
scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID, ADDR_SCS_GOAL_SPEED, SCS_MOVING_SPEED)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

def servo_angle_write(target_position):
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID, ADDR_SCS_GOAL_POSITION, target_position)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

def servo_angle_read():
    global scs_present_position_speed
    scs_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, SCS_ID, ADDR_SCS_PRESENT_POSITION)
    if scs_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print(packetHandler.getRxPacketError(scs_error))

    scs_present_position = SCS_LOWORD(scs_present_position_speed)
    return scs_present_position

serialport = '/dev/ttyUSB2'

def print_rpm(rpm):
    """
    打印车速

    :param rpm: response.rpm, 电机转速
    :return:
    """
    velocity = rpm / Trans_Ratio * (2 * 3.14 * Wheel_Radius) / 60
    print("roal_velocity",velocity,'rpm',rpm)


def get_values_example(SetMode):
    
    with serial.Serial(serialport, baudrate=115200, timeout=0.01) as ser:
        ser.write(pyvesc.encode(SetMode))        
        ser.write(pyvesc.encode_request(GetValues))
        (response, consumed) = pyvesc.decode(ser.read(78))
        if consumed == 78:
            print_rpm(response.rpm)
            return response.rpm
        else:
            return 'error'

vel_pub = rospy.Publisher ("speed_turn",speed_turn ,queue_size=10)

save_speed = 0

motor_velcity_max = 0.60   #  m/s
motor_velcity_min = 0.52 
motor_duty_min = 0.40
motor_duty_max = 0.60
servo_position_min = 900
servo_position_max = 2100
Trans_Ratio = 6.287  # 传动比为6.1
Wheel_Radius = 0.032  # 半径为0.032米


def vel_callback(msg):
    speed = msg.linear.x 
    # 占空比  
    if abs(speed) >= motor_duty_max:
        if speed > 0:
            speed = motor_duty_max
        else:
            speed = -motor_duty_max
    if abs(speed) <= motor_duty_min and speed !=0:
        if speed > 0:
            speed = motor_duty_min
        else:
            speed = -motor_duty_min
    SetDutyCycle_Values=speed*0.159
    SetMode = SetDutyCycle(SetDutyCycle_Values)
    print('msg.linear.x=',speed,'goal_velcity=',speed)   
    real_RPM = (get_values_example(SetMode))


    turn = msg.angular.z #rad/s

    if speed != 0:
        delta = math.atan((0.26 * turn) / speed) #rad
    else:
        delta = 0.0  # 设置一个默认值，避免除以零错误

    degrees = -math.degrees(delta) #度
    goal_position = int(20 * degrees + 1500)
    if goal_position > 2000:
        goal_position = 2000
    if goal_position < 1000:
        goal_position = 1000

    servo_angle_write(goal_position)
    real_position = servo_angle_read()
    print('msg.angular.z=', msg.angular.z, 'goal_position=', goal_position, 'real_position=', real_position)

    p = speed_turn()
    p.speed = speed
    p.turn = turn 
    vel_pub.publish(p)

if __name__=='__main__':
    rospy.init_node('base_node')  
    while(1):
        vel_sub = rospy.Subscriber('cmd_vel', Twist, vel_callback, queue_size=10)
        rospy.spin()
