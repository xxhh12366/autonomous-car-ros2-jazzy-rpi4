                                                               #Python_Nano_Servo_202211v1.py
import os
from scservo_sdk import *                    # Uses SCServo SDK library

# 控制表地址定义
# SCServo控制表存储了舵机的各项参数和状态
ADDR_SCS_TORQUE_ENABLE     = 40  # 力矩使能地址
ADDR_SCS_GOAL_ACC          = 41  # 目标加速度地址
ADDR_SCS_GOAL_POSITION     = 42  # 目标位置地址
ADDR_SCS_GOAL_SPEED        = 46  # 目标速度地址
ADDR_SCS_PRESENT_POSITION  = 56  # 当前位置地址

# 舵机默认配置参数
SCS_ID                      = 1                 # 舵机ID编号
BAUDRATE                    = 1000000           # 通信波特率
DEVICENAME                  = '/dev/ttyUSB0'    # 舵机控制板串口号

# 舵机运动范围限制
SCS_MINIMUM_POSITION_VALUE  = 500         # 最小位置值（约-90度）
SCS_MAXIMUM_POSITION_VALUE  = 2500        # 最大位置值（约+90度）

# 运动参数配置
SCS_MOVING_STATUS_THRESHOLD = 20          # 运动状态阈值（位置误差容限）
SCS_MOVING_SPEED            = 0           # 运动速度（0表示最大速度）
SCS_MOVING_ACC              = 0           # 运动加速度（0表示最大加速度）
protocol_end                = 0           # 协议类型（STS/SMS=0, SCS=1）

# 目标位置（中间位置）
scs_goal_position = 2000

# 初始化串口和协议处理器
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(protocol_end)

# 打开串口
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# 配置舵机加速度
scs_comm_result, scs_error = packetHandler.write1ByteTxRx(
    portHandler, SCS_ID, ADDR_SCS_GOAL_ACC, SCS_MOVING_ACC
)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

# 配置舵机速度
scs_comm_result, scs_error = packetHandler.write2ByteTxRx(
    portHandler, SCS_ID, ADDR_SCS_GOAL_SPEED, SCS_MOVING_SPEED
)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))


def servo_angle_write(target_position):
    """
    写入舵机目标位置
    
    参数:
        target_position (int): 目标位置值 (500-2500)
                              500: 约-90度
                              1500: 0度(中间位置)
                              2500: 约+90度
    
    返回:
        无返回值，通过串口向舵机发送目标位置指令
        
    异常处理:
        如果通信失败或舵机返回错误，会打印错误信息
    """
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(
        portHandler, SCS_ID, ADDR_SCS_GOAL_POSITION, target_position
    )
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))



def servo_angle_read():
    """
    读取舵机当前位置和速度
    
    返回:
        无返回值，但会更新全局变量 scs_present_position_speed
        并打印当前状态信息
    
    输出信息:
        - 舵机ID
        - 目标位置
        - 当前位置
        - 当前速度
    
    异常处理:
        如果通信失败或舵机返回错误，会打印错误信息
    """
    global scs_present_position_speed
    
    # 读取舵机当前位置和速度（4字节数据）
    scs_present_position_speed, scs_comm_result, scs_error = \
        packetHandler.read4ByteTxRx(portHandler, SCS_ID, ADDR_SCS_PRESENT_POSITION)
    
    if scs_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print(packetHandler.getRxPacketError(scs_error))

    # 从4字节数据中提取位置和速度
    scs_present_position = SCS_LOWORD(scs_present_position_speed)  # 低2字节为位置
    scs_present_speed = SCS_HIWORD(scs_present_position_speed)     # 高2字节为速度
    
    # 打印状态信息
    print("[ID:%03d] GoalPos:%03d PresPos:%03d PresSpd:%03d" 
        % (SCS_ID, scs_goal_position, scs_present_position, 
           SCS_TOHOST(scs_present_speed, 15)))



if __name__ == '__main__':
    """
    舵机驱动测试程序
    
    测试流程：
    1. 向舵机发送目标位置指令
    2. 循环读取舵机当前位置
    3. 当舵机到达目标位置（误差<阈值）时停止
    4. 关闭串口连接
    
    用途：
    - 测试舵机连接是否正常
    - 验证舵机运动是否准确
    - 调试舵机参数
    """
    print("=" * 60)
    print("舵机驱动测试程序")
    print(f"目标位置: {scs_goal_position}")
    print(f"位置范围: {SCS_MINIMUM_POSITION_VALUE} - {SCS_MAXIMUM_POSITION_VALUE}")
    print("=" * 60)
    
    # 发送目标位置指令
    servo_angle_write(scs_goal_position)
    
    # 循环读取当前位置，直到到达目标位置
    while True:
        servo_angle_read()
        # 判断是否到达目标位置（位置误差小于阈值）
        if not (abs(scs_goal_position - scs_present_position_speed) > SCS_MOVING_STATUS_THRESHOLD):
            print("舵机已到达目标位置")
            break
    
    # 关闭串口
    portHandler.closePort()
    print("测试完成")                                    