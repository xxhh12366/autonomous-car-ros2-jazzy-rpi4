                                                                #Python_Nano_Motor_202211v2.py
import pyvesc
from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
import serial

serialport = '/dev/ttyUSB2'     #电机-电调串口号

def get_values_example(SetMode):
    """
    发送VESC控制指令并读取电机状态
    
    参数:
        SetMode: VESC控制指令
                 - SetDutyCycle(duty): 占空比控制，duty范围0-1
                 - SetRPM(rpm): 转速控制，rpm为目标转速
    
    返回:
        float: 当前电机转速(RPM)
        str: 'error' - 通信失败
    
    通信流程:
        1. 打开串口连接
        2. 发送控制指令
        3. 请求获取电机状态
        4. 读取并解码响应数据(78字节)
        5. 返回当前转速
    """
    with serial.Serial(serialport, baudrate=115200, timeout=0.01) as ser:
        # 发送控制模式指令（占空比或转速）
        ser.write(pyvesc.encode(SetMode))
        
        # 请求获取电机状态信息
        ser.write(pyvesc.encode_request(GetValues))
        
        # 读取并解码响应数据
        (response, consumed) = pyvesc.decode(ser.read(78))
        
        # 检查是否成功接收完整数据包
        if consumed == 78:
            # 打印当前转速
            print(response.rpm)
            return response.rpm
        else:
            # 通信失败
            return 'error'

if __name__ == "__main__":
    """
    电机驱动测试程序
    
    测试步骤：
    1. 设置目标转速(RPM)或占空比
    2. 创建控制指令
    3. 循环发送指令并读取电机状态
    
    示例：
    - 转速控制: SetRPM(2000) - 设置目标转速为2000 RPM
    - 占空比控制: SetDutyCycle(0.1) - 设置占空比为10%
    """
    # 目标转速 (RPM模式)
    SetRPM_Values = 2000
    
    # 占空比设置 (占空比模式, 范围0-1)
    SetDutyCycle_Values = 0.1
    
    # 选择控制模式
    # 方式1: 占空比控制 (推荐用于调试)
    SetMode = SetDutyCycle(SetDutyCycle_Values)
    
    # 方式2: 转速控制 (需要解注释并注释上一行)
    # SetMode = SetRPM(SetRPM_Values)
    
    print("=" * 50)
    print("电机驱动测试程序")
    print(f"控制模式: 占空比控制")
    print(f"占空比: {SetDutyCycle_Values * 100}%")
    print("按 Ctrl+C 停止")
    print("=" * 50)
    
    # 循环测试
    while True:
        get_values_example(SetMode)                                    