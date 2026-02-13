"""
毫米波雷达驱动模块 / Millimeter-Wave Radar Driver Module

用途：解析毫米波雷达数据，获取前方目标的距离和速度信息。
Purpose: Parse millimeter-wave radar data to get distance and speed of targets ahead.

硬件接口 / Hardware Interface:
- 串口: /dev/ttyUSB3 或 /dev/ttyTHS1
- 波特率: 115200
- 数据格式: 自定义二进制协议 / Custom binary protocol

数据帧格式 / Data Frame Format:
- 帧头: 0x55 0xA5 (2 bytes) / Frame header
- 数据长度: 11 bytes / Data length
- 距离数据: data[2:4] (高位+低位，单位：毫米) / Distance (high+low byte, unit: mm)
- 速度数据: 可能在 data[4:6] / Speed data (possibly at data[4:6])

应用场景 / Application Scenarios:
- ACC自适应巡航控制 / Adaptive Cruise Control
- AEB自动紧急制动 / Automatic Emergency Braking
- PCS泊车碰撞预警 / Parking Collision System
- 前方车辆检测 / Forward vehicle detection

注意事项 / Notes:
- 确保串口权限: sudo chmod 666 /dev/ttyUSB3
- 数据包格式可能因雷达型号不同而异，需根据说明书调整
- 建议使用 /dev/serial/by-id/ 下的名称以保证设备稳定识别
- 数据帧可能包含多个目标，此版本仅解析第一个目标

使用示例 / Usage Example:
    import millimeterwave_driver as MMW
    
    port = MMW.openMMWPort("/dev/ttyTHS1", 115200)
    while True:
        distance, speed = MMW.MMWDetection(port)
        if distance > 0:
            print(f"Distance: {distance} cm, Speed: {speed} cm/s")
"""

# -*- coding: utf-8 -*
import serial
import time


def openMMWPort(port_name, baud_rate=115200, timeout=1):
    """
    打开毫米波雷达串口 / Open millimeter-wave radar serial port
    
    Args:
        port_name (str): 串口设备名 / Serial port device name
                         例如: '/dev/ttyUSB3' 或 '/dev/ttyTHS1'
        baud_rate (int): 波特率 / Baud rate (default: 115200)
        timeout (float): 超时时间（秒）/ Timeout in seconds (default: 1)
    
    Returns:
        serial.Serial: 串口对象 / Serial port object
    
    Raises:
        serial.SerialException: 串口打开失败 / Failed to open serial port
    """
    try:
        ser = serial.Serial(port_name, baud_rate, timeout=timeout)
        if not ser.isOpen():
            ser.open()
        print(f"毫米波雷达串口已打开: {port_name} @ {baud_rate} / MMW radar port opened: {port_name} @ {baud_rate}")
        return ser
    except serial.SerialException as e:
        print(f"打开串口失败 / Failed to open port: {e}")
        raise


def MMWDetection(ser):
    """
    从毫米波雷达读取并解析一次测量数据 / Read and parse one measurement from MMW radar
    
    数据帧结构 / Frame Structure:
        Byte 0-1: 帧头 0x55 0xA5 / Frame header
        Byte 2-3: 距离数据（高位在前）/ Distance (high byte first), unit: mm
        Byte 4-5: 速度数据（可选）/ Speed data (optional)
        Byte 6-12: 其他数据 / Other data
    
    Args:
        ser (serial.Serial): 已打开的串口对象 / Opened serial port object
    
    Returns:
        tuple: (distance_cm, speed_cm_s)
               distance_cm (float): 距离（厘米）/ Distance in centimeters
                                    返回0表示未检测到目标 / Returns 0 if no target detected
               speed_cm_s (float): 速度（厘米/秒）/ Speed in cm/s
                                   正值表示接近，负值表示远离 / Positive: approaching, Negative: receding
    
    注意 / Notes:
        - 此函数是阻塞调用，会等待完整数据帧 / Blocking call, waits for complete frame
        - 推荐调用频率: 10-20Hz / Recommended call rate: 10-20Hz
        - 如果数据包损坏，返回(0, 0) / Returns (0, 0) if packet is corrupted
    """
    try:
        # 1. 寻找帧头 (0x55 0xA5) / Search for frame header (0x55 0xA5)
        while True:
            head = ser.read(1)
            if head == b'\x55':
                next_byte = ser.read(1)
                if next_byte == b'\xa5':
                    break
        
        # 2. 读取剩余的数据包内容 (总包长13字节，已读2字节，还剩11字节)
        # Read remaining packet content (13 bytes total, 2 read, 11 remaining)
        data = ser.read(11)
        
        if len(data) < 11:
            print("警告：数据包不完整 / Warning: Incomplete packet")
            ser.reset_input_buffer()
            return 0, 0
        
        # 3. 解析距离 (距离通常在data[2:4]，大端序) / Parse distance (usually at data[2:4], big-endian)
        # 距离(mm) = 高字节 * 256 + 低字节 / Distance(mm) = high_byte * 256 + low_byte
        distance_mm = data[2] * 256 + data[3]
        distance_cm = distance_mm / 10.0  # 转换为厘米 / Convert to centimeters
        
        # 4. 解析速度 (如果有速度数据，通常在data[4:6])
        # Parse speed (if available, usually at data[4:6])
        # 注意：速度数据格式需根据具体雷达型号确定
        # Note: Speed data format depends on specific radar model
        speed_cm_s = 0  # 默认速度为0 / Default speed is 0
        # 如果雷达支持速度测量，取消下面的注释并调整 / If radar supports speed, uncomment and adjust below
        # speed_cm_s = (data[4] * 256 + data[5]) / 10.0  # 示例：转换为cm/s / Example: convert to cm/s
        
        # 5. 清除缓冲区，防止数据堆积 / Clear buffer to prevent data accumulation
        ser.reset_input_buffer()
        
        return distance_cm, speed_cm_s
        
    except (IndexError, TypeError) as e:
        print(f"数据解析错误 / Data parsing error: {e}")
        ser.reset_input_buffer()
        return 0, 0
    except serial.SerialException as e:
        print(f"串口通信错误 / Serial communication error: {e}")
        return 0, 0


# 主程序示例 / Main program example
if __name__ == "__main__":
    # 注意检查串口号，建议使用 /dev/serial/by-id/ 里的名称更稳定
    # Check serial port name, using /dev/serial/by-id/ names is more stable
    PORT = '/dev/ttyUSB3'  # 根据实际情况修改 / Modify according to actual situation
    BAUD_RATE = 115200
    
    try:
        # 打开串口 / Open serial port
        ser = openMMWPort(PORT, BAUD_RATE)
        print("毫米波雷达解析程序启动... / MMW radar parser started...")
        
        while True:
            # 获取距离和速度数据 / Get distance and speed data
            distance, speed = MMWDetection(ser)
            
            if distance > 0:
                print(f"实时距离: {distance:.1f} cm / Real-time distance: {distance:.1f} cm")
                if speed != 0:
                    print(f"相对速度: {speed:.1f} cm/s / Relative speed: {speed:.1f} cm/s")
            else:
                print("未检测到目标 / No target detected")
            
            time.sleep(0.05)  # 20Hz 采样率 / 20Hz sampling rate
                               
    except KeyboardInterrupt:
        print("\n程序终止 / Program terminated")
    except Exception as e:
        print(f"错误 / Error: {e}")
    finally:
        if 'ser' in locals() and ser.isOpen():
            ser.close()
            print("串口已关闭 / Serial port closed")