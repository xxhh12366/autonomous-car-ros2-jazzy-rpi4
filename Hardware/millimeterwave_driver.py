#                                                                  #Python_Nano_MMWR_202211v1.py
# # -*- coding: utf-8 -*
# import serial
# import time
# import re
# ser = serial.Serial('/dev/ttyUSB3', 115200)
# if ser.isOpen == False:
#     ser.open()                # 打开串口
# ser.write(b"Raspberry pi is ready")
# try:
#     while True:
        
#             response = ser.read(28)        # 读取内容并显示
#             print(response)
#             response=response.decode('utf-8')
#             a=re.findall(r'\d+',response)
        
           
            
#             dis=int(a[0],base=10)
#             print("距离",dis)
#             ser.flushInput()                 # 清空接收缓存区
#             time.sleep(0.1)                  # 软件延时
                       
# except KeyboardInterrupt:
#     ser.close()                                    

# -*- coding: utf-8 -*
import serial
import time

# 注意检查串口号，建议使用前面提到的 /dev/serial/by-id/ 里的名称更稳定
ser = serial.Serial('/dev/ttyUSB3', 115200, timeout=1)

if not ser.isOpen():
    ser.open()

print("毫米波雷达解析程序启动...")

try:
    while True:
        # 1. 寻找帧头 (假设帧头是 0x55 0xA5)
        head = ser.read(1)
        if head == b'\x55':
            next_byte = ser.read(1)
            if next_byte == b'\xa5':
                # 2. 读取剩余的数据包内容 (假设总包长 13，已经读了 2 字节，还剩 11)
                data = ser.read(11) 
                
                # 3. 解析距离 (通常距离在特定的字节位，这里示例取 data 的前两个字节)
                # 假设：距离(mm) = 高字节 * 256 + 低字节
                # 请根据说明书修改索引，比如 data[1] 和 data[2]
                try:
                    distance_mm = data[2] * 256 + data[3] 
                    print(f"实时距离: {distance_mm} mm")
                except IndexError:
                    pass
                
                # 4. 清除多余缓存，防止数据堆积导致的延迟
                ser.reset_input_buffer() 
        
        time.sleep(0.05)
                           
except KeyboardInterrupt:
    ser.close()
    print("串口已关闭")