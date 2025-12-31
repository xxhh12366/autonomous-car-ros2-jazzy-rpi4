                                                                 #Python_Nano_MMWR_202211v1.py
# -*- coding: utf-8 -*
import serial
import time
import re
ser = serial.Serial('/dev/ttyUSB0', 115200)
if ser.isOpen == False:
    ser.open()                # 打开串口
ser.write(b"Raspberry pi is ready")
try:
    while True:
        
            response = ser.read(28)        # 读取内容并显示
            print(response)
            response=response.decode('utf-8')
            a=re.findall(r'\d+',response)
        
           
            
            dis=int(a[0],base=10)
            print("距离",dis)
            ser.flushInput()                 # 清空接收缓存区
            time.sleep(0.1)                  # 软件延时
                       
except KeyboardInterrupt:
    ser.close()                                    