                                                                #Python_Nano_GPS_202211v1.py
import serial

def gps_get():
  #创建gps串口的句柄
  ser = serial.Serial("/dev/ttyTHS1", 9600)
    print("GPS串口已打开，开始接收数据...")
    
    c = 0  # 计数器
    
    while True:
        # 读取一行数据并转换为字符串
        line = str(str(ser.readline())[2:])
        
        # 判断是否为GNGGA语句（全球定位系统固定数据）
        if line.startswith('$GNGGA'):
            print('接收的原始数据：' + str(line))
            
            # 按逗号分割数据
            line = str(line).split(',')
            
            # 解析时间信息（UTC时间转北京时间）
            hour = float(line[1][:2]) + float(8)  # UTC+8小时
            minute = line[1][2:4]  # 分钟
            second = line[1][4:6]  # 秒
            
            # 解析经度（度分格式转换为度）
            # 格式：DDDMM.MMMM -> DDD + MM.MMMM/60
            longitude = float(line[4][:3]) + float(line[4][3:5])/60
            
            # 解析纬度（度分格式转换为度）
            # 格式：DDMM.MMMM -> DD + MM.MMMM/60
            latitude = float(line[2][:2]) + float(line[2][2:4])/60
            
            # 解析其他信息
            altitude = float(line[9])           # 海拔高度（米）
            state = float(line[6])              # GPS状态
            satellite_nums = float(line[7])     # 卫星数量
            HDOP = float(line[8])               # 水平精度因子
            elevation_anomaly = float(line[11]) # 高程异常（米）
            
            # 输出解析结果
            print("=" * 60)
            print("GPS状态指示:", state, "(0-无效, 1-GPS有效, 2-差分GPS有效)")
            print("北京时间:", hour, "时", minute, "分", second, "秒")
            print("经度:", longitude, line[5])  # line[5]为E/W方向
            print("纬度:", latitude, line[3])    # line[3]为N/S方向
            print("海拔:", altitude, "米")
            print("参与定位的卫星数：", satellite_nums)
            print("HDOP值（水平精度因子）:", HDOP)
            print("高程异常:", elevation_anomaly, "米")
            print("=" * 60)
            
            # 计数器加1
            c = c + 1
            
            # 读取5次后退出（可根据需要修改）
            if c == 5:
                break


if __name__ == '__main__':
    """
    GPS驱动测试程序
    
    运行此程序可测试GPS模块是否正常工作
    按Ctrl+C可随时退出程序
    """
    try:
        gps_get()
    except KeyboardInterrupt:
        print("\n程序已停止")
    except Exception as e:
        print(f"错误: {e}")                                    