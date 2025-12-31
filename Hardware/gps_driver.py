                                                                #Python_Nano_GPS_202211v1.py
import serial

def gps_get():
  #创建gps串口的句柄
  ser = serial.Serial("/dev/ttyTHS1", 9600)
 
  print("获取句柄成功，进入循环：")
  c = 0
  while True:
    line = str(str(ser.readline())[2:]) 
    if line.startswith('$GNGGA'):
      print('接收的数据：' + str(line))
      line = str(line).split(',')
      hour=float(line[1][:2])+float(8)
      minute=line[1][2:4]
      second=line[1][4:6]
      longitude = float(line[4][:3]) + float(line[4][3:5])/60
      latitude = float(line[2][:2]) + float(line[2][2:4])/60
      altitude=float(line[9])
      state=float(line[6])
      satellite_nums=float(line[7])
      HDOP=float(line[8])
      elevation_anomaly=float(line[11])
      print("GPS状态指示", state, "---(0-定位模式不可用或无效, 1-GPS SPS模式, 2-定位有效)")
      print("北京时间:",hour,"时",minute,"分",second,"秒")
      print("经度:",longitude,line[5])
      print("纬度:",latitude,line[3])
      print("海拔",altitude,"米")
      print("参与定位的卫星数：",satellite_nums)
      print("高程异常",elevation_anomaly,"米")
      print("HDOP值",HDOP)
      c=c+1
      if(c==5):
        break

if __name__=='__main__':
  gps_get()                                    