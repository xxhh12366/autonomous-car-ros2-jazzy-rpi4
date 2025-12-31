                                                              #Python_Nano_Camera_202211v1.py
import cv2
import numpy

width = 320
height = 240
cap = cv2.VideoCapture(0)  # 调整参数实现读取视频或调用摄像头
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)  # 设置图像宽度
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)  # 设置图像高度
while True:
    ret, frame = cap.read()
    cv2.imshow("cap", frame)
    if cv2.waitKey(100) & 0xff == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()                                    