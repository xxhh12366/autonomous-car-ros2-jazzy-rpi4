#                                                               #Python_Nano_Camera_202211v1.py
# import cv2
# import numpy

# width = 320
# height = 240
# cap = cv2.VideoCapture(0)  # 调整参数实现读取视频或调用摄像头
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)  # 设置图像宽度
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)  # 设置图像高度
# while True:
#     ret, frame = cap.read()
#     cv2.imshow("cap", frame)      #cv2.imshow 需要一个图形界面环境。Wi-Fi 通过终端操作，树莓派不知道把图片弹窗显示在哪里
#     if cv2.waitKey(100) & 0xff == ord('q'):
#         break
# cap.release()
# cv2.destroyAllWindows()      

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge      #cv_bridge：这是 OpenCV 图像与 ROS 消息之间的“翻译官”

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        # 创建发布者，话题名为 'camera/image_raw'，队列长度为 10
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # 初始化 OpenCV 摄像头
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        # 实例化桥接器
        self.bridge = CvBridge()
        
        # 创建定时器，每 0.05 秒（20 FPS）执行一次回调
        # Timer：以固定频率（如 30 FPS）循环读取并发布
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('ROS 2 摄像头节点已启动，正在发布图像...')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # 将 OpenCV 格式转换为 ROS 2 Image 消息
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            # 发布消息
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('无法读取摄像头画面')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraPublisher()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()                              