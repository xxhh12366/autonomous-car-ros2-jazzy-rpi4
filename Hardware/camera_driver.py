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
    """
    摄像头图像发布节点
    
    继承自rclpy.node.Node，实现摄像头图像的采集和发布功能。
    
    主要功能：
    - 初始化摄像头设备
    - 创建图像发布者
    - 定时采集并发布图像
    - 格式转换(OpenCV -> ROS2 Image消息)
    
    属性：
        publisher_: 图像发布者对象
        cap: OpenCV摄像头对象
        bridge: cv_bridge转换器
        timer: 定时器，控制发布频率
    """
    
    def __init__(self):
        """
        初始化摄像头发布节点
        
        初始化步骤：
        1. 调用父类构造函数，设置节点名称
        2. 创建图像发布者(话题: camera/image_raw)
        3. 初始化摄像头(320x240分辨率)
        4. 创建cv_bridge转换器
        5. 创建定时器(20 FPS)
        """
        # 调用父类构造函数，节点名称为 'camera_publisher'
        super().__init__('camera_publisher')
        
        # 创建发布者，话题名为 'camera/image_raw'，队列长度为 10
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # 初始化 OpenCV 摄像头
        self.cap = cv2.VideoCapture(0)  # 0表示默认摄像头
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)   # 设置宽度320像素
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # 设置高度240像素
        
        # 实例化桥接器，用于OpenCV图像与ROS消息的转换
        self.bridge = CvBridge()
        
        # 创建定时器，每 0.05 秒（20 FPS）执行一次回调
        # Timer：以固定频率循环读取并发布图像
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # 打印启动信息
        self.get_logger().info('ROS 2 摄像头节点已启动，正在发布图像...')

    def timer_callback(self):
        """
        定时器回调函数
        
        功能：
        1. 从摄像头读取一帧图像
        2. 将OpenCV格式(numpy数组)转换为ROS2 Image消息
        3. 发布消息到话题
        
        编码格式：
        - 'bgr8': OpenCV默认的BGR颜色空间，8位深度
        
        异常处理：
        - 如果无法读取图像，打印警告信息
        """
        # 从摄像头读取一帧图像
        ret, frame = self.cap.read()
        
        if ret:
            # 将 OpenCV 格式转换为 ROS 2 Image 消息
            # encoding='bgr8': 表示BGR颜色空间，每个通道8位
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            
            # 发布消息到话题
            self.publisher_.publish(msg)
        else:
            # 读取失败，打印警告
            self.get_logger().warn('无法读取摄像头画面')

    def destroy_node(self):
        """
        销毁节点并释放资源
        
        功能：
        1. 释放摄像头资源
        2. 调用父类的销毁方法
        
        注意：
        必须在节点关闭前调用，否则摄像头可能无法正常释放
        """
        # 释放摄像头资源
        self.cap.release()
        
        # 调用父类的销毁方法
        super().destroy_node()


def main(args=None):
    """
    主函数入口
    
    功能流程：
    1. 初始化ROS2客户端库
    2. 创建摄像头发布节点
    3. 进入事件循环，持续发布图像
    4. 捕获中断信号(Ctrl+C)
    5. 清理资源并关闭
    
    参数：
        args: 命令行参数（可选）
    
    使用方法：
        python camera_driver.py
        或
        ros2 run <package_name> camera_driver
    """
    # 初始化ROS2客户端库
    rclpy.init(args=args)
    
    # 创建摄像头节点
    camera_node = CameraPublisher()
    
    try:
        # 进入事件循环，保持节点运行
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        # 用户按下Ctrl+C，正常退出
        pass
    finally:
        # 清理资源
        camera_node.destroy_node()
        rclpy.shutdown()
        print("\n摄像头节点已停止")

if __name__ == '__main__':
    main()                              