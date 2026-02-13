#!/bin/bash
#
# 激光雷达一键启动脚本
# 
# 功能说明：
# 该脚本用于快速启动RPLidar A1激光雷达的ROS2节点
# 自动配置环境变量和串口权限
#
# 使用方法：
# chmod +x start_lidar.sh
# ./start_lidar.sh
#
# 注意事项：
# 1. 确保激光雷达已通过USB连接
# 2. 串口设备为 /dev/ttyUSB1
# 3. 需要sudo权限来修改串口权限
#
# 作者: xxhh12366
# 日期: 2023

# 以下为脚本内容（目前已注释）
# 实际使用时请取消注释

# #!/bin/bash
# # 一键启动 A1 激光雷达（全程在 251223 项目内）
# cd ~/xt_ws/src/251223
# source .venv_car/bin/activate
# source /opt/ros/jazzy/setup.bash
# source install/setup.bash
# sudo chmod 666 /dev/ttyUSB1  #这个明天连接后才能看到串口
# ros2 launch rplidar_ros rplidar.launch.py