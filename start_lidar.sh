#!/bin/bash
# 一键启动 A1 激光雷达（全程在 251223 项目内）
cd ~/xt_ws/src/251223
source .venv_car/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
sudo chmod 666 /dev/ttyUSB0  #这个明天连接后才能看到串口
ros2 launch rplidar_ros rplidar.launch.py