"""
自动驾驶车辆主程序 / Autonomous Car Main Program

项目名称 / Project Name: 
    Autonomous Car - ROS2 Jazzy on Raspberry Pi 4

功能概述 / Overview:
    这是一个基于树莓派4和ROS2 Jazzy构建的L2级自动驾驶车辆平台主程序。
    本文件作为系统的主入口点，用于启动和协调各个自动驾驶功能模块。
    
    This is the main program for a Level 2 autonomous vehicle platform 
    built on Raspberry Pi 4 with ROS2 Jazzy. It serves as the system entry 
    point to launch and coordinate various autonomous driving modules.

系统架构 / System Architecture:
    ┌─────────────────────────────────────────────┐
    │            应用层 / Application              │
    │  ACC | AEB | LKS | PCS | APS | Zebra ...   │
    ├─────────────────────────────────────────────┤
    │           控制层 / Control Layer            │
    │        PID Controller | Path Planning       │
    ├─────────────────────────────────────────────┤
    │        硬件驱动层 / Hardware Drivers         │
    │  Motor | Servo | Camera | Sensors | ...     │
    └─────────────────────────────────────────────┘

可用功能模块 / Available Feature Modules:
    1. ACC (Adaptive Cruise Control) - 自适应巡航控制
       路径: function/Python_Nano_ACC_202302V2.py
       
    2. AEB (Automatic Emergency Braking) - 自动紧急制动
       路径: function/Python_Nano_AEB_202302V2.py
       
    3. LKS (Lane Keeping System) - 车道保持系统
       路径: function/LKS_Hough/Python_Nano_LKS_202302V1.py
       
    4. PCS (Parking Collision System) - 泊车碰撞系统
       路径: function/PCS/Python_Nano_PCS_202302v1.py
       
    5. APS (Automated Parking System) - 自动泊车系统
       路径: function/APS/Python_Nano_APS_202304v2.py
       
    6. Zebra Crossing Detection - 斑马线检测
       路径: function/Zebra/Python_Nano_ZebraBrake_202302V1.py
       
    7. License Plate Recognition - 车牌识别
       路径: function/License_Plate/Python_Nano_License_plate_202307v1.py
       
    8. Traffic Light Detection - 交通信号灯检测
       路径: function/traffic_lights/

使用方法 / Usage:

    方式1: 直接运行单个功能模块 / Method 1: Run individual module directly
    ---------------------------------------------------------------
    cd function
    python3 Python_Nano_ACC_202302V2.py      # 运行ACC
    python3 Python_Nano_AEB_202302V2.py      # 运行AEB
    
    cd function/LKS_Hough
    python3 Python_Nano_LKS_202302V1.py      # 运行车道保持
    
    cd function/APS
    python3 Python_Nano_APS_202304v2.py      # 运行自动泊车

    方式2: 使用本主程序启动 / Method 2: Launch via main program
    ---------------------------------------------------------------
    python3 main.py --mode acc                # 启动ACC模式
    python3 main.py --mode aeb                # 启动AEB模式
    python3 main.py --mode lks                # 启动车道保持模式
    python3 main.py --mode aps                # 启动自动泊车模式
    python3 main.py --mode pcs                # 启动泊车碰撞系统
    
    方式3: 启动ROS2节点 / Method 3: Launch ROS2 nodes
    ---------------------------------------------------------------
    python3 Hardware/camera_driver.py         # 启动摄像头节点
    ./start_lidar.sh                          # 启动激光雷达节点

硬件要求 / Hardware Requirements:
    - 树莓派4 (4GB+ RAM) / Raspberry Pi 4 (4GB+ RAM)
    - VESC电机控制器 / VESC motor controller
    - SCServo舵机 / SCServo for steering
    - 摄像头 / Camera
    - 激光雷达 / LIDAR (RPLidar)
    - IMU (9轴) / IMU (9-axis)
    - GPS / GPS
    - 毫米波雷达 / Millimeter-wave radar
    - 超声波传感器 / Ultrasonic sensors

软件依赖 / Software Dependencies:
    - Ubuntu 22.04
    - ROS2 Jazzy
    - Python 3.10+
    - OpenCV
    - NumPy
    - PySerial
    - simple-pid
    - ultralytics (YOLO)

配置说明 / Configuration:
    1. 确保所有硬件正确连接 / Ensure all hardware is properly connected
    2. 检查串口权限 / Check serial port permissions:
       sudo usermod -a -G dialout $USER
       sudo chmod 666 /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS*
    3. 设置ROS2环境 / Set up ROS2 environment:
       source /opt/ros/jazzy/setup.bash
    4. 根据实际情况修改设备路径 / Modify device paths as needed:
       - 电机 Motor: /dev/motor
       - 舵机 Servo: /dev/ttyUSB0
       - 毫米波雷达 MMW: /dev/ttyTHS1

开发者 / Developer:
    xxhh12366

版本 / Version:
    202302V2 - 2026年2月

许可证 / License:
    [待添加 / To be added]

更多信息 / More Information:
    详见 README.md 文件 / See README.md for more details
"""

import sys
import argparse


def print_banner():
    """打印欢迎横幅 / Print welcome banner"""
    banner = """
    ╔═══════════════════════════════════════════════════════════╗
    ║                                                           ║
    ║     自动驾驶车辆控制系统 / Autonomous Car System          ║
    ║                 ROS2 Jazzy on Raspberry Pi 4              ║
    ║                                                           ║
    ╚═══════════════════════════════════════════════════════════╝
    """
    print(banner)


def main():
    """
    主函数 / Main function
    
    解析命令行参数并启动相应的功能模块。
    Parse command line arguments and launch corresponding feature modules.
    """
    print_banner()
    
    # 创建参数解析器 / Create argument parser
    parser = argparse.ArgumentParser(
        description='自动驾驶车辆主程序 / Autonomous Car Main Program',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例 / Examples:
  python3 main.py --mode acc          # 启动自适应巡航控制 / Launch ACC
  python3 main.py --mode aeb          # 启动自动紧急制动 / Launch AEB
  python3 main.py --mode lks          # 启动车道保持系统 / Launch LKS
  python3 main.py --mode aps          # 启动自动泊车系统 / Launch APS
  python3 main.py --mode pcs          # 启动泊车碰撞系统 / Launch PCS
  python3 main.py --list              # 列出所有可用模式 / List all modes

注意：部分功能需要先启动ROS2节点（摄像头、激光雷达等）
Note: Some features require ROS2 nodes (camera, lidar, etc.) to be started first
        """
    )
    
    # 添加命令行参数 / Add command line arguments
    parser.add_argument(
        '--mode', '-m',
        type=str,
        choices=['acc', 'aeb', 'lks', 'aps', 'pcs', 'zebra', 'license_plate'],
        help='选择运行模式 / Select running mode'
    )
    
    parser.add_argument(
        '--list', '-l',
        action='store_true',
        help='列出所有可用的功能模块 / List all available feature modules'
    )
    
    # 解析参数 / Parse arguments
    args = parser.parse_args()
    
    # 如果使用--list参数，显示所有可用模式 / If --list is used, show all available modes
    if args.list:
        print("\n可用的功能模块 / Available Feature Modules:\n")
        print("  acc            - 自适应巡航控制 / Adaptive Cruise Control")
        print("  aeb            - 自动紧急制动 / Automatic Emergency Braking")
        print("  lks            - 车道保持系统 / Lane Keeping System")
        print("  aps            - 自动泊车系统 / Automated Parking System")
        print("  pcs            - 泊车碰撞系统 / Parking Collision System")
        print("  zebra          - 斑马线检测 / Zebra Crossing Detection")
        print("  license_plate  - 车牌识别 / License Plate Recognition")
        print()
        return
    
    # 如果没有指定模式，显示帮助信息 / If no mode specified, show help
    if not args.mode:
        parser.print_help()
        print("\n提示：使用 --list 查看所有可用模式")
        print("Tip: Use --list to see all available modes\n")
        return
    
    # 根据模式启动相应的功能 / Launch feature based on mode
    print(f"\n正在启动 {args.mode.upper()} 模式... / Launching {args.mode.upper()} mode...\n")
    
    try:
        if args.mode == 'acc':
            print("启动自适应巡航控制 / Launching Adaptive Cruise Control")
            print("模块路径 / Module path: function/Python_Nano_ACC_202302V2.py")
            print("\n请直接运行: python3 function/Python_Nano_ACC_202302V2.py")
            print("Please run directly: python3 function/Python_Nano_ACC_202302V2.py")
            
        elif args.mode == 'aeb':
            print("启动自动紧急制动 / Launching Automatic Emergency Braking")
            print("模块路径 / Module path: function/Python_Nano_AEB_202302V2.py")
            print("\n请直接运行: python3 function/Python_Nano_AEB_202302V2.py")
            print("Please run directly: python3 function/Python_Nano_AEB_202302V2.py")
            
        elif args.mode == 'lks':
            print("启动车道保持系统 / Launching Lane Keeping System")
            print("模块路径 / Module path: function/LKS_Hough/Python_Nano_LKS_202302V1.py")
            print("\n请直接运行: python3 function/LKS_Hough/Python_Nano_LKS_202302V1.py")
            print("Please run directly: python3 function/LKS_Hough/Python_Nano_LKS_202302V1.py")
            
        elif args.mode == 'aps':
            print("启动自动泊车系统 / Launching Automated Parking System")
            print("模块路径 / Module path: function/APS/Python_Nano_APS_202304v2.py")
            print("\n请直接运行: python3 function/APS/Python_Nano_APS_202304v2.py")
            print("Please run directly: python3 function/APS/Python_Nano_APS_202304v2.py")
            
        elif args.mode == 'pcs':
            print("启动泊车碰撞系统 / Launching Parking Collision System")
            print("模块路径 / Module path: function/PCS/Python_Nano_PCS_202302v1.py")
            print("\n请直接运行: python3 function/PCS/Python_Nano_PCS_202302v1.py")
            print("Please run directly: python3 function/PCS/Python_Nano_PCS_202302v1.py")
            
        elif args.mode == 'zebra':
            print("启动斑马线检测 / Launching Zebra Crossing Detection")
            print("模块路径 / Module path: function/Zebra/Python_Nano_ZebraBrake_202302V1.py")
            print("\n请直接运行: python3 function/Zebra/Python_Nano_ZebraBrake_202302V1.py")
            print("Please run directly: python3 function/Zebra/Python_Nano_ZebraBrake_202302V1.py")
            
        elif args.mode == 'license_plate':
            print("启动车牌识别 / Launching License Plate Recognition")
            print("模块路径 / Module path: function/License_Plate/Python_Nano_License_plate_202307v1.py")
            print("\n请直接运行: python3 function/License_Plate/Python_Nano_License_plate_202307v1.py")
            print("Please run directly: python3 function/License_Plate/Python_Nano_License_plate_202307v1.py")
        
        print("\n注意事项 / Notes:")
        print("1. 确保硬件已正确连接 / Ensure hardware is properly connected")
        print("2. 检查串口权限 / Check serial port permissions")
        print("3. 必要时启动ROS2节点（摄像头、激光雷达）/ Start ROS2 nodes if needed (camera, lidar)")
        print()
        
    except KeyboardInterrupt:
        print("\n\n程序被用户中断 / Program interrupted by user")
    except Exception as e:
        print(f"\n错误 / Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
