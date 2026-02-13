#!/usr/bin/env python
# coding:utf-8

"""
UDP通信客户端模块 / UDP Communication Client Module

用途：使用UDP协议向远程服务器发送数据。
Purpose: Send data to remote server using UDP protocol.

使用场景 / Use Cases:
- 传感器数据快速上传 / Fast sensor data upload
- 实时状态广播 / Real-time status broadcasting
- 非关键数据传输 / Non-critical data transmission

UDP特点 / UDP Characteristics:
- 无连接协议 / Connectionless protocol
- 传输速度快 / Fast transmission
- 不保证数据到达 / No delivery guarantee
- 数据包可能乱序 / Packets may arrive out of order
- 适合实时性要求高的场景 / Suitable for real-time applications

配置 / Configuration:
- addr: 远程服务器地址 / Remote server address (IP, Port)
- socket type: SOCK_DGRAM (UDP)

对比TCP / Compared to TCP:
- UDP更快但不可靠 / UDP is faster but unreliable
- UDP无需建立连接 / UDP doesn't require connection establishment
- UDP适合视频流、传感器数据 / UDP suitable for video stream, sensor data
"""

import socket

# 设置远程服务器地址信息(根据实际地址信息修改)
# Set remote server address (modify according to actual address)
addr=('192.168.0.13',6000)  # (IP地址, 端口号) / (IP address, Port number)

# 创建UDP套接字 / Create UDP socket
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
print(f"UDP客户端已创建，目标服务器: {addr[0]}:{addr[1]} / UDP client created, target: {addr[0]}:{addr[1]}")

while True:
    # 获取用户输入并编码 / Get user input and encode
    data=input('input:').encode()
    if not data:
        break
    
    # 发送数据到指定地址 / Send data to specified address
    s.sendto(data,addr)
    print(f"已发送数据: {data.decode()} / Data sent: {data.decode()}")

# 关闭套接字 / Close socket
s.close()
print("UDP客户端已关闭 / UDP client closed")                            