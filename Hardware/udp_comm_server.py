#!/usr/bin/env python
# coding:utf-8

"""
UDP通信服务器模块 / UDP Communication Server Module

用途：创建UDP服务器，接收来自客户端的数据包。
Purpose: Create UDP server to receive data packets from clients.

使用场景 / Use Cases:
- 接收多个传感器的数据 / Receive data from multiple sensors
- 实时状态监控 / Real-time status monitoring
- 高频数据采集 / High-frequency data collection

UDP特点 / UDP Characteristics:
- 无连接服务 / Connectionless service
- 支持多客户端同时通信 / Supports multiple clients simultaneously
- 接收速度快 / Fast reception
- 可能丢包 / Packet loss possible
- 接收数据包时同时获得发送者地址 / Receives sender address with data

配置 / Configuration:
- address: 本地绑定地址 / Local bind address (IP, Port)
- buffer size: 2048字节 / 2048 bytes
- socket type: SOCK_DGRAM (UDP)

工作流程 / Workflow:
1. 创建UDP套接字 / Create UDP socket
2. 绑定本地地址和端口 / Bind to local address and port
3. 循环接收数据 / Loop receiving data
4. 处理数据和发送者信息 / Process data and sender info

注意 / Notes:
- UDP不保证数据到达顺序 / UDP doesn't guarantee packet order
- 无需accept()，直接接收 / No need for accept(), receive directly
- 可同时服务多个客户端 / Can serve multiple clients simultaneously
"""

import socket

# 设置本地地址信息(根据实际地址信息修改)
# Set local address information (modify according to actual address)
address=('192.168.0.13',6000)  # (IP地址, 端口号) / (IP address, Port number)

# 创建UDP套接字 / Create UDP socket
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

# 绑定本地地址信息 / Bind to local address
s.bind(address)
print(f"UDP服务器已绑定到 {address[0]}:{address[1]} / UDP server bound to {address[0]}:{address[1]}")
print("等待接收数据... / Waiting for data...")

while True:
    # 接收数据，返回数据和发送者地址 / Receive data and sender address
    # data: 接收到的数据 / received data
    # addr: 发送者地址(IP, Port) / sender address (IP, Port)
    data,addr=s.recvfrom(2048)  # 最大接收2048字节 / Max 2048 bytes
    
    # 解码 / Decode
    data = data.decode()
    
    if not data:
        break
    
    # 显示接收到的数据和来源 / Display received data and source
    print("got data from",addr)
    print(data)

# 关闭套接字 / Close socket
s.close()
print("UDP服务器已关闭 / UDP server closed")                                    