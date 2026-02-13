"""
TCP通信客户端模块 / TCP Communication Client Module

用途：与远程TCP服务器建立连接，进行双向数据传输。
Purpose: Establish connection with remote TCP server for bidirectional data transfer.

使用场景 / Use Cases:
- 与上位机通信 / Communicate with host computer
- 远程控制指令接收 / Receive remote control commands
- 传感器数据上传 / Upload sensor data

配置 / Configuration:
- server_ip: 服务器IP地址 / Server IP address (default: 192.168.0.13)
- server_port: 服务器端口号 / Server port number (default: 6000)
- encoding: GBK编码支持中文 / GBK encoding for Chinese characters

注意 / Notes:
- TCP是面向连接的可靠传输协议 / TCP is connection-oriented reliable protocol
- 自动处理数据包的顺序和重传 / Automatically handles packet ordering and retransmission
- 需要先启动服务器端 / Server must be started first
"""

from socket import *

# 创建socket / Create socket
tcp_client_socket = socket(AF_INET, SOCK_STREAM)

# 目的信息 / Destination information
#server_ip = input("请输入服务器ip:")
server_ip ='192.168.0.13'  # 服务器IP地址，根据实际情况修改 / Server IP, modify as needed
#server_port = int(input("请输入服务器port:"))
server_port =6000  # 服务器端口号 / Server port number

# 链接服务器 / Connect to server
tcp_client_socket.connect((server_ip, server_port))
print(f"已连接到服务器 {server_ip}:{server_port} / Connected to server {server_ip}:{server_port}")

while True:
    # 提示用户输入数据 / Prompt user for data input
    send_data = input("请输入要发送的数据：")
    
    # 发送数据，使用GBK编码 / Send data with GBK encoding
    tcp_client_socket.send(send_data.encode("gbk"))

    # 接收对方发送过来的数据，最大接收1024个字节 / Receive data from server, max 1024 bytes
    recvData = tcp_client_socket.recv(1024)
    print('接收到的数据为:', recvData.decode('gbk'))

# 关闭套接字 / Close socket
tcp_client_socket.close()                                    