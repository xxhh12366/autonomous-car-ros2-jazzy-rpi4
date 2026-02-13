"""
TCP通信服务器模块 / TCP Communication Server Module

用途：创建TCP服务器，等待客户端连接并进行数据交换。
Purpose: Create TCP server, wait for client connections and exchange data.

使用场景 / Use Cases:
- 接收上位机控制指令 / Receive control commands from host computer
- 实时数据流传输 / Real-time data streaming
- 车载传感器数据服务 / Vehicle sensor data service

配置 / Configuration:
- address: 服务器绑定地址 / Server bind address (IP, Port)
- listen backlog: 最大等待连接数 / Maximum pending connections (128)
- encoding: GBK编码支持中文 / GBK encoding for Chinese characters

工作流程 / Workflow:
1. 创建套接字并绑定地址 / Create socket and bind address
2. 监听客户端连接请求 / Listen for client connection requests
3. 接受客户端连接 / Accept client connection
4. 收发数据 / Send and receive data
5. 关闭连接 / Close connection

注意 / Notes:
- 此版本仅支持单客户端连接 / This version supports only single client
- 若需多客户端，需使用多线程 / Multi-threading required for multiple clients
- TCP保证数据按序到达 / TCP guarantees ordered data delivery
"""

from socket import *

# 创建socket / Create socket
tcp_server_socket = socket(AF_INET, SOCK_STREAM)

# 本地信息 / Local binding information
address = ('192.168.0.13', 6000)  # (IP地址, 端口号) / (IP address, Port number)

# 绑定 / Bind to address
tcp_server_socket.bind(address)
print(f"TCP服务器已绑定到 {address[0]}:{address[1]} / TCP server bound to {address[0]}:{address[1]}")

# 使用socket创建的套接字默认的属性是主动的，使用listen将其变为被动的，这样就可以接收别人的链接了
# listen里的数字表征同一时刻能连接客户端的程度
# Convert socket to passive mode to accept connections
# The number in listen() represents max number of pending connections
tcp_server_socket.listen(128)
print("等待客户端连接... / Waiting for client connection...")

# 如果有新的客户端来链接服务器，那么就产生一个新的套接字专门为这个客户端服务
# client_socket用来为这个客户端服务
# tcp_server_socket就可以省下来专门等待其他新客户端的链接
# clientAddr 是元组（ip，端口）
# Accept client connection and create new socket for this client
# client_socket: dedicated socket for this client
# tcp_server_socket: continues to wait for new connections
# clientAddr: tuple (ip, port) of client
client_socket, clientAddr = tcp_server_socket.accept()
print(f"客户端 {clientAddr} 已连接 / Client {clientAddr} connected")


while True:
    # 接收对方发送过来的数据，和udp不同返回的只有数据
    # Receive data from client (unlike UDP, only data is returned, no address)
    recv_data = client_socket.recv(1024)  # 接收1024个字节 / Receive 1024 bytes
    
    if not recv_data:
        print("客户端已断开连接 / Client disconnected")
        break
    
    print('接收到的数据为:', recv_data.decode('gbk'))

    # 发送一些数据到客户端 / Send data to client
    client_socket.send("thank you !".encode('gbk')) 

# 关闭为这个客户端服务的套接字，只要关闭了，就意味着为不能再为这个客户端服务了，如果还需要服务，只能再次重新连接
# Close client socket. Once closed, cannot serve this client anymore unless reconnected
client_socket.close()
tcp_server_socket.close()
print("服务器已关闭 / Server closed")                                    