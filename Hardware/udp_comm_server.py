#!/usr/bin/env python
# coding:utf-8
import socket

address=('192.168.0.13',6000)  # 设置本地地址信息(根据实际地址信息修改)
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)  # 创建UDP套接字
s.bind(address)  # 绑定本地地址信息

while True:
    data,addr=s.recvfrom(2048)
    data = data.decode() #解码
    if not data:
        break
    print("got data from",addr)
    print(data)
s.close()                                    