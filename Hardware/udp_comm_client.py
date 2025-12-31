#!/usr/bin/env python
# coding:utf-8
import socket

addr=('192.168.0.13',6000)  # 设置远程服务器地址信息(根据实际地址信息修改)
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)  # 创建UDP套接字

while True:
    data=input('input:').encode()
    if not data:
        break
    s.sendto(data,addr)
s.close()                            