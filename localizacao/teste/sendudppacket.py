import socket
import random

# IP = '192.168.1.95'
# PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #socket.SOCK_DGRAM means UDP packet

message = b"Hello, World"



sock.sendto(message,('192.168.1.95',9800))

# sock= socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# 
# address=('127.0.0.1', 5006)
# 
# sock.bind(address)
# while True:
#     data,addr=sock.recvfrom(1024)
#     print(data)
    