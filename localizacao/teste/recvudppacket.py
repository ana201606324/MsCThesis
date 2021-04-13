import socket
import random

# IP = '192.168.1.95'
# PORT = 5005
# 
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #socket.SOCK_DGRAM means UDP packet
# 
# message = "Hello, World"
# 
# bytes = random._urandom(1024)
# 
# sock.sendto(bytes,('192.168.1.95',5001))

sock= socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

address=('192.168.1.115', 5006)

sock.bind(address)

data,addr=sock.recvfrom(1024)
print(data[4])
print(addr)
    
