import socket

import socket

server_ip = '166.104.46.232' 
server_port = 8080 

socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket.connect((server_ip, server_port))
while True :
    
    msg = input('msg:') 
    socket.sendall(msg.encode(encoding='utf-8'))


# data = socket.recv(100)
# msg = data.decode() 
# print('echo msg:', msg)
