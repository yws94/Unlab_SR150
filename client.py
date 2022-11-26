import socket
import threading
import pickle

if __name__ == "__main__": 
    server_ip = '166.104.46.232' 
    server_port = 8080 
    socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socket.connect((server_ip, server_port))
    msg = input("msg: ")
    socket.sendall(msg.encode("utf-8"))
    while True:
        data = socket.recv(1024)
        data = pickle.loads(data)
        # d = int.from_bytes(data, byteorder = "little", signed = True)
        # print(d.bit_length())
        print("receive :",data)
