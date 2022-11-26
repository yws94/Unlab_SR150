import sys
import socket
import threading

class AppServer():
    def __init__(self, ip, port, p1, p2, q2):
        self.host = ip
        self.port = port
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen()
        self.p1, self.p2, self.q2 = p1, p2, q2
        print("waiting for Client Connection..")
        
    def Connect(self):
        cnt = 0
        while True:
            try:
                client_sock, addr = self.server_sock.accept()
            except KeyboardInterrupt:
                self.server_sock.close()
                print("Exit Program..")
                sys.exit()
            
            if client_sock:
                print('Connected by, ', addr)
                recv = client_sock.recv(1024).decode("utf-8")
                print(recv, type(recv))
                # if recv == 's':
                print('------------------------ START UWB SESSION ------------------------')
                for i in [self.p1, self.p2]:
                    i.start()

                ht = threading.Thread(target = self.handler, args=(client_sock,))
                ht.daemon = True
                ht.start()
                
                while self.q2:
                    cnt+=1
                    # msg = []
                    data = self.q2.get()
                    
                    client_sock.sendall(str(data).encode("utf-8"))
                    
                    # client_sock.sendall(str(data[0]).encode("utf-8"))
                    # client_sock.sendall(str(data[1]).encode("utf-8"))
                    # for i in data:
                    #     msg.append(int(i * 1000).to_bytes(4, byteorder = "little", signed = True))
                    # msg = pickle.dumps(msg)
                    # client_sock.sendall(int(15000).to_bytes(10, byteorder = "little", signed = True))
                    # client_sock.sendall(int(-15000).to_bytes(10, byteorder = "little", signed = True))

                    # client_sock.sendall(msg)
                    # client_sock.sendall(msg[1])
                    
                    # print(msg[0],msg[1])
                    print('send : ', data, cnt)
            
    def handler(self, c):
        while True:
            try:
                data = c.recv(1024)
                msg = data.decode("utf-8")

                if msg == 'f':
                    for i in [self.p1, self.p2]:
                        i.terminate()
                        i.join()
                    c.close()
                    print('------------------------ TERMINATE UWB SESSION ------------------------')
            except:
                c.close()
                break