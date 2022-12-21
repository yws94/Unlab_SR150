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
            except OSError:
                print("Exit Program..")
                sys.exit()

            if client_sock:
                print('Connected by, ', addr)

                for i in [self.p1, self.p2]:
                    i.start()

                print('------------------------ START UWB SESSION ------------------------')

                ht = threading.Thread(target = self.handler, args=(client_sock,))
                ht.daemon = True
                ht.start()

                while self.q2:
                    cnt += 1
                    data = ''.join(str(self.q2.get()))
                    client_sock.sendall(data.encode())
                    print('send : ', data, cnt)

    def handler(self, client):
        given_string = ""
        while True:
            try:
                data = client.recv(1024)
                if data :
                    given_string = data.decode("utf-8", errors = "ignore")
                    if "f" in given_string:
                        for i in [self.p1, self.p2]:
                            i.terminate()
                            i.join()
                        client.close()
                        self.server_sock.close()
                        print('------------------------ TERMINATE UWB SESSION ------------------------')
                        sys.exit()
            except:
                client.close()
                self.server_sock.close()
                break