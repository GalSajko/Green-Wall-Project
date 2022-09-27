import socket

class UdpServer:

    def __init__(self, addressToSend):
        self.LOCAL_IP = '192.168.1.18'
        self.LOCAL_PORT = 20000
        self.REMOTE_PORT = 6565
        
        self.addressToSend = addressToSend
        self.udpServerSocket = socket.socket(family = socket.AF_INET, type = socket.SOCK_DGRAM)
        try:
            self.udpServerSocket.bind((self.LOCAL_IP, self.LOCAL_PORT))
        except socket.error as se:
            print(se)

        print("UDP Server running.")
    
    def send(self, data):
        bytesToSend = bytearray(data.flatten())
        self.udpServerSocket.sendto(bytesToSend, (self.addressToSend, self.REMOTE_PORT))
