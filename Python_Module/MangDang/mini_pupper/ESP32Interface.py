import socket
import sys
from struct import pack, unpack


class ESP32Interface:
    """ESP32Interface"""

    def __init__(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        # Connect the socket to the port where the server is listening
        server_address = "/tmp/9Lq7BNBnBycd6nxy.socket"
        try:
            self.sock.connect(server_address)
        except Exception as e:
            print("%s" % e)
            sys.exit(1)

    def servos_set_position(self, positions):
        self.sock.sendall(pack("BB12H", 26, 1, *positions))
        data = self.sock.recv(2)
        if data != pack("BB", 2, 1):
            print("Invalid Ack")
            self.sock.close()
            sys.exit(1)

    def servos_get_position(self):
        self.sock.sendall(pack("BB", 2, 2))
        data = self.sock.recv(26)
        if data[0:2] != pack("BB", 26, 2):
            print("Invalid Ack")
            self.sock.close()
            sys.exit(1)
        positions = list(unpack("12H", data[2:]))
        return positions

    def servos_get_load(self):
        self.sock.sendall(pack("BB", 2, 3))
        data = self.sock.recv(26)
        if data[0:2] != pack("BB", 26, 3):
            print("Invalid Ack")
            self.sock.close()
            sys.exit(1)
        load = list(unpack("12h", data[2:]))
        return load
