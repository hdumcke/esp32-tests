import socket
import sys
from struct import pack

positions = [512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512]

# Create a socket
sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)

# Connect the socket to the port where the server is listening
server_address = "/tmp/9Lq7BNBnBycd6nxy.socket"
try:
    sock.connect(server_address)
except Exception as e:
    print("%s" % e)
    sys.exit(1)

while True:
    sock.sendall(pack("BB12H", 26, 1, *positions))
    data = sock.recv(2)
    if data != pack("BB", 2, 1):
        print("Invalid Ack")
        sock.close()
        sys.exit(1)
    positions[0] += 1
    positions[0] %= 1024
