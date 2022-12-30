import socket
import sys
from struct import pack, unpack

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
    sock.sendall(pack("BB", 2, 3))
    data = sock.recv(26)
    if data[0:2] != pack("BB", 26, 3):
        print("Invalid Ack")
        sock.close()
        sys.exit(1)
    positions = list(unpack("12h", data[2:]))
    print(positions)
