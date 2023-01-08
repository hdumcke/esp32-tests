import socket
import errno
from struct import pack, unpack


class ESP32Interface:
    """ESP32Interface"""

    def __init__(self):
        self.connect()

    def connect(self):
        # Connect the socket to the port where the server is listening
        server_address = "/tmp/esp32-proxy.socket"
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            self.sock.connect(server_address)
        except Exception as e:
            print("%s" % e)

    def close(self):
        try:
            self.sock.close()
        except Exception as e:
            print("%s" % e)

    def servos_set_position_torque(self, positions, torque):
        try:
            self.sock.sendall(pack("BB12B12H", 38, 1, *torque, *positions))
            data = self.sock.recv(2)
        except Exception as e:
            if e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF:
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return

        if data != pack("BB", 2, 1):
            print("Invalid Ack")
            self.close()
            return

    def servos_set_position(self, positions):
            torque = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
            self.servos_set_position_torque(positions, torque)

    def servos_get_position(self):
        try:
            self.sock.sendall(pack("BB", 2, 2))
            data = self.sock.recv(26)
        except Exception as e:
            if e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF:
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return None

        if data[0:2] != pack("BB", 26, 2):
            print("Invalid Ack")
            self.close()
            return None

        positions = list(unpack("12H", data[2:]))
        return positions

    def servos_get_load(self):
        try:
            self.sock.sendall(pack("BB", 2, 3))
            data = self.sock.recv(26)
        except Exception as e:
            if e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF:
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return None

        if data[0:2] != pack("BB", 26, 3):
            print("Invalid Ack")
            self.close()
            return None

        load = list(unpack("12h", data[2:]))
        return load

    def imu_get_attitude(self):
        try:
            self.sock.sendall(pack("BB", 2, 4))
            data = self.sock.recv(14)
        except Exception as e:
            if e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF:
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return None

        if data[0:2] != pack("BB", 14, 4):
            print("Invalid Ack")
            self.close()
            return None

        raw_attitude = list(unpack("3f", data[2:]))
        attitude = {"roll": raw_attitude[0],
                    "pitch": raw_attitude[1],
                    "yaw": raw_attitude[2]}
        return attitude

    def imu_get_quaternion(self):
        try:
            self.sock.sendall(pack("BB", 2, 5))
            data = self.sock.recv(18)
        except Exception as e:
            if e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF:
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return None

        if data[0:2] != pack("BB", 18, 5):
            print("Invalid Ack")
            self.close()
            return None

        quaternion = list(unpack("4f", data[2:]))
        return quaternion

    def get_power_status(self):
        try:
            self.sock.sendall(pack("BB", 2, 6))
            data = self.sock.recv(10)
        except Exception as e:
            if e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF:
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return None

        if data[0:2] != pack("BB", 10, 6):
            print("Invalid Ack")
            self.close()
            return None

        raw_power = list(unpack("2f", data[2:]))
        power = {"volt": raw_power[0],
                 "ampere": raw_power[1]}
        return power
