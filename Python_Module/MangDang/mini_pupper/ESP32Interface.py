import serial
from cobs import cobsr
from enum import Enum
from dataclasses import dataclass
import threading
import random
import time
import struct


class ParserStates(Enum):
    idle = 0
    waitForCMD = 1
    waitForCI = 2
    waitForLEN = 3
    collectEncodedData = 4


@dataclass
class BipropellantPacket:
    raw: bytearray
    rawDecoded: bytearray
    ACK: int
    CMD: chr
    CI: int
    LEN: int
    code: int
    CS: int


class ParseProtocol:
    """Parse Bipropellant Protocol"""

    def __init__(self):
        self.state = ParserStates.idle
        self.lastPacket = BipropellantPacket(raw=bytearray(), ACK=0, CMD=chr(0), LEN=0, rawDecoded=bytearray(), CI=0, code=0, CS=0)
        self.expectedData = 0
        self.ci = random.randrange(254)
        self.receiveCounter = 0

    def compileMessage(self, cmd, code, data=bytearray(), ACK=0, CI=0):
        if CI == 0:
            self.ci = (self.ci + 1) % 254
        else:
            self.ci = CI - 1

        sendCi = self.ci + 1   # CI only goes from 1 to 255, not 0 to 255.

        newMsg = bytearray()

        newMsg += b'\x00'                            # Start with 0x00
        newMsg.append(((ACK & 1) << 7) | ord(cmd))   # ACK or noACK encoded with cmd
        newMsg.append(sendCi)                            # continuity indicator
        newMsg.append(len(data))
        newMsg.append(code)
        newMsg += data

        # Calculate Checksum
        msgSum = 0
        for x in newMsg:
            msgSum += x

        check = (256 - (msgSum % 256)) % 256
        newMsg.append(check)

        # encode part of MSG. Starting at code
        newMsg = newMsg[:4] + cobsr.encode(newMsg[4:])

        # replace data length with length of encoded part
        newMsg[3] = len(newMsg[4:])

    #    print(newMsg)
        return newMsg

    def parse(self, newCharacter, verbose):
        self.lastPacket.raw.append(ord(newCharacter))

        if newCharacter == b'\x00':
            self.lastPacket = BipropellantPacket(raw=bytearray(), ACK=0, CMD=chr(0), LEN=0, rawDecoded=bytearray(), CI=0, code=0, CS=0)
            self.lastPacket.raw.append(ord(newCharacter))

            self.receiveCounter += 1
            if verbose:
                print('\n' + str(self.receiveCounter), end='< ')
            self.state = ParserStates.waitForCMD

        elif self.state == ParserStates.waitForCMD:
            self.lastPacket.ACK = (ord(newCharacter) & 0b10000000) >> 7
            self.lastPacket.CMD = chr(ord(newCharacter) & 0b01111111)

            if verbose:
                print('ACK:' + str(self.lastPacket.ACK), end=' ')
            if verbose:
                print('CMD:' + str(self.lastPacket.CMD), end=' ')
            self.state = ParserStates.waitForCI

        elif self.state == ParserStates.waitForCI:
            self.lastPacket.CI = ord(newCharacter)

            if verbose:
                print('CI:' + "{:02x}".format(self.lastPacket.CI), end=' ')
            self.state = ParserStates.waitForLEN

        elif self.state == ParserStates.waitForLEN:
            self.lastPacket.LEN = ord(newCharacter)

            self.expectedData = self.lastPacket.LEN
            self.lastPacket.encodedData = bytearray()

            if verbose:
                print('LEN:' + "{:02x}".format(self.lastPacket.LEN), end=' ')
            self.state = ParserStates.collectEncodedData

        elif self.state == ParserStates.collectEncodedData and self.expectedData != 0:
            self.expectedData -= 1
            if self.expectedData == 0:
                self.lastPacket.rawDecoded = self.lastPacket.raw[:4] + cobsr.decode(self.lastPacket.raw[4:])
                self.lastPacket.code = "{:02x}".format(self.lastPacket.rawDecoded[4])
                dataAsInteger = int.from_bytes(self.lastPacket.rawDecoded[5:-1], byteorder='little')

                if self.lastPacket.code == '26':
                    if verbose:
                        print('Text:"' + self.lastPacket.rawDecoded[5:-1].decode("utf-8") + '"', end=' ')
                elif self.lastPacket.code == 'fe':
                    if verbose:
                        print('ProtocolVersion:' + str(dataAsInteger), end=' ')
                elif self.lastPacket.code == '27':
                    if verbose:
                        print('Ping:' + str(int(round(time.time() * 1000)) - dataAsInteger) + 'ms', end=' ')
                else:
                    if verbose:
                        print('Code:' + self.lastPacket.code, end=' ')
                    if self.lastPacket.CMD == 'w':
                        if verbose:
                            print('Write:' + str(dataAsInteger), end=' ')
                    else:
                        if verbose:
                            print(self.lastPacket.rawDecoded[5:-1], end=' ')

                self.lastPacket.CS = "{:02x}".format(self.lastPacket.rawDecoded[-1])
                if verbose:
                    print('CS:' + self.lastPacket.CS, end='\n')

                self.state = ParserStates.idle

        else:
            if verbose:
                print('Unknown Character:', newCharacter)

        return self.lastPacket


class ESP32Interface:
    """ESP32Interface"""

    def __init__(self):
        self.code = 0
        self.err = False
        self.verbose = False
        self.sentCounter = 0
        self.parseProtocol = ParseProtocol()
        port = '/dev/ttyAMA1'
        baudrate = 3000000
        self.ser = serial.Serial(port, baudrate=baudrate)  # open serial port

    def sendFunction(self, data):
        if self.code == 0x27:
            data += int(round(time.time() * 1000)).to_bytes(length=8, byteorder='little', signed=True)
        msg = self.parseProtocol.compileMessage(self.command, self.code, data)
        self.ser.write(msg)
        self.sentCounter += 1

    def receiveFunction(self):
        self.err = False
        while True:
            ret = self.parseProtocol.parse(self.ser.read(), self.verbose)
            if isinstance(ret.code, str) and int(ret.code, base=16) == self.code:
                break
        return ret

    def decodeServoResponse(self, buffer):
        ret = buffer.rawDecoded[5:-1]
        ret_array = []
        for i in range(0, len(ret), 2):
            ret_array.append(int.from_bytes(ret[i:i + 2], 'little'))
        return ret_array

    def executeServoCommand(self, code, command, data=None):
        if data is None:
            data = bytearray()
        self.code = code
        self.command = command
        self.sendFunction(data)
        return self.receiveFunction()

    def servos_enable(self):
        self.executeServoCommand(0x70, 'W')

    def servos_torque_disable(self):
        self.executeServoCommand(0x71, 'W')

    def servos_torque_enable(self):
        self.executeServoCommand(0x72, 'W')

    def servos_disable(self):
        self.executeServoCommand(0x73, 'W')

    def servos_isEnabled(self):
        ret = self.executeServoCommand(0x74, 'R')
        if not self.err:
            ret_val = ret.rawDecoded[5:-1]
            return ret_val[0]
        return False

    def servos_torque_isEnabled(self):
        ret = self.executeServoCommand(0x75, 'R')
        if not self.err:
            ret_val = ret.rawDecoded[5:-1]
            return ret_val[0]
        return False

    def servo_set_position(self, id, pos):
        data = bytearray()
        data += int(id).to_bytes(length=2, byteorder='little', signed=False)
        data += int(pos).to_bytes(length=2, byteorder='little', signed=False)
        self.executeServoCommand(0x76, 'W', data)

    def servos_set_position(self, positions):
        data = bytearray()
        for i in range(12):
            data += int(positions[i]).to_bytes(length=2, byteorder='little', signed=False)
        self.executeServoCommand(0x76, 'W', data)

    def servo_get_position(self):
        ret = self.executeServoCommand(0x77, 'R')
        if not self.err:
            return self.decodeServoResponse(ret)

    def servo_get_feedback(self):
        ret = self.executeServoCommand(0x78, 'R')
        if not self.err:
            return self.decodeServoResponse(ret)

    def servo_get_speed(self):
        ret = self.executeServoCommand(0x79, 'R')
        if not self.err:
            return self.decodeServoResponse(ret)

    def servo_get_load(self):
        ret = self.executeServoCommand(0x7a, 'R')
        if not self.err:
            return self.decodeServoResponse(ret)

    def servo_get_voltage(self):
        ret = self.executeServoCommand(0x7b, 'R')
        if not self.err:
            return self.decodeServoResponse(ret)

    def servo_get_temperature(self):
        ret = self.executeServoCommand(0x7c, 'R')
        if not self.err:
            return self.decodeServoResponse(ret)

    def servo_get_move(self):
        ret = self.executeServoCommand(0x7d, 'R')
        if not self.err:
            return self.decodeServoResponse(ret)

    def servo_get_current(self):
        ret = self.executeServoCommand(0x7e, 'R')
        if not self.err:
            return self.decodeServoResponse(ret)

    def servo_ping(self):
        ret = self.executeServoCommand(0x7f, 'R')
        if not self.err:
            return self.decodeServoResponse(ret)

    def imu_get_6dof(self):
        ret = self.executeServoCommand(0x60, 'R')
        buff = ret.rawDecoded[5:-1]
        ret_dict = {'acc': [], 'gyro': []}
        if not self.err:
            for i in range(0, 12, 4):
                ret_dict['acc'].append(struct.unpack('f', buff[i:i + 4])[0])
            for i in range(12, 24, 4):
                ret_dict['gyro'].append(struct.unpack('f', buff[i:i + 4])[0])
        return ret_dict

    def imu_get_attitude(self):
        ret = self.executeServoCommand(0x61, 'R')
        buff = ret.rawDecoded[5:-1]
        ret_dict = {'dq': [], 'dv': [], 'ae_reg1': 0, 'ae_reg2': 0}
        if not self.err:
            for i in range(0, 16, 4):
                ret_dict['dq'].append(struct.unpack('f', buff[i:i + 4])[0])
            for i in range(16, 28, 4):
                ret_dict['dv'].append(struct.unpack('f', buff[i:i + 4])[0])
            ret_dict['ae_reg1'] = int(buff[29])
            ret_dict['ae_reg1'] = int(buff[30])
        return ret_dict


if __name__ == "__main__":

    esp32 = ESP32Interface()
    esp32.servos_enable()
    esp32.servos_torque_enable()
    positions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    esp32.servos_set_position(positions)
    time.sleep(6)
    start_time = time.time()
    for i in range(1024):
        positions = [i for j in range(12)] 
        esp32.servos_set_position(positions)
    for i in range(1022):
        positions = [(1022 - i) for j in range(12)] 
        esp32.servos_set_position(positions)
    end_time = time.time()
    print("Time: %s microseconds" % ((end_time-start_time) * 1000000))
