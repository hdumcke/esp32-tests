import serial
from cobs import cobsr
import argparse
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

    def __init__(self, code):
        self.pos = 0
        self.pos_step = 1
        self.code = code
        self.sentCounter = 0

    def periodicFunctions(self):
        data = bytearray()
        if self.code == 0x27:
            data += int(round(time.time() * 1000)).to_bytes(length=8, byteorder='little', signed=True)
        self.only_once = False
        self.command = 'R'
        # enable
        if(self.code == 0x70):
            self.only_once = True
            self.command = 'W'
        # disable
        if(self.code == 0x71):
            self.only_once = True
            self.command = 'W'
        # isEnabled
        if(self.code == 0x72):
            self.only_once = True
        # set
        if(self.code == 0x73):
            self.only_once = False
            self.command = 'W'
            for i in range(12):
                data += int(self.pos).to_bytes(length=2, byteorder='little', signed=False)
            self.pos += self.pos_step
            if(self.pos > 1022):
                self.pos = 1023
                self.pos_step = -self.pos_step
            if(self.pos < 1):
                self.pos = 0
                self.pos_step = -self.pos_step
            self.pos = self.pos % 1024
        # 0x74, "servo get get position"
        if(self.code == 0x74):
            self.only_once = True
        # 0x75, "servo get get feedback"
        if(self.code == 0x75):
            self.only_once = False
        # 0x76, "servo get get speed"
        if(self.code == 0x76):
            self.only_once = True
        # 0x77, "servo get get load"
        if(self.code == 0x77):
            self.only_once = True
        # 0x78, "servo get get voltage"
        if(self.code == 0x78):
            self.only_once = True
        # 0x79, "servo get get temper"
        if(self.code == 0x79):
            self.only_once = True
        # 0x7A, "servo get get move"
        if(self.code == 0x7A):
            self.only_once = True
        # 0x7B, "servo get get current"
        if(self.code == 0x7B):
            self.only_once = True
        # 0x7C, "servo ping"
        if(self.code == 0x7C):
            self.only_once = True
        # 0x7E, "imu read attitude",
        if(self.code == 0x7E):
            self.only_once = True
        msg = parseProtocol.compileMessage(self.command, self.code, data)
        ser.write(msg)
        if(self.only_once):
            return
        self.sentCounter += 1
        # print('\n'+str(self.sentCounter),end='> ')
        # print("%s" % binascii.hexlify(msg))
        threading.Timer(0.01, self.periodicFunctions).start()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    port = '/dev/ttyAMA1'
    baudrate = 3000000

    verbose = True
    command = "0x0D"

    ser = serial.Serial(port, baudrate=baudrate)  # open serial port

    parseProtocol = ParseProtocol()

    periodic = ESP32Interface(int(command, base=16))
    periodic.periodicFunctions()

    while(1):
        receivedPacket = parseProtocol.parse(ser.read(), verbose)
        if receivedPacket.code == '70' or \
           receivedPacket.code == '71' or \
           receivedPacket.code == '72' or \
           receivedPacket.code == '73':
            ret = receivedPacket.rawDecoded[5:-1]
            print("%s %s" % (receivedPacket.CI, ret))
        if receivedPacket.code == '74' or \
           receivedPacket.code == '75' or \
           receivedPacket.code == '76' or \
           receivedPacket.code == '77' or \
           receivedPacket.code == '78' or \
           receivedPacket.code == '79' or \
           receivedPacket.code == '7a' or \
           receivedPacket.code == '7b' or \
           receivedPacket.code == '7c':
            ret = receivedPacket.rawDecoded[5:-1]
            ret_array = []
            for i in range(0, len(ret), 2):
                ret_array.append(int.from_bytes(ret[i:i + 2], 'little'))
            print("%s %s" % (receivedPacket.CI, ret_array))
        if receivedPacket.code == '7d':
            ret = receivedPacket.rawDecoded[5:-1]
            ret_dict = {'acc': [], 'gyro': []}
            for i in range(0, 12, 4):
                ret_dict['acc'].append(struct.unpack('f', ret[i:i + 4])[0])
            for i in range(12, 24, 4):
                ret_dict['gyro'].append(struct.unpack('f', ret[i:i + 4])[0])
            print("%s %s" % (receivedPacket.CI, ret_dict))
        if receivedPacket.code == '7e':
            ret = receivedPacket.rawDecoded[5:-1]
            ret_dict = {'dq': [], 'dv': [], 'ae_reg1': 0, 'ae_reg2': 0}
            for i in range(0, 16, 4):
                ret_dict['dq'].append(struct.unpack('f', ret[i:i + 4])[0])
            for i in range(16, 28, 4):
                ret_dict['dv'].append(struct.unpack('f', ret[i:i + 4])[0])
            ret_dict['ae_reg1'] = int(ret[29])
            ret_dict['ae_reg1'] = int(ret[30])
            print("%s %s" % (receivedPacket.CI, ret_dict))
