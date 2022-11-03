#!/usr/bin/env python3

# deploy uart-echo example on ESP32 (TXD 17, RXD 18)
# detoverlay=uart3 on raspi

import time
import serial

ser = serial.Serial(
    port='/dev/ttyAMA1',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
        timeout=0)

ser.isOpen()

print('Enter your commands below.\nInsert "exit" to leave the application.\n')

while 1 :
    # get keyboard input
    my_input = str(input())
    if my_input == 'exit':
        ser.close()
        exit()
    else:
        ser.write(my_input.encode())
        out = ''
        # let's wait one second before reading output (let's give device time to answer)
        time.sleep(1)
        while ser.inWaiting() > 0:
            out += ser.read(1).decode()

        if out != '':
            print(">>%s" % out)
