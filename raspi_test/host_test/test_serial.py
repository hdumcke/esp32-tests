import serial
import time


def main():
    ser = serial.Serial('/dev/ttyAMA1',baudrate=3000000,timeout=1)
    print(ser)

    goal_position = [512,512,512,512,512,512,512,512,512,512,512,512]

    while True:
        frame = bytearray()
        frame += b'\xFF'   # Header
        frame += b'\xFF'   # Header
        frame += b'\x01'   # ID
        frame += b'\x1A'   # Length
        frame += b'\x01'   # Instruction
        for pos in goal_position:
            param = pos.to_bytes(2,'little', signed=True)   
            frame += param   # Param
        checksum = 0
        for byte in frame[2:]:
            checksum += byte
        checksum = ~checksum
        checksum_bytes = checksum.to_bytes(1,'little', signed=True)   
        frame += checksum_bytes # CheckSum
        ser.write(frame)
        print("frame sent: "+str((frame)))

        time.sleep(1)
        ack_frame = ser.read(64)
        print("ack: "+str((ack_frame)))

        ack_payload_length = ack_frame[3]
        print("ack_payload_length:"+str(ack_payload_length))

        checksum = 0
        for byte in ack_frame[2:4+ack_payload_length-1]:
            checksum += byte
        checksum = ~checksum
        checksum = checksum % 0xFF
        print("checksum:"+str(hex(checksum)))

if __name__ == "__main__":
    main()
