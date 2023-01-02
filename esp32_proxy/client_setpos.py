from MangDang.mini_pupper.ESP32Interface import ESP32Interface
import time

positions = [512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512]
# which servos to move: count servos from 1 to 12
servos = [2, 11]

esp32 = ESP32Interface()

delta = 1
while True:
    esp32.servos_set_position(positions)
    for servo in servos:
        positions[servo - 1] += delta
        if positions[servo - 1] >= 1023 or positions[servo - 1] <= 0:
            delta *= -1
        positions[servo - 1] %= 1024
    time.sleep(1 / 500)  # 500 Hz
