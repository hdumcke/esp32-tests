from MangDang.mini_pupper.ESP32Interface import ESP32Interface
import time

positions = [512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512]

esp32 = ESP32Interface()

delta = 1
while True:
    esp32.servos_set_position(positions)
    positions[2] += delta
    if positions[2] >= 1023 or positions[2] <= 0:
        delta *= -1
    positions[2] %= 1024
    time.sleep(1 / 500)  # 500 Hz
