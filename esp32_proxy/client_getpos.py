from MangDang.mini_pupper.ESP32Interface import ESP32Interface

esp32 = ESP32Interface()

while True:
    print(esp32.servos_get_position())
