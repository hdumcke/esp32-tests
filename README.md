# esp32-tests

## Status

This is work in progress and should be of no interest to anyone other than people I work with directly

## ESP32

### Install esp-idf on Raspberry Pi

You should use a dedicated SD card and not try to install esp-idf at the same card with mini pupper.

Boot Raspberry Pi with Ubuntu 22.04 and then open a terminal on Raspberry Pi.

- git clone https://github.com/hdumcke/multipass-orchestrator-configurations.git
- cd /multipass-orchestrator-configurations/esp-idf
- ./build.sh
- sudo reboot

### Compile esp-idf code and flash ESP32

In the environment you prepared above:

- git clone https://github.com/hdumcke/esp32-tests
- cd esp32-tests/esp32
- idf.py set-target esp32s3
- idf.py menuconfig # optional
- idf.py build
- idf.py flash

### Install Raspberry Pi

Prepare a SD card with Ubuntu 22.04 server. Boot Raspberry Pi and then open a terminal:

- git clone https://github.com/hdumcke/esp32-tests
- cd esp32-tests/
- ./setup.sh &lt;my SSID&gt; &lt;my wifi password&gt;

At the end the Raspberry PI will reboot and display the IP address on the LCD. You can now control mini pupper with your PS4 joystick or with a web browser at:

http://&lt;mini pupper IP&gt;:8080

#### Fully automated installation with cloud-init

Flash your SD card with ubuntu-22.04.1-preinstalled-server-arm64+raspi.img.xz

On the same PC where you flashed your SD card:

git clone https://github.com/hdumcke/mini_pupper_testig

And then execute 

./mini_pupper_testig/prepare_sd.py

And answer the questions. At the end eject your SD card, stick it into Mini Pupper, boot Mini Pupper and wait until the IP address is shown on the LCD.

### Calibration

Calbration is done on the ESP32. We assume that the final product ships with a cable that allows to connect the USB port on the Raspberry Pi to the ESP32 programming port and you can open a terminal connection on the Raspberry Pi
to the ESP32 using minincom or screen.

For now use the same environment you used to flash the ESP32 and do idf.py monitor

The calibration happens in two phases, first you set Mini Pupper into calibration mode and you manually adjust the legs. After that you store the manual set positions in NVRAM and your Mini Pupper is calibrated

#### Begin Calibration

With a termina connection to the ESP32 enter::

  calibrate-begin

You can now assemble the Mini Pupper legs, move all hip servos in a position so that the legs are vertical, and leg servos in a position so that the legs are in an orientation of 45 degrees.

#### End Calibration

Once the legs are in the ideal position you can store the configiration by entering::

  calibrate-end

Calbration offsets are stored in NVRAM and read each time you restart Mini Pupper

#### Clear Calibration

To clear calibration data from NVRAM enter::

  calibrate-clear

You may now start the calibration process again.
