# Demo Scripts

## Haptic Demo

Run:

./hapic-demo.py

You can now move the front right leg and the other legs will follow. Note that some legs will move in the oposite direction due to the oriantation of the servos. Correcting this is left as an excercise for the user.

## IMU Demo

Run:

./mpl_plotter.py

and/or

./quaternion_plotter.py

These script will produce a GUI, either run it when a monitor is connect to your Mini Pupper or use it with ssh X Windows forwarding.

mpl_plotter will read yaw, pitch and roll, quaternion_plotter will read the quaternions. All IMU data is filtered on the ESP32. The IMU is positioned "upside down" on the main board. The orientation of yaw, pitch and roll is already corrected, the quaternions are not correctet (for the time being).
