from MangDang.mini_pupper.HardwareInterface import HardwareInterface
import MangDang.mini_pupper.nvram as nvram
import numpy as np


def main():
    MICROS_PER_RAD = (760 - 210) / np.pi
    NEUTRAL_ANGLE_DEGREES = np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
    data = {'MICROS_PER_RAD': MICROS_PER_RAD,
            'NEUTRAL_ANGLE_DEGREES': NEUTRAL_ANGLE_DEGREES}

    # clear clibration data
    nvram.write(data)

    hardware_interface = HardwareInterface()
    joint_angles = np.array([[0, 0, 0, 0], [45, 45, 45, 45], [-45, -45, -45, -45]])
    hardware_interface.set_actuator_postions(np.radians(joint_angles))


if __name__ == "__main__":
    main()
