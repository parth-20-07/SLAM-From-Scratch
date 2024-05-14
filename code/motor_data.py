import os
import numpy as np
import matplotlib.pyplot as plt
import pprint
import utils

motor_data_file_path = "data/robot4_motors.txt"


def read_file_and_return_data(filePath):
    motor_data_file_object = open(motor_data_file_path)
    motor_data = motor_data_file_object.readlines()

    ts = []
    left_motor = []
    right_motor = []
    for data in motor_data:
        data = data.split()
        ts.append(int(data[1]))
        left_motor.append(int(data[2]))
        right_motor.append(int(data[6]))

    ts = np.asarray(ts)
    left_motor = np.asarray(left_motor)
    right_motor = np.asarray(right_motor)

    motor_data_file_object.close()
    return np.array([ts, left_motor, right_motor])


def calculate_motion(data):
    ts, left_data, right_data = data

    dt = []
    dLeft = []
    dRight = []

    for idx in range(1, ts.size):
        dt.append(ts[idx] - ts[idx - 1])
        dLeft.append(left_data[idx] - left_data[idx - 1])
        dRight.append(right_data[idx] - right_data[idx - 1])

    dt = np.asarray(dt)
    dLeft = np.asarray(dLeft)
    dRight = np.asarray(dRight)

    return np.array([dt, dLeft, dRight])


def main():
    raw_encoder_data = read_file_and_return_data(motor_data_file_path)
    utils.plot_data(
        1,
        raw_encoder_data,
        ["left_motor", "right_motor"],
        "TS",
        "Encoder Value",
        "resources/results/Encoder_Values.png",
    )

    motion_difference = calculate_motion(raw_encoder_data)
    utils.plot_data(
        2,
        np.array([raw_encoder_data[0][1:], motion_difference[1], motion_difference[2]]),
        ["left_motor", "right_motor"],
        "TS",
        "Encoder Change",
        "resources/results/Encoder_Change.png",
    )
    plt.show()


if __name__ == "__main__":
    main()
