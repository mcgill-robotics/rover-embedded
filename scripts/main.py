import csv
import numpy as np
from matplotlib import pyplot as plt

# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    data = np.genfromtxt('C:/Users/Eren/Desktop/hall.csv', delimiter=',')
    print(data)
    speed = data[:, 0]
    halla = data[:, 1]
    hallb = data[:, 2]
    hallc = data[:, 3]

    hall_avg = (halla + hallb + hallc) / 3
    dire = data[:, 4]
    x = np.linspace(0, 1851, 1851)

    di = np.subtract(halla, hall_avg)
    direction = np.where(di > 0, 500, 0)
    # interrupts = data[:, 2]

    # speed_fit = np.polynomial.polynomial.Polynomial.fit(speed, halla, 2)
    # spe = speed_fit.linspace(speed.size, [speed[0], speed[1]])

    ha = plt.plot(x, halla, label="Hall Sensor A")
    hb = plt.plot(x, hallb, label="Hall Sensor B")
    hc = plt.plot(x, hallc, label="Hall Sensor C")
    havg = plt.plot(x, hall_avg, label="Hall Sensor Averages")

    plt.plot(x, direction)
    # plt.plot(speed, interrupts, label="Interrupts per 100ms")
    plt.legend()
    plt.show()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
