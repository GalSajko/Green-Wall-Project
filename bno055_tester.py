import adafruit_bno055
import board
import time
import numpy as np


if __name__ == '__main__':
    i2c = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c)

    mag_radius = (2, 203)
    acc_radius = (131, 232)
    gyr_offset = (0, 0, 255, 254, 0, 1)
    mag_offset = (128, 232, 129, 170, 254, 132)
    acc_offset = (255, 250, 255, 243, 255, 236)

    # print(sensor.mag_radius)
    # print(sensor.acc_radius)
    # print(sensor.gyr_offset)
    # print(sensor.mag_offset)
    # print(sensor.acc_offset)

    sensor.calibrate(mag_radius, acc_radius, gyr_offset, mag_offset, acc_offset)

    # print(sensor.calibrated)

    while not sensor.calibrated:
        print(sensor.calibration_status)
        time.sleep(0.1)
    
    print(sensor.calibration_status)
    print(sensor.mag_radius)
    print(sensor.acc_radius)
    print(sensor.gyr_offset)
    print(sensor.mag_offset)
    print(sensor.acc_offset)

    while True:
        print(sensor.euler)
        time.sleep(0.1)
    