"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import time
import numpy as np
import matplotlib.pyplot as plt
import serial

import calculations
import dynamixel
import planning
import controllers
import environment as env
import simulaton as sim

if __name__ == "__main__":
    # velocityController = controllers.VelocityController()
    # wall = env.Wall('squared')
    # spider = env.Spider()

    # pins = wall.createGrid(True)

    # spiderPose = [0.4, 0.33, spider.LYING_HEIGHT, 0]

    # velocityController.moveLegsAndGrabPins([0], [pins[24]], spiderPose, [4])

    ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
    ser.reset_input_buffer()

    while True:
        msg = input("Enter msg: ")
        ser.write(msg)
        time.sleep(0.5)
        rec = ser.readline().decode("utf-8").rstrip()
        print(rec)
    






    





    

