"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import time
import numpy as np
import matplotlib.pyplot as plt
import serial
import threading
import itertools as itt
import csv

import calculations
import dynamixel
import planning
import controllers
import environment as env
import simulaton as sim

if __name__ == "__main__":
    # controller = controllers.VelocityController()
    grippers = controllers.GripperController()

    while True:
        states = grippers.getSwitchesStates()
        print(states)

    # for i in range(5):
    #     # grippers.moveGripper(i, 'o')
    #     # time.sleep(1)
    #     grippers.openGrippersAndWait([i])
    #     grippers.moveGripper(i, 'c')
    #     time.sleep(1)

    # print("OPEN")





    

