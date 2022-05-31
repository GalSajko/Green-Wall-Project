"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import time
import numpy as np
import matplotlib.pyplot as plt
import serial
import threading

import calculations
import dynamixel
import planning
import controllers
import environment as env
import simulaton as sim

if __name__ == "__main__":
    grippers = controllers.GripperController()

    grippers.openGrippersAndWait([0, 1, 2, 3, 4])
    for i in range(5):
        grippers.moveGripper(i, 'c')
        time.sleep(1)

    

    
    






    





    

