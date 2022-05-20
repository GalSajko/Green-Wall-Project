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
    velocityController = controllers.VelocityController()
    # gripperController = controllers.GripperController()
    wall = env.Wall('squared')
    spider = env.Spider()

    pins = wall.createGrid(True)

    spiderPose = [0.4, 0.33, spider.LYING_HEIGHT, 0]

    # gripperController.moveGripper(0, 'c')
    # gripperController.moveGripper(1, 'c')


    velocityController.moveLegsWrapper([0, 1], [pins[17], pins[1]], spiderPose, [5, 5], ["o", "o"])
    


    
    






    





    

