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
    wall = env.Wall('squared')
    spider = env.Spider()

    pins = wall.createGrid(True)

    spiderPose = [0.4, 0.33, spider.LYING_HEIGHT, 0]

    velocityController.moveLegsAndGrabPins([4], [pins[24]], spiderPose, [5], False, [pins[30]])
    time.sleep(3)   
    velocityController.moveLegsAndGrabPins([4], [pins[31]], spiderPose, [5], False, [pins[24]])
    time.sleep(3)
    velocityController.moveLegsAndGrabPins([4], [pins[30]], spiderPose, [5], False, [pins[31]])


    
    






    





    

