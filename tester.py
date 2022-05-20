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
    # velocityController = controllers.VelocityController()
    wall = env.Wall('squared')
    spider = env.Spider()

    pins = wall.createGrid(True)

    spiderPose = [0.4, 0.33, spider.LYING_HEIGHT, 0]

    # velocityController.moveLegsAndGrabPins([0, 1], [pins[10], pins[0]], spiderPose, [5, 5])
    gripperController = controllers.GripperController()

    readingThread = threading.Thread(target = gripperController.readData)
    readingThread.start()

    while True:
        print(gripperController.receivedMessage)
        time.sleep(0.01)

    
    






    





    

