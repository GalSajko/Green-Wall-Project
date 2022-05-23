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
    wall = env.Wall('squared')
    spider = env.Spider()
    # controller = controllers.VelocityController()
    motors = dynamixel.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43]], False)

    pins = wall.createGrid(True)
    spiderPose = [0.4, 0.33, spider.LYING_HEIGHT, 0.0]

    print(pins[2], pins[1])

    # controller.moveLegsWrapper([1], [pins[2]], spiderPose, [5])
    motors.addGroupSyncReadParams([2])
    while True:
        localLegPosition = motors.syncReadMotorsPositionsInLegs([2], True)

        globalLegPosition = calculations.MatrixCalculator().getLegsInGlobal([2], localLegPosition, spiderPose)
        print(globalLegPosition)


    
    






    





    

