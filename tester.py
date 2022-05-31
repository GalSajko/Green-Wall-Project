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
    # controller = controllers.VelocityController()
    wall = env.Wall('squared')
    pins = wall.createGrid(True)
    # print(pins[28])
    motors = dynamixel.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]])
    # grippers = controllers.GripperController()
    # legs = [0, 1, 2, 3, 4]
    # for leg in legs:
    #     grippers.moveGripper(leg, 'c')

    planner = planning.PathPlanner(0.05, 0.2)
    path = planner.calculateSpiderBodyPath([0.6, 0.5, 0.2, 0.0], [0.6, 0.6, 0.2, 0.0])
    gridPlan = planner.calculateSpiderLegsPositionsXyzRpyFF(path)
    

    # controller.movePlatformWrapper([0.6, 0.5, 0.2, 0.0], [0.6, 0.6, 0.2, 0.0], [pins[22], pins[9], pins[13], pins[25], pins[33]], 10)
    # controller.moveLegsAndGrabPins([2], [pins[13]], [0.6, 0.5, 0.2, 0.0], [7])
    # controller.moveLegsWrapper(5, gridPlan[0], [0.6, 0.5, 0.2, 0.0], [7, 7, 7, 7, 7])
    while True:
        print(motors.readLegPosition(0))

    

    

    
    






    





    

