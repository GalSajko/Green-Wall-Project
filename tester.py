"""
This module is meant as a testing sandbox for other modules, during implementation.
"""

import numpy as np
import time

import environment
import simulaton
import calculations 
import mappers
import dynamixel

if __name__ == "__main__":

    pathPlanner = calculations.PathPlanner()
    path = pathPlanner.calculateSpiderBodyPath([0.5, 0.5], [5.5, 3.5], 0.05)
    bestParams = [0.0323541 , 0.38081064, 0.58683527]
    selectedPins = pathPlanner.calculateSpiderLegsPositionsFF(path, bestParams)

    goalPosition = [0.3, 0, 0]
    kinematics = calculations.Kinematics()

    motorsIds = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    motorsDriver = dynamixel.MotorDriver(motorsIds)
    # motorsDriver.moveLeg(4, goalPosition)


    # time.sleep(0.1)
    # moving = True

    # while moving:
    #     moving = motorsDriver.isLegMoving(2)
    #     # print(moving)

    # time.sleep(0.2)
    position = motorsDriver.readLegPosition(3)
    print(position)

    motorsDriver.disableMotors()