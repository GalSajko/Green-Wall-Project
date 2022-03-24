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

    goalPosition = [0.22, 0.1, 0]
    kinematics = calculations.Kinematics()

    motorsIds = [[41, 42, 43]]
    motorsDriver = dynamixel.MotorDriver(motorsIds)
    motorsDriver.moveLeg(0, goalPosition)
    time.sleep(0.1)
    moving = True

    while moving:
        firstMotor = motorsDriver.isMotorMoving(41)
        secondMotor = motorsDriver.isMotorMoving(42)
        thirdMotor = motorsDriver.isMotorMoving(43)
        moving = firstMotor or secondMotor or thirdMotor
        print(moving)

    position = motorsDriver.readLegPosition(0)
    print(position)