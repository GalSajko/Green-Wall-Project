"""
This module is meant as a testing sandbox for other modules, during implementation.
"""

import numpy as np
import time
from threading import Thread

import environment
import simulaton
import calculations 
import mappers
import dynamixel


if __name__ == "__main__":

    #region PATH PLANNING
    # pathPlanner = calculations.PathPlanner()
    # path = pathPlanner.calculateSpiderBodyPath([0.5, 0.5], [5.5, 3.5], 0.05)
    # bestParams = [0.0323541 , 0.38081064, 0.58683527]
    # selectedPins = pathPlanner.calculateSpiderLegsPositionsFF(path, bestParams)
    #endregion

    #region LEG MOVEMENT AND POSITION READING.
    # goalPositions = [0.35, 0, -0.15]
    # kinematics = calculations.Kinematics()
    # spider = environment.Spider()
    # motorsIds = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    # motorsDriver = dynamixel.MotorDriver(motorsIds)

    # for legId in range(spider.NUMBER_OF_LEGS):
    #     motorsDriver.moveLeg(legId, goalPositions)
    # motorsDriver.moveLeg(0, [0.35, 0.1, 0])
    # motorsDriver.moveLeg(2, [0.35, 0.1, 0])

    # while True:
    #     position = motorsDriver.readLegPosition(3)
    #     print(position)

    # motorsDriver.disableMotors()
    #endregion

    #region PARALLEL PLATFORM
    calculations.Kinematics().platformInverseKinematics([0, 0, 0, 0, 0, 0], [[0, 0.325, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])


    
    #endregion

