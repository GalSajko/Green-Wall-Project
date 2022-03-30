"""
This module is meant as a testing sandbox for other modules, during implementation.
"""

import numpy as np
import time
import math
from threading import Thread

import environment
import simulaton
import calculations 
import mappers
import dynamixel
import planning


if __name__ == "__main__":

    #region PATH PLANNING
    # pathPlanner = planning.PathPlanner()
    # path = pathPlanner.calculateSpiderBodyPath([0.5, 0.5], [5.5, 3.5], 0.05)
    # bestParams = [0.0323541 , 0.38081064, 0.58683527]
    # selectedPins = pathPlanner.calculateSpiderLegsPositionsFF(path, bestParams)
    # print(selectedPins)
    #endregion

    #region LEG MOVEMENT AND POSITION READING.
    goalPositions = [0.35, 0, -0.035]
    kinematics = calculations.Kinematics()
    spider = environment.Spider()
    motorsIds = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    motorsDriver = dynamixel.MotorDriver(motorsIds)

    for i in range(spider.NUMBER_OF_LEGS):
        motorsDriver.moveLeg(i, goalPositions)

    startLegPositions = [motorsDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
    startLegLenghts = [np.linalg.norm(legPosition) for legPosition in startLegPositions]
    print(startLegLenghts)
    #endregion

    #region PARALLEL PLATFORM
    trajectoryPlanner = planning.TrajectoryPlanner()

    startPose = [0, 0, 0, 0, 0, 0]
    goalPose = [0, 0, 0.15, 0, 0, 0]
    trajectory = trajectoryPlanner.platformLinearTrajectory(startPose, goalPose)

    pins = [
        [(0.125 + startLegLength) * math.sin(idx * -spider.ANGLE_BETWEEN_LEGS), (0.125 + startLegLength) * math.cos(idx * spider.ANGLE_BETWEEN_LEGS), 0]
        for idx, startLegLength in enumerate(startLegLenghts)]

    for pose in trajectory:
        # motorsDriver.movePlatform(pose, pins)
        # time.sleep(1)
        
    
    # motorsDriver.disableMotors()

    #endregion

