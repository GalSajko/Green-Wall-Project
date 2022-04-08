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
    legIds = [0, 1, 2, 3, 4]
    goalPosition = [0.35, 0, 0.035]
    goalPositions = [goalPosition] * len(legIds)

    kinematics = calculations.Kinematics()
    spider = environment.Spider()
    trajectoryPlanner = planning.TrajectoryPlanner()
    matrixCalculator = calculations.MatrixCalculator()

    
    motorsIds = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    motorDriver = dynamixel.MotorDriver(motorsIds, False)

    # for motorId in np.array(motorsIds).flatten():
    #     motorDriver.setVelocityProfile(motorId, 500, 1000)
    #     motorDriver.setPositionPids(motorId, 2000, 200, 80)

    
    
    # motorDriver.enableMotors()

    # motorDriver.moveLegs(legIds, goalPositions)
    # goalPosition = [0.35, 0, -0.035]
    # goalPositions = [goalPosition] * len(legIds)
    # motorDriver.moveLegs(legIds, goalPositions)

    motorDriver.disableMotors()
    for motorId in np.array(motorsIds).flatten():
        motorDriver.setMotorDriveMode(motorId, 0)
        motorDriver.setPositionPids(motorId, 2000, 200, 80)
        motorDriver.setVelocityProfile(motorId, 10, 20)
    motorDriver.enableMotors()

    motorDriver.moveLegs(legIds, goalPositions)
    goalPosition = [0.35, 0, -0.035]
    goalPositions = [goalPosition] * len(legIds)
    motorDriver.moveLegs(legIds, goalPositions)
    

    #endregion

    #region PARALLEL PLATFORM

    startPose = [0, 0, 0.04, 0, 0, 0]
    goalPoses = [
        [0, 0, 0.2, 0, 0, 0],
        [0, 0.1, 0.2, 0, 0, 0],
        [0.1, 0, 0.2, 0, 0, 0],
        [0, -0.1, 0.2, 0, 0, 0],
        [-0.1, 0, 0.2, 0, 0, 0],
        [0, 0.1, 0.2, 0, 0, 0],
        [0, 0, 0.2, 0, 0, 0],
        startPose
    ]

    for idx, goalPose in enumerate(goalPoses):
        if idx != 0:
            startPose = goalPoses[idx - 1]

        startLegPositions = [motorDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
        T_GS = matrixCalculator.transformMatrixFromGlobalPose(startPose)
        if idx == 0:
            pins = []
            for idx, t in enumerate(spider.T_ANCHORS):
                anchorInGlobal = np.dot(T_GS, t)
                pinMatrix = np.array([
                    [1, 0, 0, startLegPositions[idx][0]],
                    [0, 1, 0, startLegPositions[idx][1]],
                    [0, 0, 1, startLegPositions[idx][2]],
                    [0, 0, 0, 1]])
                pinInGlobal = np.dot(anchorInGlobal, pinMatrix)
                pins.append(pinInGlobal[:,3][0:3])

        trajectory = trajectoryPlanner.platformLinearTrajectory(startPose, goalPose)
        motorDriver.movePlatformTrajectory(trajectory, pins)

    #endregion

