"""
This module is meant as a testing sandbox for other modules, during implementation.
"""

import numpy as np
import time
import math
from threading import Thread
import sys

import environment
import simulaton
import calculations 
import mappers
import dynamixel
import planning

def main():
    #region PATH PLANNING
    # pathPlanner = planning.PathPlanner()
    # path = pathPlanner.calculateSpiderBodyPath([0.5, 0.5], [5.5, 3.5], 0.05)
    # bestParams = [0.0323541 , 0.38081064, 0.58683527]
    # selectedPins = pathPlanner.calculateSpiderLegsPositionsFF(path, bestParams)
    # print(selectedPins)
    #endregion

    #region MOVEMENT.
    kinematics = calculations.Kinematics()
    spider = environment.Spider()
    trajectoryPlanner = planning.TrajectoryPlanner()
    matrixCalculator = calculations.MatrixCalculator()

    
    motorsIds = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    motorDriver = dynamixel.MotorDriver(motorsIds)

    for motorId in np.array(motorsIds).flatten():
        # motorDriver.setMotorDriveMode(motorId, 4)
        # motorDriver.setPositionPids(motorId, 800, 0, 0)
        motorDriver.setVelocityProfile(motorId, 300, 700)


    legIds = [0, 1, 2, 3, 4]
    goalPosition = [0.25, 0, 0.035]
    goalPositions = [goalPosition] * len(legIds)
    motorDriver.moveLegs(legIds, goalPositions)
    goalPositions = [[0.25, 0, -0.035]] * len(legIds)
    motorDriver.moveLegs(legIds, goalPositions)

    # Show leg trajectory movement.
    legTrajectory = trajectoryPlanner.legCircularTrajectory(goalPosition, [0.45, 0, -0.035])
    for motorId in np.array(motorsIds).flatten():
        motorDriver.setVelocityProfile(motorId, 1, 500)
    for position in legTrajectory:
        goalPositions = [position] * len(legIds)
        motorDriver.moveLegs(legIds, goalPositions)

    # Show parallel movement.

    startPose = [0, 0, 0.04, 0, 0, 0]
    # Write any goal pose (rpy included).
    goalPoses = [
        [0, 0, 0.2, 0, 0, 0],
        [0.1, 0.1, 0.2, 0.2, 0.2, 0.2],
        [-0.1, -0.1, 0.2, -0.2, -0.2, -0.2],
        # [0, 0, 0.2, -0.2, -0.2, -0.2],
        # [0, 0.1, 0.2, 0, 0, 0],
        # [0.1, 0, 0.2, 0, 0, 0],
        # [0, -0.1, 0.2, 0, 0, 0],
        # [-0.1, 0, 0.2, 0, 0, 0],
        # [0, 0.1, 0.2, 0, 0, 0],
        [0, 0, 0.2, 0, 0, 0],
        startPose
    ]

    for idx, goalPose in enumerate(goalPoses):
        if idx != 0:
            startPose = goalPoses[idx - 1]

        startLegPositions = [motorDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
        T_GS = matrixCalculator.transformMatrixFromGlobalPose(startPose)
        if idx == 0:
            pins = matrixCalculator.getPinsInGlobal(spider.T_ANCHORS, T_GS, startLegPositions, startPose)

        trajectory = trajectoryPlanner.platformLinearTrajectory(startPose, goalPose)
        motorDriver.movePlatformTrajectory(trajectory, pins)

    motorDriver.disableMotors()
    #endregion    

if __name__ == "__main__":
    main()