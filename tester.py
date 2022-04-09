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
        motorDriver.setVelocityProfile(motorId, 1, 500)

    # Show parallel movement.
    motorDriver.enableMotors()
    startPose = [0, 0, 0.04, 0, 0, 0]
    # Write any goal pose (rpy included).
    goalPoses = [
        [0, 0, 0.2, 0, 0, 0],
        [0.1, 0.1, 0.2, 0.2, 0.2, 0.2],
        [-0.1, -0.1, 0.2, -0.2, -0.2, -0.2],
        [-0.1, 0.1, 0.2, 0.2, 0.2, 0.2],
        [0.1, -0.1, 0.2, -0.2, -0.2, -0.2],
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

    #region WALKING

    # Stand up and move platform 15 cm forward.
    # startPose = [0, 0, 0.04, 0, 0, 0]
    # goalPoses = [
    #     [0, 0, 0.2, 0, 0, 0],
    #     [0.0, 0.1, 0.2, 0, 0, 0]]
    # for idx, goalPose in enumerate(goalPoses):
    #     if idx != 0:
    #         startPose = goalPoses[idx - 1]

    #     if idx == 0:
    #         startLegPositions = [motorDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
    #         T_GS = matrixCalculator.transformMatrixFromGlobalPose(startPose)
    #         pins = matrixCalculator.getPinsInGlobal(spider.T_ANCHORS, T_GS, startLegPositions, startPose)

    #     trajectory = trajectoryPlanner.platformLinearTrajectory(startPose, goalPose)
    #     motorDriver.movePlatformTrajectory(trajectory, pins)
    
    # # Move legs 3 and 5 in direction of movement.
    # currentLegPositions = [motorDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
    # print(currentLegPositions)
    # trajectoryLeg3 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[2], [0.25, -0.1, -0.04])
    # trajectoryLeg5 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[4], [0.35, 0.1, -0.04])
    # for position in trajectoryLeg3:
    #     motorDriver.moveLegs([2], [position])
    # for position in trajectoryLeg5:
    #     motorDriver.moveLegs([4], [position])

    # # Move legs 2 and 4 in direction of movement.
    # trajectoryLeg2 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[1], [0.35, -0.1, -0.04])
    # trajectoryLeg4 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[3], [0.25, 0.1, -0.04])
    # for position in trajectoryLeg2:
    #     motorDriver.moveLegs([1], [position])
    # for position in trajectoryLeg4:
    #     motorDriver.moveLegs([3], [position])
    
    # # Move leg 1 in direciton of movement.
    # trajectoryLeg1 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[0], [0.35, 0, -0.04])
    # for position in trajectoryLeg1:
    #     motorDriver.moveLegs([0], [position])
    
    # # Move platform for 15 cm forward.
    # startPose = [0, 0.1, 0.2, 0, 0, 0]
    # goalPoses = [
    #     [0, 0.2, 0.2, 0, 0, 0]]
    # for idx, goalPose in enumerate(goalPoses):
    #     if idx != 0:
    #         startPose = goalPoses[idx - 1]

    #     if idx == 0:
    #         startLegPositions = [motorDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
    #         T_GS = matrixCalculator.transformMatrixFromGlobalPose(startPose)
    #         pins = matrixCalculator.getPinsInGlobal(spider.T_ANCHORS, T_GS, startLegPositions, startPose)

    #     trajectory = trajectoryPlanner.platformLinearTrajectory(startPose, goalPose)
    #     motorDriver.movePlatformTrajectory(trajectory, pins)
    #endregion 

if __name__ == "__main__":
    main()