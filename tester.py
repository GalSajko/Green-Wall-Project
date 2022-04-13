"""
This module is meant as a testing sandbox for other modules, during implementation.
"""

import numpy as np
import time
import math
from threading import Thread
import sys
import matplotlib.pyplot as plt

import environment
import simulaton
import calculations 
import mappers
import dynamixel
import planning
 

if __name__ == "__main__":
    spider = environment.Spider()
    trajectoryPlanner = planning.TrajectoryPlanner()
    kinematics = calculations.Kinematics()
    matrixCalculator = calculations.MatrixCalculator()
    motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    motorDriver = dynamixel.MotorDriver(motors) 
    startPose = [0, 0, 0.2, 0, 0, 0]
    goalPose = [0, 0.15, 0.2, 0, 0, 0]  
    trajectory, velocity = trajectoryPlanner.minJerkTrajectory(startPose, goalPose, 3)
    
    # plt.plot(trajectory[:,-1], trajectory[:,1], trajectory[:,-1], velocity[:,1])
    # plt.show()

    legsIds = [0, 1, 2, 3, 4]

    # Calculate angles in joints for each step of trajectory.
    qD = []
    timeStep = trajectory[1][-1] - trajectory[0][-1]

    for idx, t in enumerate(trajectory):
        pose, time = t[:-1], t[-1]
        T_GS = matrixCalculator.transformMatrixFromGlobalPose(pose)
        anchorsGlobal = np.array([np.dot(T_GS, tAnchor)[:,3][0:3] for tAnchor in spider.T_ANCHORS])

        if idx == 0:
            legs = [motorDriver.readLegPosition(i) for i in legsIds]
            legsGlobal = matrixCalculator.getLegsInGlobal(spider.T_ANCHORS, T_GS, legs, startPose)

        # Positions of legs on each step of trajectory - for calculating legs speed.
        jointRadians = kinematics.platformInverseKinematics(pose, legsGlobal)
        motorDegrees = np.array([np.degrees(mappers.mapJointRadiansToMotorRadians(values)) for values in jointRadians])
        legPositions = np.array([kinematics.legDirectKinematics(legId, motorDegrees[i])[:,3][0:3] for i, legId in enumerate(legsIds)])

        if idx != 0:
            deltaLegPositions = np.array(legPositions - legPositionsOld)
            deltaMotorDegrees = np.array(motorDegrees - motorDegreesOld)
            J = []
            for i in range(spider.NUMBER_OF_LEGS):
                J.append(np.array([
                    [deltaLegPositions[i][0] / deltaMotorDegrees[i][0], deltaLegPositions[i][0] / deltaMotorDegrees[i][1], deltaLegPositions[i][0] / deltaMotorDegrees[i][2]],
                    [deltaLegPositions[i][1] / deltaMotorDegrees[i][0], deltaLegPositions[i][1] / deltaMotorDegrees[i][1], deltaLegPositions[i][1] / deltaMotorDegrees[i][2]],
                    [deltaLegPositions[i][2] / deltaMotorDegrees[i][0], deltaLegPositions[i][2] / deltaMotorDegrees[i][1], deltaLegPositions[i][2] / deltaMotorDegrees[i][2]],
                ]))
            print(np.array(J))
            print("========")
        qD.append(jointRadians)

        legPositionsOld = legPositions
        motorDegreesOld = motorDegrees
    
    # print(np.array(qD))

    