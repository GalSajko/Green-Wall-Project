"""
This module is meant as a testing sandbox for other modules, during implementation.
"""

import numpy as np
import time
import matplotlib.pyplot as plt

import environment
import calculations
import dynamixel
import planning
import controllers

if __name__ == "__main__":
    velocityController = controllers.VelocityController()
    trajectoryPlanner = planning.TrajectoryPlanner()
    motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    motorDriver = dynamixel.MotorDriver(motors)

    firstLegStartPosition = motorDriver.readLegPosition(0)
    firstLegStartPosition = np.append(firstLegStartPosition, [0, 0, 0])
    firstLegGoalPose = np.array([0.35, 0, -0.035, 0, 0, 0])
    trajectory, velocity = trajectoryPlanner.minJerkTrajectory(firstLegStartPosition, firstLegGoalPose, 1)

    secondLegStartPosition = motorDriver.readLegPosition(1)
    secondLegStartPosition = np.append(secondLegStartPosition, [0, 0, 0])
    secondLegGoalPose = np.array([0.35, 0, -0.035, 0, 0, 0])
    trajectory1, velocity1 = trajectoryPlanner.minJerkTrajectory(secondLegStartPosition, secondLegGoalPose, 1)


    startTime = time.time()
    result = velocityController.moveLegs([0, 1], [trajectory, trajectory1], [velocity, velocity1])
    print(time.time() - startTime)

    if result:
        print(motorDriver.readLegPosition(0))
        print(motorDriver.readLegPosition(1))


    

