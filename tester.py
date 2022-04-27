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
    spider = environment.Spider()
    trajectoryPlanner = planning.TrajectoryPlanner()
    kinematics = calculations.Kinematics()
    matrixCalculator = calculations.MatrixCalculator()
    motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    motorDriver = dynamixel.MotorDriver(motors)

    firstLegStartPosition = motorDriver.readLegPosition(1)
    firstLegStartPosition = np.append(firstLegStartPosition, [0, 0, 0])
    firstLegGoalPose = np.array([0.35, 0, 0.4, 0, 0, 0])

    
    trajectory, velocity = trajectoryPlanner.minJerkTrajectory(firstLegStartPosition, firstLegGoalPose, 3)
    # plt.plot(trajectory[:,-1], trajectory[:0])

    startTime = time.time()
    result =  velocityController.moveLeg(1, trajectory, velocity)
    print(time.time() - startTime)

    if result:
        print(motorDriver.readLegPosition(1))

    

