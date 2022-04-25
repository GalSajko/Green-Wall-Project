"""
This module is meant as a testing sandbox for other modules, during implementation.
"""

from logging.handlers import RotatingFileHandler
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
import controllers

if __name__ == "__main__":
    velocityController = controllers.VelocityController()
    spider = environment.Spider()
    trajectoryPlanner = planning.TrajectoryPlanner()
    kinematics = calculations.Kinematics()
    matrixCalculator = calculations.MatrixCalculator()
    motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    motorDriver = dynamixel.MotorDriver(motors)

    firstLegStartPosition = motorDriver.readLegPosition(0)
    firstLegStartPosition = np.append(firstLegStartPosition, [0, 0, 0])
    firstLegGoalPose = np.array([0.35, -0.15, 0, 0, 0, 0])

    
    trajectory, velocity = trajectoryPlanner.minJerkTrajectory(firstLegStartPosition, firstLegGoalPose, 3)
    timeVector = trajectory[:,-1]


    startTime = time.time()
    velocityController.moveLeg(0, trajectory, velocity)

    currentLegPosition = motorDriver.readLegPosition(0)
    print(np.round(currentLegPosition, 3))

