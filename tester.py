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

    trajectory, velocity = trajectoryPlanner.minJerkTrajectory([0, 0, 0], [0, 0, 1], 10)

    J = kinematics.legJacobi(0, [0, 0, 0])
    xDot = [1, 0, 0]
    qDot = np.dot(np.linalg.inv(J), xDot)*(-1)

    bezierTraj = trajectoryPlanner.bezierTrajectory([0, 0, 0], [0.35, 0, 0], 5)
    print(bezierTraj)

    plt.plot(bezierTraj[:,0], bezierTraj[:,2],)
    plt.show()