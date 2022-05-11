"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import time
import numpy as np
import matplotlib.pyplot as plt

import calculations
import dynamixel
import planning
import controllers
import environment as env
import simulaton as sim

if __name__ == "__main__":
    spider = env.Spider()
    wall = env.Wall('squared')
    plotter = sim.Plotter('squared')
    # velocityController = controllers.VelocityController()
    pathPlanner = planning.PathPlanner(0.05, 0.2, 'squared')
    trajPlanner = planning.TrajectoryPlanner()
    kinematics = calculations.Kinematics()

    motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    motorDriver = dynamixel.MotorDriver(motors)

    spiderStartPose = [0.4, 0.33, spider.LYING_HEIGHT, 0]
    spiderGoalPose = [0.4, 1, spider.WALKING_HEIGHT, 0]

    path, pins = pathPlanner.calculateWalkingMovesFF(spiderStartPose, spiderGoalPose)

    p0 = pins[0][0]
    p2 = pins[0][2]
    p3 = pins[0][3]

    legs = [0, 1, 2, 3, 4]
    legsLocalPositions = [motorDriver.readLegPosition(leg) for leg in legs]
    kinematics.platformDirectKinematics([p0, p2, p3], [legsLocalPositions[0], legsLocalPositions[2], legsLocalPositions[3]])








    





    

