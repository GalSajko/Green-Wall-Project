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
    wall = env.Wall()
    plotter = sim.Plotter()
    velocityController = controllers.VelocityController()
    pathPlanner = planning.PathPlanner('squared')

    # LEGS MOVEMENT
    motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    # motorDriver = dynamixel.MotorDriver(motors)
    # motorDriver.disableLegs(5)

    spiderStartPose = [0.4, 0.33, 0.06, 0, 0, 0]
    spiderGoalPose = [0.4, 1.0, 0.15, 0, 0, 0]


    velocityController.walk(spiderStartPose, spiderGoalPose)

    # path = pathPlanner.calculateSpiderBodyPath(spiderStartPose[:2], spiderGoalPose[:2], 0.05)
    # bestParams = [0.2 , 0.4, 0.4]
    # selectedPins = pathPlanner.calculateSpiderLegsPositionsFF(path, bestParams)
    # plotter.plotSpiderMovement(path, selectedPins)


    





    

