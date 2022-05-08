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
    plotter = sim.Plotter('squared')
    geometryTools = calculations.GeometryTools()
    # velocityController = controllers.VelocityController()
    pathPlanner = planning.PathPlanner(0.05, 0.1, 'squared')

    # LEGS MOVEMENT
    # motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    # motorDriver = dynamixel.MotorDriver(motors)
    # motorDriver.disableLegs(5)

    spiderStartPose = [0.3, 0.3, spider.WALKING_HEIGHT, 1.57]
    spiderGoalPose = [0.5, 1, spider.WALKING_HEIGHT, -1.57]

    path = pathPlanner.calculateSpiderBodyPath(spiderStartPose, spiderGoalPose, True)
    pins = pathPlanner.calculateSpiderLegsPositionsXyzRpyFF(path)
    plotter.plotSpiderMovement(path, pins)

    # velocityController.walk(spiderStartPose, spiderGoalPose)

    # velocityController.walk(spiderGoalPose, spiderStartPose, False)

    # velocityController.movePlatformWrapper([0.4, 0.33, 0.2, 0, 0, 0], spiderStartPose, 4)

    # path = pathPlanner.calculateSpiderBodyPath(spiderStartPose[:2], spiderGoalPose[:2], 0.05)
    # bestParams = [0.2 , 0.4, 0.4]
    # selectedPins = pathPlanner.calculateSpiderLegsPositionsFF(path, bestParams)
    # plotter.plotSpiderMovement(path, selectedPins)


    





    

