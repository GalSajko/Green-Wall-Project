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
    velocityController = controllers.VelocityController()
    pathPlanner = planning.PathPlanner(0.05, 0.2, 'squared')
    trajPlanner = planning.TrajectoryPlanner()
    kinematics = calculations.Kinematics()

    # LEGS MOVEMENT
    motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    motorDriver = dynamixel.MotorDriver(motors)
    # motorDriver.disableLegs(5)

    spiderStartPose = [0.4, 0.33, spider.LYING_HEIGHT, 0]
    spiderGoalPose = [0.4, 1, spider.WALKING_HEIGHT, 0]

    # path = pathPlanner.calculateSpiderBodyPath(spiderStartPose, spiderGoalPose)
    # pins = pathPlanner.calculateSpiderLegsPositionsXyzRpyFF(path)
    path, pins = pathPlanner.calculateWalkingMovesFF(spiderStartPose, spiderGoalPose)
    # plotter.plotSpiderMovement(path, pins)

    # velocityController.walk(spiderStartPose, spiderGoalPose)
    legs = [0, 1]
    motorDriver.addGroupSyncReadParams(legs)
    jointsValues = motorDriver.syncReadMotorsPositionsInLegs(legs)

    kinematics.platformDirectKinematics(legs, [pins[0][0], pins[0][1]], jointsValues)

    # velocityController.walk(spiderGoalPose, spiderStartPose, False)

    # velocityController.movePlatformWrapper([0.4, 0.33, 0.2, 0, 0, 0], spiderStartPose, 4)

    # path = pathPlanner.calculateSpiderBodyPath(spiderStartPose[:2], spiderGoalPose[:2], 0.05)
    # bestParams = [0.2 , 0.4, 0.4]
    # selectedPins = pathPlanner.calculateSpiderLegsPositionsFF(path, bestParams)
    # plotter.plotSpiderMovement(path, selectedPins)


    





    

