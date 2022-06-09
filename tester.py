"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import time
import numpy as np
import matplotlib.pyplot as plt
import serial
import threading
import itertools as itt
import csv

import calculations
import dynamixel
import planning
import controllers
import environment as env
import simulaton as sim

if __name__ == "__main__":
    time.sleep(5)
    controller = controllers.VelocityController()
    # kinematics = calculations.Kinematics()
    motors = dynamixel.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]], False)
    wall = env.Wall('squared')
    # trajPlanner = planning.TrajectoryPlanner()
    pathPlanner = planning.PathPlanner(0.05, 0.15)
    startPosition = [0.6, 0.3, 0.3, 0.0]
    goalPosition = [0.6, 0.75, 0.25, 0.0]

    pins = wall.createGrid(True)

    legs = [0, 1, 2, 3, 4]
    legsGlobal = [pins[21], pins[8], pins[6], pins[30], pins[32]]


    # path = pathPlanner.calculateSpiderBodyPath(startPosition, goalPosition)
    # pathPins = pathPlanner.calculateSpiderLegsPositionsXyzRpyFF(path)
    # startingPins = pathPins[0]

    pose,ppins = pathPlanner.calculateWalkingMovesFF(startPosition, goalPosition)
    print(pose)
    print(ppins)

    # controller.moveLegsWrapper(legs, ppins[0], startPosition, [5] * 5, trajectoryType = 'minJerk')

    # controller.movePlatformWrapper(legs, startPosition, legsGlobal, 2)
    # controller.movePlatformWrapper(legs, [0.6, 0.5, 0.3, 0.0], legsGlobal, 5)
    # controller.movePlatformWrapper(legs, [0.4, 0.3, 0.3, 0.0], legsGlobal, 5)
    # controller.movePlatformWrapper(legs, [0.8, 0.3, 0.3, 0.0], legsGlobal, 5)
    # controller.movePlatformWrapper(legs, [0.6, 0.1, 0.3, 0.0], legsGlobal, 5)
    # controller.movePlatformWrapper(legs, startPosition, legsGlobal, 5)
    # spiderPose = motors.readPlatformPose(legs, legsGlobal)


    # spiderPose = motors.readPlatformPose(legs, ppins[-1])
    # controller.walk([spiderPose[0], spiderPose[1], spiderPose[2], spiderPose[-1]], startPosition)

    spiderPose = motors.readPlatformPose(legs, ppins[0])
    print(spiderPose)
    controller.walk([spiderPose[0], spiderPose[1], spiderPose[2], spiderPose[-1]], goalPosition)
    time.sleep(5)
    spiderPose = motors.readPlatformPose(legs, ppins[-1])
    print(spiderPose)
    controller.walk([spiderPose[0], spiderPose[1], spiderPose[2], spiderPose[-1]], startPosition)


   


