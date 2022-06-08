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
    controller = controllers.VelocityController()
    # motors = dynamixel.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]], False)
    wall = env.Wall('squared')
    trajPlanner = planning.TrajectoryPlanner()
    pathPlanner = planning.PathPlanner(0.05, 0.15)
    startPosition = [0.6, 0.5, 0.2, 0.0]

    pins = wall.createGrid(True)

    legs = [0, 1, 2, 3, 4]
    # controller.moveLegsWrapper(legs, [pins[22], pins[9], pins[13], pins[25], pins[33]], startPosition, [7] * 5, ['o'] * 5, trajectoryType = 'minJerk')
    # controller.moveLegsWrapper([legs[0]], [pins[28]], startPosition, [7], ['o'], trajectoryType = 'minJerk')
    controller.movePlatformWrapper(startPosition, startPosition, [pins[22], pins[9], pins[13], pins[25], pins[33]], 3)
    controller.moveLegsAndGrabPins([legs[2]], [pins[13]], startPosition, [6])

    





    





    

