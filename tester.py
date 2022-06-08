"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
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
    motors = dynamixel.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]], False)
    trajPlanner = planning.TrajectoryPlanner()


    # legs = [0, 1, 2, 3, 4]

    testLegsPosition = [0.35, 0.0, -0.25]
    legId = 1

    motors.clearGroupSyncReadParams()
    motors.addGroupSyncReadParams([legId])

    while True:
        currentLegsPositions = motors.syncReadMotorsPositionsInLegs([legId], True, base = 'leg')
        currentLegsPositionsSpiderBase = motors.syncReadMotorsPositionsInLegs([legId], True, base = 'spider')
        print("BEFORE MOVEMENT: ", currentLegsPositions.flatten())

        desiredPositionString = input("Enter x, y, z and time: ")
        values = desiredPositionString.split(',')
        desiredPosition = []
        for value in values:
            desiredPosition.append(float(value))

        traj, vel = trajPlanner.minJerkTrajectory(currentLegsPositions[0], desiredPosition[:3], desiredPosition[3])
        try:
            result = controller.moveLegs([legId], [traj], [vel])
        except:
            print("Cannot move there.")
            continue

        currentLegsPositions = motors.syncReadMotorsPositionsInLegs([legId], True, base = 'leg')
        currentLegsPositionsSpiderBase = motors.syncReadMotorsPositionsInLegs([legId], True, base = 'spider')
        print("AFTER MOVEMENT: ", currentLegsPositions.flatten())
        print("===========")
    





    





    

