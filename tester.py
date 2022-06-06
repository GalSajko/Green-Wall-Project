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
    motors = dynamixel.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]])
    trajPlanner = planning.TrajectoryPlanner()

    # motors.disableLegs(0)

    # grippers = controllers.GripperController()
    # grippers.openGrippersAndWait([0, 1, 2, 3, 4])
    legs = [0, 1, 2, 3, 4]
    # for leg in legs:
    #     grippers.moveGripper(leg, 'c')

    testLegsPosition = [0.55, 0.0, -0.1]
    legId = 0

    motors.clearGroupSyncReadParams()
    motors.addGroupSyncReadParams(legs)
    # while True:
    currentLegsPositions = motors.syncReadMotorsPositionsInLegs(legs, True, base = 'leg')
    currentLegsPositionsSpiderBase = motors.syncReadMotorsPositionsInLegs(legs, True, base = 'spider')
    # print(currentLegsPositions.flatten())
    # time.sleep(0.02)
    # print(currentLegsPositionsSpiderBase)
    
    trajectories, velocities = [], []
    for leg in legs:
        traj, vel = trajPlanner.minJerkTrajectory(currentLegsPositions[leg], testLegsPosition, 7)
        trajectories.append(traj)
        velocities.append(vel)
    # controller.moveLegs([legId], [trajectories[legId]], [velocities[legId]])
    # controller.moveLegs(5, trajectories, velocities)

    time.sleep(1)
    currentLegsPositions = motors.syncReadMotorsPositionsInLegs(legs, True, base = 'leg')
    currentLegsPositionsSpiderBase = motors.syncReadMotorsPositionsInLegs(legs, True, base = 'spider')
    print(currentLegsPositions)
    print(currentLegsPositionsSpiderBase)
    
    # poses = ['flat', '1st up', '2nd up', '3rd up', '4th up', '5th up', 'flat']
    # i = 6
    # with open('legs.csv', 'a') as f:
    #     writer = csv.writer(f)
    #     writer.writerow(poses[i])
    #     for legPos in currentLegsPositionsSync:
    #         writer.writerow(legPos)






    





    

