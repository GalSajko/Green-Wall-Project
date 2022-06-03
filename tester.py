"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import time
import numpy as np
import matplotlib.pyplot as plt
import serial
import threading
import itertools as itt

import calculations
import dynamixel
import planning
import controllers
import environment as env
import simulaton as sim

if __name__ == "__main__":
    controller = controllers.VelocityController()
    trajPlanner = planning.TrajectoryPlanner()
    kinematics = calculations.Kinematics()
    wall = env.Wall('squared')
    pins = wall.createGrid(True)
    motors = dynamixel.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]], False)
    # motors.disableMotor(13)
    # time.sleep(1)
    # motors.enableMotors()

    # grippers = controllers.GripperController()
    # grippers.openGrippersAndWait([0, 1, 2, 3, 4])
    # legs = [0, 1, 2, 3, 4]
    # for leg in legs:
    #     grippers.moveGripper(leg, 'o')
    
    # time.sleep(7)
    # for leg in legs:
    #     grippers.moveGripper(leg, 'c')

    pathPlanner = planning.PathPlanner(0.05, 0.2)
    path = pathPlanner.calculateSpiderBodyPath([0.6, 0.5, 0.2, 0.0], [0.6, 0.65, 0.2, 0.0])
    gridPlan = pathPlanner.calculateSpiderLegsPositionsXyzRpyFF(path)
    pos = np.copy(pins[25])
    pos[2] += 0.15

    controller.moveLegsWrapper([3], [pos], [0.6, 0.5, 0.25, 0.0], [5], trajectoryType = 'minJerk', gripperCommands = ['o'])
    # controller.movePlatformWrapper([0.6, 0.5, 0.25, 0.0, 0.0, 0.0], [0.6, 0.5, 0.25, 0.0, 0.0, 0.0], gridPlan[0], 3)
    # controller.moveLegsWrapper(5, gridPlan[0], [0.6, 0.5, 0.25, 0.0], [5, 5, 5, 5, 5], trajectoryType = 'minJerk')
    # controller.moveLegsAndGrabPins([3], [pins[25]], [0.6, 0.5, 0.25, 0.0], [7])
    # controller.movePlatformWrapper([0.6, 0.6, 0.2, 0.0], [0.6, 0.5, 0.2, 0.0], gridPlan[0], 3)


    legs = np.array([0, 1, 2, 3, 4])
    motors.clearGroupSyncReadParams()
    motors.addGroupSyncReadParams(legs)
    joints = motors.syncReadMotorsPositionsInLegs(legs)
    spiderBaseToLegsTransforms = np.array([kinematics.spiderBaseToLegTipForwardKinematics(leg, joints[leg]) for leg in legs])
    usedLegs = [0, 1, 2, 4]
    usedPins = np.array([pins[22], pins[9], pins[13], pins[25], pins[33]])
    poses = []
    for legsSubset in itt.combinations(usedLegs, 3):
        legsSubset = np.array(legsSubset)
        position = kinematics.platformForwardKinematics(legsSubset, usedPins[legsSubset], spiderBaseToLegsTransforms[legsSubset])
        poses.append(position)

    pose = np.mean(poses, axis = 0)
    # print(np.round(pose, 3))

    # pose = kinematics.platformForwardKinematics([0, 2, 4], [pins[22], pins[13], pins[33]], [spiderBaseToLegsTransforms[0], spiderBaseToLegsTransforms[2], spiderBaseToLegsTransforms[4]])
    controller.moveLegsWrapper([0, 1, 2, 4], [pins[22], pins[9], pins[13], pins[33]], [0.6, 0.5, 0.25, 0.0], [3, 3, 3, 3])
    # controller.movePlatformWrapper(pose, [0.6, 0.5, 0.25, 0.0, 0.0, 0.0], gridPlan[0], 3)
    controller.moveLegsAndGrabPins([3], [pins[30]], [0.6, 0.5, 0.25, 0.0], [7])






    





    

