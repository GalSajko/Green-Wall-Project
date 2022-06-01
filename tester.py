"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import time
import numpy as np
import matplotlib.pyplot as plt
import serial
import threading

import calculations
import dynamixel
import planning
import controllers
import environment as env
import simulaton as sim

if __name__ == "__main__":
    controller = controllers.VelocityController()
    trajPlanner = planning.TrajectoryPlanner()
    wall = env.Wall('squared')
    pins = wall.createGrid(True)
    # motors = dynamixel.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]], False)
    # motors.disableMotor(13)
    # time.sleep(1)
    # motors.enableMotors()

    # grippers = controllers.GripperController()
    # legs = [0, 1, 2, 3, 4]
    # for leg in legs:
    #     grippers.moveGripper(leg, 'o')
    
    # time.sleep(7)
    # for leg in legs:
    #     grippers.moveGripper(leg, 'c')

    pathPlanner = planning.PathPlanner(0.05, 0.2)
    path = pathPlanner.calculateSpiderBodyPath([0.6, 0.5, 0.2, 0.0], [0.6, 0.6, 0.2, 0.0])
    gridPlan = pathPlanner.calculateSpiderLegsPositionsXyzRpyFF(path)

    # controller.moveLegsWrapper(5, gridPlan[0], [0.6, 0.5, 0.2, 0.0], [7, 7, 7, 7, 7], trajectoryType = 'minJerk')

    controller.movePlatformWrapper([0.6, 0.5, 0.2, 0.0], [0.6, 0.5, 0.2, 0.0], gridPlan[0], 300)
    # controller.movePlatformWrapper([0.6, 0.7, 0.2, 0.0], [0.75, 0.5, 0.2, 0.0], gridPlan[0], 7)
    # controller.movePlatformWrapper([0.75, 0.5, 0.2, 0.0], [0.6, 0.5, 0.2, 0.0], gridPlan[0], 7)
    # controller.moveLegsAndGrabPins([0], [pins[27]], [0.6, 0.7, 0.2, 0.0], [7])

    

    

    
    






    





    

