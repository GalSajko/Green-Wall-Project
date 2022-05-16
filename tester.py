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
    velocityController = controllers.VelocityController()
    matrixCalculator = calculations.MatrixCalculator()

    motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    motorDriver = dynamixel.MotorDriver(motors, False)

    spiderStartPose = [0.405, 0.3305, spider.LYING_HEIGHT, 0, 0, 0]
    spiderGoalPose = [0.405, 1, spider.WALKING_HEIGHT, 0, 0, 0]

    pins = wall.createGrid(True)


    velocityController.moveLegsAndGrabPins([0, 1, 2, 3, 4], [[0.4016, 0.6793, 0.0287], [-0.0022, 0.6764, 0.0289], [0.0056, 0.2292, 0.026], [0.5993, -0.0012, 0.0274], [0.8034, 0.2196, 0.0274]], spiderStartPose, [4, 4, 4, 4, 4])

    # velocityController.moveLegsAndGrabPins([0], [pins[17]], spiderStartPose, [4])
    # velocityController.moveLegsAndGrabPins([0], [pins[10]], spiderStartPose, [4], False, [pins[3]])
    # velocityController.moveLegsAndGrabPins([0], [pins[17]], spiderStartPose, [4], False, [pins[10]])



    legs = [0, 1, 2, 3, 4]
    globalLegsPositions = matrixCalculator.getLegsInGlobal([motorDriver.readLegPosition(leg) for leg in legs], spiderStartPose)
    print(np.round(globalLegsPositions, 4))



 






    





    

