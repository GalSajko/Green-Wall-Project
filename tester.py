"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import numpy as np
import matplotlib.pyplot as plt

import calculations
import dynamixel
import planning
import controllers

if __name__ == "__main__":
    velocityController = controllers.VelocityController()
    matrixCalculator = calculations.MatrixCalculator()
    trajectoryPlanner = planning.TrajectoryPlanner()

    # LEGS MOVEMENT
    motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    motorDriver = dynamixel.MotorDriver(motors)

    # motorDriver.disableLegs(2)
    # motorDriver.disableLegs(4)

    # PIN TO PIN TWO LEGS MOVEMENT DEMO:
    leg2Start = motorDriver.readLegPosition(2)
    leg2Goal = [0.40915978, -0.23594964, -0.00924019]
    bezierTraj2, bezierVel2 = trajectoryPlanner.bezierTrajectory(leg2Start, leg2Goal, 10)
    leg5Start = motorDriver.readLegPosition(4)
    leg5Goal = [0.28647617,  0.25409757, -0.00695381]
    bezierTraj5, bezierVel5 = trajectoryPlanner.bezierTrajectory(leg5Start, leg5Goal, 10)
    result = velocityController.moveLegs([2, 4], [bezierTraj2, bezierTraj5], [bezierVel2, bezierVel5])


    # PLATFORM MOVEMENT
    # startPose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # goalPose = [0.0, 0.0, -0.2, 0.0, 0.0, 0.0]
    # trajectory, velocity = trajectoryPlanner.minJerkTrajectory(startPose, goalPose, 4)

    # result = velocityController.movePlatform(trajectory, velocity, startPose)



    

