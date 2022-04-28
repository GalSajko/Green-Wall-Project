"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import numpy as np

import calculations
import dynamixel
import planning
import controllers

if __name__ == "__main__":
    velocityController = controllers.VelocityController()
    matrixCalculator = calculations.MatrixCalculator()
    trajectoryPlanner = planning.TrajectoryPlanner()

    # LEGS MOVEMENT
    # motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    # motorDriver = dynamixel.MotorDriver(motors)

    # legsStart = [np.append(motorDriver.readLegPosition(leg), [0, 0, 0]) for leg in range(5)]
    # legsGoal = [[legsStart[leg][0], legsStart[leg][1], -0.035, 0, 0, 0] for leg in range(5)]

    # trajectory = []
    # velocity = []
    # for leg in range(5):
    #     trajectoryLeg, velocityLeg = trajectoryPlanner.minJerkTrajectory(legsStart[leg], legsGoal[leg], 1)
    #     trajectory.append(trajectoryLeg)
    #     velocity.append(velocityLeg)
    
    # legId = 3
    # # velocityController.moveLegs([legId], [trajectory[legId]], [velocity[legId]])
    # result = velocityController.moveLegs([0, 1, 2, 3, 4], trajectory, velocity)
    # # velocityController.moveLegs([0, 1, 2], [trajectory[0], trajectory[1], trajectory[2]], [velocity[0], velocity[1], velocity[2]])
    # if result:
    #     for i in range(5):
    #         print(motorDriver.readLegPosition(i))

    # PLATFORM MOVEMENT
    startPose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goalPose = [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
    trajectory, velocity = trajectoryPlanner.minJerkTrajectory(startPose, goalPose, 5)

    velocityController.movePlatform(trajectory, velocity, startPose)


    

