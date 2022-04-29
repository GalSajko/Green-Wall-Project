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
    motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    motorDriver = dynamixel.MotorDriver(motors)

    # motorDriver.disableMotors(2)
    # motorDriver.disableMotors(4)

    # PIN TO PIN TWO LEGS MOVEMENT DEMO:
    leg2Start = motorDriver.readLegPosition(2)
    # leg2FirstGoal = [0.28823509, -0.08071822, -0.05071149]
    # leg2SecondGoal = [0.35819129, -0.15484667, -0.14786673]
    leg2ThirdGoal = [0.36920065, -0.15359186, -0.19563053]

    # leg5Start = motorDriver.readLegPosition(4)
    # leg5FirstGoal = [0.32622328, 0.13395503, -0.12944647]
    # leg5SecondGoal = [0.35927198, 0.21759289, -0.21003822]
    # leg5ThirdGoal = [0.3617698, 0.21308209, -0.23695283]


    # firstTraj, firstVel = trajectoryPlanner.minJerkTrajectory(leg2Start, leg2FirstGoal, 1)
    # first5Traj, first5Vel = trajectoryPlanner.minJerkTrajectory(leg5Start, leg5FirstGoal, 1)
    # result1 = velocityController.moveLegs([2, 4], [firstTraj, first5Traj], [firstVel, first5Vel])
    # if result1:
    #     secondTraj, secondVel = trajectoryPlanner.minJerkTrajectory(motorDriver.readLegPosition(2), leg2SecondGoal, 1)
    #     second5Traj, second5Vel = trajectoryPlanner.minJerkTrajectory(motorDriver.readLegPosition(4), leg5SecondGoal, 1)
    #     result2 = velocityController.moveLegs([2, 4], [secondTraj, second5Traj], [secondVel, second5Vel])
    #     if result2:
    #         thirdTraj, thirdVel = trajectoryPlanner.minJerkTrajectory(motorDriver.readLegPosition(2), leg2ThirdGoal, 2)
    #         third5Traj, third5Vel = trajectoryPlanner.minJerkTrajectory(motorDriver.readLegPosition(4), leg5ThirdGoal, 2)
    #         result3 = velocityController.moveLegs([2, 4], [thirdTraj, third5Traj], [thirdVel, third5Vel])


    # legsStart = [np.append(motorDriver.readLegPosition(leg), [0, 0, 0]) for leg in range(5)]
    # legsGoal = [[legsStart[leg][0], legsStart[leg][1], 0.2, 0, 0, 0] for leg in range(5)]

    # trajectory = []
    # velocity = []
    # for leg in range(5):
    #     trajectoryLeg, velocityLeg = trajectoryPlanner.minJerkTrajectory(legsStart[leg], legsGoal[leg], 1)
    #     trajectory.append(trajectoryLeg)
    #     velocity.append(velocityLeg)
    
    # legId = 2
    # result = velocityController.moveLegs([legId], [trajectory[legId]], [velocity[legId]])
    # # result = velocityController.moveLegs([0, 1, 2, 3, 4], trajectory, velocity)
    # # velocityController.moveLegs([0, 1, 2], [trajectory[0], trajectory[1], trajectory[2]], [velocity[0], velocity[1], velocity[2]])
    # if result:
    #     for i in range(5):
    #         print(motorDriver.readLegPosition(i))

    # PLATFORM MOVEMENT
    # startPose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # goalPose = [0.1, 0.1, 0.0, 0.1, 0.1, -0.1]
    # trajectory, velocity = trajectoryPlanner.minJerkTrajectory(startPose, goalPose, 4)

    # result = velocityController.movePlatform(trajectory, velocity, startPose)

    bezier = trajectoryPlanner.bezierTrajectory(leg2Start, leg2ThirdGoal, 5)
    print(bezier)


    

