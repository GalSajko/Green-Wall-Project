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
    plotter = sim.Plotter('squared')
    velocityController = controllers.VelocityController()
    pathPlanner = planning.PathPlanner(0.05, 0.2, 'squared')
    trajPlanner = planning.TrajectoryPlanner()
    kinematics = calculations.Kinematics()

    # LEGS MOVEMENT
    motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    motorDriver = dynamixel.MotorDriver(motors)
    # motorDriver.disableLegs(5)

    spiderStartPose = [0.4, 0.33, spider.LYING_HEIGHT, 0]
    spiderGoalPose = [0.4, 1, spider.WALKING_HEIGHT, 0]

    # path = pathPlanner.calculateSpiderBodyPath(spiderStartPose, spiderGoalPose)
    # pins = pathPlanner.calculateSpiderLegsPositionsXyzRpyFF(path)
    path, pins = pathPlanner.calculateWalkingMovesFF(spiderStartPose, spiderGoalPose)
    # plotter.plotSpiderMovement(path, pins)

    # velocityController.walk(spiderStartPose, spiderGoalPose)
    legs = [0, 1]
    motorDriver.addGroupSyncReadParams(legs)
    jointsValues = motorDriver.syncReadMotorsPositionsInLegs(legs)

    kinematics.platformDirectKinematics(legs, [pins[0][0], pins[0][1]], jointsValues)

    # velocityController.walk(spiderGoalPose, spiderStartPose, False)

    # velocityController.movePlatformWrapper([0.4, 0.33, 0.2, 0, 0, 0], spiderStartPose, 4)

    # path = pathPlanner.calculateSpiderBodyPath(spiderStartPose[:2], spiderGoalPose[:2], 0.05)
    # bestParams = [0.2 , 0.4, 0.4]
    # selectedPins = pathPlanner.calculateSpiderLegsPositionsFF(path, bestParams)
    # plotter.plotSpiderMovement(path, selectedPins)


    
<<<<<<< HEAD
    motorsIds = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    motorDriver = dynamixel.MotorDriver(motorsIds)
    # motorDriver.disableMotors()

    for motorId in np.array(motorsIds).flatten():
        motorDriver.setVelocityProfile(motorId, 1, 500)

    # Show parallel movement.
    motorDriver.enableMotors()
    startPose = [0, 0, 0.04, 0, 0, 0]
    # Write any goal pose (rpy included).
    goalPoses = [
        [0, 0, 0.2, 0, 0, 0],
        [0.1, 0.1, 0.2, 0.2, 0.2, 0.2],
        [-0.1, -0.1, 0.2, -0.2, -0.2, -0.2],
        [-0.1, 0.1, 0.2, 0.2, 0.2, 0.2], 
        [0.1, -0.1, 0.2, -0.2, -0.2, -0.2],
        [0, 0, 0.2, 0, 0, 0],
        # startPose
    ]

    for idx, goalPose in enumerate(goalPoses):
        if idx != 0:
            startPose = goalPoses[idx - 1]

        startLegPositions = [motorDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
        T_GS = matrixCalculator.transformMatrixFromGlobalPose(startPose)
        if idx == 0:
            pins = matrixCalculator.getPinsInGlobal(spider.T_ANCHORS, T_GS, startLegPositions, startPose)

        trajectory = trajectoryPlanner.platformLinearTrajectory(startPose, goalPose)
        motorDriver.movePlatformTrajectory(trajectory, pins)

    # motorDriver.disableMotors()
    #endregion    

    #region WALKING

    # Stand up and move platform 5 cm forward.
    time.sleep(1)
    # motorDriver.enableMotors()
    startPose = [0, 0, 0.2, 0, 0, 0]
    goalPoses = [
        # [0, 0, 0.2, 0, 0, 0],
        [0.0, 0.05, 0.2, 0, 0, 0]]
    for idx, goalPose in enumerate(goalPoses):
        if idx != 0:
            startPose = goalPoses[idx - 1]

        if idx == 0:
            startLegPositions = [motorDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
            T_GS = matrixCalculator.transformMatrixFromGlobalPose(startPose)
            pins = matrixCalculator.getPinsInGlobal(spider.T_ANCHORS, T_GS, startLegPositions, startPose)

        trajectory = trajectoryPlanner.platformLinearTrajectory(startPose, goalPose)
        motorDriver.movePlatformTrajectory(trajectory, pins)
    
    # Move legs 3 and 5 in direction of movement.
    currentLegPositions = [motorDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
    trajectoryLeg3 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[2], [0.25, -0.08, currentLegPositions[2][2]])
    trajectoryLeg5 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[4], [0.35, 0.08, currentLegPositions[4][2]])
    for position in trajectoryLeg3:
        motorDriver.moveLegs([2], [position])
    for position in trajectoryLeg5:
        motorDriver.moveLegs([4], [position])

    # Move legs 2 and 4 in direction of movement.
    trajectoryLeg2 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[1], [0.35, -0.08, currentLegPositions[1][2]])
    trajectoryLeg4 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[3], [0.25, 0.08, currentLegPositions[3][2]])
    for position in trajectoryLeg2:
        motorDriver.moveLegs([1], [position])
    for position in trajectoryLeg4:
        motorDriver.moveLegs([3], [position])
    
    # Move leg 1 in direciton of movement.
    trajectoryLeg1 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[0], [0.45, 0, currentLegPositions[0][2]])
    for position in trajectoryLeg1:
        motorDriver.moveLegs([0], [position])
    
    # Move platform for 10 cm forward.
    startPose = [0, 0.05, 0.2, 0, 0, 0]
    goalPoses = [
        [0, 0.15, 0.2, 0, 0, 0]]
    for idx, goalPose in enumerate(goalPoses):
        if idx != 0:
            startPose = goalPoses[idx - 1]

        if idx == 0:
            startLegPositions = [motorDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
            T_GS = matrixCalculator.transformMatrixFromGlobalPose(startPose)
            pins = matrixCalculator.getPinsInGlobal(spider.T_ANCHORS, T_GS, startLegPositions, startPose)

        trajectory = trajectoryPlanner.platformLinearTrajectory(startPose, goalPose)
        motorDriver.movePlatformTrajectory(trajectory, pins)

    ###############
    
    # Move legs 3 and 5 in direction of movement.
    currentLegPositions = [motorDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
    trajectoryLeg3 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[2], [0.25, -0.08, currentLegPositions[2][2]])
    trajectoryLeg5 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[4], [0.35, 0.08, currentLegPositions[4][2]])
    for position in trajectoryLeg3:
        motorDriver.moveLegs([2], [position])
    for position in trajectoryLeg5:
        motorDriver.moveLegs([4], [position])

    # Move legs 2 and 4 in direction of movement.
    trajectoryLeg2 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[1], [0.35, -0.08, currentLegPositions[1][2]])
    trajectoryLeg4 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[3], [0.25, 0.08, currentLegPositions[3][2]])
    for position in trajectoryLeg2:
        motorDriver.moveLegs([1], [position])
    for position in trajectoryLeg4:
        motorDriver.moveLegs([3], [position])
    
    # Move leg 1 in direciton of movement.
    trajectoryLeg1 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[0], [0.45, 0, currentLegPositions[0][2]])
    for position in trajectoryLeg1:
        motorDriver.moveLegs([0], [position])
    
    # Move platform for 10 cm forward.
    startPose = [0, 0.15, 0.2, 0, 0, 0]
    goalPoses = [
        [0, 0.25, 0.2, 0, 0, 0]]
    for idx, goalPose in enumerate(goalPoses):
        if idx != 0:
            startPose = goalPoses[idx - 1]

        if idx == 0:
            startLegPositions = [motorDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
            T_GS = matrixCalculator.transformMatrixFromGlobalPose(startPose)
            pins = matrixCalculator.getPinsInGlobal(spider.T_ANCHORS, T_GS, startLegPositions, startPose)

        trajectory = trajectoryPlanner.platformLinearTrajectory(startPose, goalPose)
        motorDriver.movePlatformTrajectory(trajectory, pins)
    
    #########
    
    # Move legs 3 and 5 in direction of movement.
    currentLegPositions = [motorDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
    trajectoryLeg3 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[2], [0.25, -0.08, currentLegPositions[2][2]])
    trajectoryLeg5 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[4], [0.35, 0.08, currentLegPositions[4][2]])
    for position in trajectoryLeg3:
        motorDriver.moveLegs([2], [position])
    for position in trajectoryLeg5:
        motorDriver.moveLegs([4], [position])

    # Move legs 2 and 4 in direction of movement.
    trajectoryLeg2 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[1], [0.35, -0.08, currentLegPositions[1][2]])
    trajectoryLeg4 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[3], [0.25, 0.08, currentLegPositions[3][2]])
    for position in trajectoryLeg2:
        motorDriver.moveLegs([1], [position])
    for position in trajectoryLeg4:
        motorDriver.moveLegs([3], [position])
    
    # Move leg 1 in direciton of movement.
    trajectoryLeg1 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[0], [0.45, 0, currentLegPositions[0][2]])
    for position in trajectoryLeg1:
        motorDriver.moveLegs([0], [position])
    
    # Move platform 10cm forward.
    startPose = [0, 0.25, 0.2, 0, 0, 0]
    goalPoses = [
        [0, 0.35, 0.2, 0, 0, 0]]
    for idx, goalPose in enumerate(goalPoses):
        if idx != 0:
            startPose = goalPoses[idx - 1]

        if idx == 0:
            startLegPositions = [motorDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
            T_GS = matrixCalculator.transformMatrixFromGlobalPose(startPose)
            pins = matrixCalculator.getPinsInGlobal(spider.T_ANCHORS, T_GS, startLegPositions, startPose)

        trajectory = trajectoryPlanner.platformLinearTrajectory(startPose, goalPose)
        motorDriver.movePlatformTrajectory(trajectory, pins)

    #########
    
    # Move legs 3 and 5 in direction of movement.
    currentLegPositions = [motorDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
    trajectoryLeg3 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[2], [0.25, -0.08, currentLegPositions[2][2]])
    trajectoryLeg5 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[4], [0.35, 0.08, currentLegPositions[4][2]])
    for position in trajectoryLeg3:
        motorDriver.moveLegs([2], [position])
    for position in trajectoryLeg5:
        motorDriver.moveLegs([4], [position])

    # Move legs 2 and 4 in direction of movement.
    trajectoryLeg2 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[1], [0.35, -0.08, currentLegPositions[1][2]])
    trajectoryLeg4 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[3], [0.25, 0.08, currentLegPositions[3][2]])
    for position in trajectoryLeg2:
        motorDriver.moveLegs([1], [position])
    for position in trajectoryLeg4:
        motorDriver.moveLegs([3], [position])
    
    # Move leg 1 in direciton of movement.
    trajectoryLeg1 = trajectoryPlanner.legCircularTrajectory(currentLegPositions[0], [0.45, 0, currentLegPositions[0][2]])
    for position in trajectoryLeg1:
        motorDriver.moveLegs([0], [position])
 
    # Move platform 10cm forward.
    startPose = [0, 0.35, 0.2, 0, 0, 0]
    goalPoses = [
        [0, 0.35, 0.04, 0, 0, 0]]
    for idx, goalPose in enumerate(goalPoses):
        if idx != 0:
            startPose = goalPoses[idx - 1]

        if idx == 0:
            startLegPositions = [motorDriver.readLegPosition(legId) for legId in range(spider.NUMBER_OF_LEGS)]
            T_GS = matrixCalculator.transformMatrixFromGlobalPose(startPose)
            pins = matrixCalculator.getPinsInGlobal(spider.T_ANCHORS, T_GS, startLegPositions, startPose)

        trajectory = trajectoryPlanner.platformLinearTrajectory(startPose, goalPose)
        motorDriver.movePlatformTrajectory(trajectory, pins)
    
    time.sleep(1)
    motorDriver.disableMotors()
    #endregion 
=======





    
>>>>>>> rpi_python3

