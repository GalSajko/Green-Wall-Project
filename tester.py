"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import controllers
from environment import wall
from environment import spider
from planning import pathplanner
import udpServer as udpServer
from periphery import grippers
import simulation as sim

import time
import threading
import numpy as np

def forceSending(frequency):
    while True:
        udpServer.send(controller.fAMean)
        time.sleep(1.0 / frequency)

def initSendingThread():
    udpSendingThread = threading.Thread(target = forceSending, args = (20, ), name = 'udp_sending_thread', daemon = False)
    udpSendingThread.start()
    print("UDP thread is running.")

if __name__ == "__main__":
    pinsInstructions = wall.createGrid(True)

    startPose = np.array([0.3, 0.7, 0.3, 0.0], dtype = np.float32)
    endPose = np.array([0.8, 0.7, 0.3, 0.0], dtype = np.float32)

    poses, pinsInstructions = pathplanner.createWalkingInstructions(startPose, endPose)

    print(pinsInstructions)
    # print(poses)
    # print(pins)

    plotter = sim.Plotter()
    plotter.plotSpiderMovementSteps(poses, pinsInstructions)

    # controller = controllers.VelocityController()

    # # udpServer = udpServer.UdpServer('192.168.1.41')
    # # initSendingThread()


    # # leg = 0
    # # pins = pinsInstructions[:, :, 1:]
    # # controller.moveLegsSync([0, 1, 2, 3, 4], pins[0], 'g', 3, 'minJerk', poses[0])
    # # time.sleep(5)

    # # for leg in [0, 1, 2, 3, 4]:
    # #     controller.gripperController.moveGripper(leg, 'o')

    
    # _ = input("PRESS ENTER TO START WALKING")
    # for step, pose in enumerate(poses):
    #     currentPinsPositions = pinsInstructions[step, :, 1:]
    #     currentLegsMovingOrder = pinsInstructions[step, :, 0]

    #     if step == 0:
    #         # controller.moveLegsSync(spider.LEGS_IDS, currentPinsPositions, 'g', 5, 'minJerk', pose)
    #         # time.sleep(5.5)
    #         # controller.distributeForces(spider.LEGS_IDS, 10)
    #         continue
        
    #     previusPinsPositions = pinsInstructions[step - 1, :, 1:]
    #     # controller.moveLegsSync(spider.LEGS_IDS, previusPinsPositions, 'g', 5, 'minJerk', pose)
    #     # time.sleep(5.5)

    #     for instruction in pinsInstructions[1:, :, :]:
    #         for i in range(len(spider.LEGS_IDS)):
    #             legToMove = int(instruction[i][0])
    #             goalPosition = instruction[i][1:]
    #             pinOffset = goalPosition - previusPinsPositions[legToMove]
    #             print(legToMove)
    #             print(goalPosition)
    #             print(pinOffset)
                    
    #             # if pinOffset.any():
    #             #     controller.gripperController.moveGripper(legToMove, controller.gripperController.OPEN_COMMAND)
    #             #     time.sleep(1)
    #             #     controller.moveLegAsync(legToMove, [0.0, 0.0, 0.05], 'g', 1, 'minJerk', pose, True)
    #             #     time.sleep(1)
    #             #     controller.moveLegAsync(legToMove, pinOffset, 'g', 3, 'minJerk', pose, True)
    #             #     time.sleep(3)
    #             #     controller.moveLegAsync(legToMove, [0.0, 0.0, -0.05], 'g', 1, 'minJerk', pose, True)
    #             #     time.sleep(1)
    #             #     controller.gripperController.moveGripper(legToMove, controller.gripperController.CLOSE_COMMAND)
    #             #     time.sleep(1)                   
    #     # controller.distributeForces(spider.LEGS_IDS, 3)
    #     # time.sleep(3.5)

        
        

