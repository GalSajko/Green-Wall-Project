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

    startPose = np.array([0.8, 0.4, 0.3, 0.0], dtype = np.float32)
    endPose = np.array([0.4, 0.8, 0.3, 0.0], dtype = np.float32)

    poses, pinsInstructions = pathplanner.createWalkingInstructions(endPose, startPose)

    print(pinsInstructions)
    # print(poses)
    # print(pins)

    plotter = sim.Plotter()
    plotter.plotSpiderMovementSteps(poses, pinsInstructions)



    # # udpServer = udpServer.UdpServer('192.168.1.41')
    # # initSendingThread()


    # # leg = 0
    # # pins = pinsInstructions[:, :, 1:]
    # legs = [0, 1, 4, 2, 3]
    # positions = [[0.4, 0.75, 0.0], [0.2, 0.75, 0.0], [0.6, 0.75, 0.0], [0.2, 0.0, 0.0], [0.4, 0.0, 0.0]]
    # controller.moveLegsSync(legs, positions, 'g', 3, 'minJerk', poses[0])
    # # time.sleep(5)

    # for leg in spider.LEGS_IDS:
    #     controller.gripperController.moveGripper(leg, 'o')

    
    _ = input("PRESS ENTER TO START WALKING")
    time.sleep(4)
    controller = controllers.VelocityController(True)
    for step, pose in enumerate(poses):
        currentPinsPositions = pinsInstructions[step, :, 1:]
        currentLegsMovingOrder = pinsInstructions[step, :, 0].astype(int)

        if step == 0:
            controller.moveLegsSync(currentLegsMovingOrder, currentPinsPositions, 'g', 3, 'minJerk', pose)
            time.sleep(3.5)
            controller.distributeForces(spider.LEGS_IDS, 10)
            continue
        
        previusPinsPositions = np.array(pinsInstructions[step - 1, :, 1:])
        controller.moveLegsSync(currentLegsMovingOrder, previusPinsPositions, 'g', 3, 'minJerk', pose)
        time.sleep(3.5)

        pinsOffsets = currentPinsPositions - previusPinsPositions
        for idx, leg in enumerate(currentLegsMovingOrder):
            if pinsOffsets[idx].any():
                controller.distributeForces(np.delete(spider.LEGS_IDS, leg), 2)
                time.sleep(2.2)
                controller.gripperController.moveGripper(leg, controller.gripperController.OPEN_COMMAND)
                time.sleep(1)
                controller.moveLegAsync(leg, [0.0, 0.0, 0.05], 'g', 1, 'minJerk', pose, True)
                time.sleep(1.5)
                controller.moveLegAsync(leg, pinsOffsets[idx], 'g', 3, 'minJerk', pose, True)
                time.sleep(3.5)
                controller.moveLegAsync(leg, [0.0, 0.0, -0.05], 'g', 1, 'minJerk', pose, True)
                time.sleep(1.5)
                controller.gripperController.moveGripper(leg, controller.gripperController.CLOSE_COMMAND)
                time.sleep(1)                   
        controller.distributeForces(spider.LEGS_IDS, 3)
        time.sleep(3.5)

        
        

