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
    pins = wall.createGrid(True)

    startPose = np.array([0.6, 0.3, 0.3, 0.0], dtype = np.float32)
    endPose = np.array([0.6, 0.85, 0.3, 0.0], dtype = np.float32)

    poses, pins = pathplanner.createWalkingInstructions(startPose, endPose)

    # print(poses)
    # print(pins)

    # plotter = sim.Plotter()
    # plotter.plotSpiderMovementSteps(poses, pins)

    # controller = controllers.VelocityController(True)
    gripper = grippers.GripperController()
    # udpServer = udpServer.UdpServer('192.168.1.41')
    # initSendingThread()
    while True:
        for i in spider.LEGS_IDS:
            gripper.moveGripper(i, 'o')
            time.sleep(1)
            gripper.moveGripper(i, 'c')
            time.sleep(1)

    # leg = 0
    # controller.moveLegsSync([0, 1, 2, 3, 4], pins[0], 'g', 3, 'minJerk', poses[0])
    # time.sleep(5)

    # _ = input("PRESS ENTER TO START WALKING")
    # for step, pose in enumerate(poses):
    #     if step == 0:
    #         controller.moveLegsSync(spider.LEGS_IDS, pins[step], 'g', 5, 'minJerk', pose)
    #         time.sleep(5.5)
    #         controller.distributeForces(spider.LEGS_IDS, 10)
    #         continue

    #     controller.moveLegsSync(spider.LEGS_IDS, pins[step - 1], 'g', 5, 'minJerk', pose)
    #     time.sleep(5.5)
    #     pinsOffsets = pins[step] - pins[step - 1]
    #     for leg, pinsOffset in enumerate(pinsOffsets):
    #         if pinsOffset.any():
    #             controller.distributeForces(np.delete(spider.LEGS_IDS, leg), 2)
    #             time.sleep(3)
    #             gripper.moveGripper(leg, gripper.OPEN_COMMAND)
    #             time.sleep(1)
    #             controller.moveLegAsync(leg, [0.0, 0.0, 0.05], 'g', 1, 'minJerk', poses[step], True)
    #             time.sleep(1)
    #             controller.moveLegAsync(leg, pinsOffset, 'g', 3, 'minJerk', poses[step], True)
    #             time.sleep(3)
    #             controller.moveLegAsync(leg, [0.0, 0.0, -0.05], 'g', 1, 'minJerk', poses[step], True)
    #             time.sleep(1)
    #             gripper.moveGripper(leg, gripper.CLOSE_COMMAND)
    #             time.sleep(1)
    #     controller.distributeForces(spider.LEGS_IDS, 3)
    #     time.sleep(3.5)


        
        

