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

    # startPose = np.array([1.0, 0.4, 0.3, 0.0], dtype = np.float32)
    # goalPose = np.array([1.0, 0.8, 0.3, 0.0], dtype = np.float32)

    startGoalPoses = [
        [np.array([0.2, 0.4, 0.3, 0.0], dtype = np.float32), np.array([1.0, 0.4, 0.3, 0.0], dtype = np.float32)],
        [np.array([1.0, 0.4, 0.3, 0.0], dtype = np.float32), np.array([1.0, 0.8, 0.3, 0.0], dtype = np.float32)],
        [np.array([1.0, 0.8, 0.3, 0.0], dtype = np.float32), np.array([0.2, 0.8, 0.3, 0.0], dtype = np.float32)],
        [np.array([0.2, 0.8, 0.3, 0.0], dtype = np.float32), np.array([0.2, 0.4, 0.3, 0.0], dtype = np.float32)],
    ]


    # poses, pinsInstructions = pathplanner.createWalkingInstructions(goalPose, startPose)

    # print(poses)
    # print(pinsInstructions)

    # plotter = sim.Plotter()
    # plotter.plotSpiderMovementSteps(poses, pinsInstructions)

    # # udpServer = udpServer.UdpServer('192.168.1.41')
    # # initSendingThread()
    
    _ = input("PRESS ENTER TO START WALKING")
    time.sleep(4)
    controller = controllers.VelocityController(True)

    for startGoalPose in startGoalPoses:
        controller.walk(startGoalPose[0], startGoalPose[1])


            

