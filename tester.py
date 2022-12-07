"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import controllers
from environment import wall
from environment import spider
from planning import pathplanner
from calculations import kinematics as kin
import udpServer as udpServer
from periphery import grippers
import simulation as sim

import time
import threading
import numpy as np
import random

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

    # poses, pinsInstructions = pathplanner.createWalkingInstructions(startPose, goalPose)
    # plotter = sim.Plotter()
    # plotter.plotSpiderMovementSteps(poses, pinsInstructions)

    # # udpServer = udpServer.UdpServer('192.168.1.41')
    # # initSendingThread()
    
    _ = input("PRESS ENTER TO START WALKING")
    time.sleep(4)
    controller = controllers.VelocityController(True)
    
    startPose = np.array([0.2, 0.4, 0.3, 0.0], dtype = np.float32)

    while True:
        goalPose = np.array([random.uniform(0.2, 1.0), random.uniform(0.4, 0.8), 0.3, 0.0], dtype = np.float32)
        print(goalPose)
        controller.walk(startPose, goalPose)
        startPose = goalPose

