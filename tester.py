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
from periphery import waterpumpsbno as wpb
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
    # pinsInstructions = wall.createGrid(True)

    # _ = input("PRESS ENTER TO RESET BNO")
    # bno.initBno()

    # print("Start receiving")
    # while True:
    #     rpy, grav = bno.getRpyAndGravity()
    #     print(rpy)
    #     print(grav)
    #     print("====")
    #     time.sleep(0.05)
    


    

    # poses, pinsInstructions = pathplanner.createWalkingInstructions(startPose, goalPose)
    # plotter = sim.Plotter()
    # plotter.plotSpiderMovementSteps(poses, pinsInstructions)

    # # udpServer = udpServer.UdpServer('192.168.1.41')
    # # initSendingThread()
    
    _ = input("PRESS ENTER TO START WALKING")
    time.sleep(4)
    controller = controllers.VelocityController(True)
    
    startPose = np.array([0.6, 0.4, 0.3, 0.0], dtype = np.float32)

    # pins = wall.createGrid(True)
    # controller.moveLegsSync([0, 1, 2, 3, 4], [pins[22], pins[15], pins[12], pins[24], pins[27]], 'g', 5, 'minJerk', startPose)
    # time.sleep(5.5)

    # controller.distributeForces([1, 2, 3, 4], 5)
    # time.sleep(6)
    # controller.grippersArduino.moveGripper(0, 'o')
    # controller.startForceMode([0], [[0.0, 0.0, 0.0]])
    # while True:
    #     controller.startForceMode([0], [[0.0, 0.0, 5.0]])
    #     time.sleep(2)
    #     controller.startForceMode([0], [[0.0, 0.0, -5.0]])
    #     time.sleep(2)

    counter = 0

    while True:
        goalPose = np.array([random.uniform(0.2, 1.0), random.uniform(0.4, 0.8), 0.3, 0.0], dtype = np.float32)
        print(goalPose)
        controller.walk(startPose, goalPose, doInitBno = counter == 0)
        startPose = goalPose
        counter += 1



