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
    # # udpServer = udpServer.UdpServer('192.168.1.41')
    # # initSendingThread()
    
    _ = input("PRESS ENTER TO START WALKING")
    time.sleep(4)
    controller = controllers.VelocityController(True)
    
    startPose = np.array([0.6, 0.4, 0.3, 0.0], dtype = np.float32)

    counter = 0

    while True:
        goalPose = np.array([random.uniform(0.2, 1.0), random.uniform(0.4, 0.8), 0.3, 0.0], dtype = np.float32)
        print(goalPose)
        controller.walk(startPose, goalPose, doInitBno = counter == 0)

        controller.startForceMode([0, 1, 2, 3, 4], [[0.0, -1.0, 0.0]] * 5)
        time.sleep(5)
        controller.stopForceMode()

        startPose = goalPose
        counter += 1



