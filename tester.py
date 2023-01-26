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
import numba

def forceSending(frequency):
    while True:
        udpServer.send(controller.currents)
        time.sleep(1.0 / frequency)

def initSendingThread():
    udpSendingThread = threading.Thread(target = forceSending, args = (20, ), name = 'udp_sending_thread', daemon = False)
    udpSendingThread.start()
    print("UDP thread is running.")

if __name__ == "__main__":
    pins = wall.createGrid(True)
    selectedPins = [pins[22], pins[15], pins[12], pins[24], pins[27]]
    startPose = [0.6, 0.5, 0.3, 0.0]
    poses = [
        [0.7, 0.5, 0.3, 0.0],
        [0.6, 0.6, 0.3, 0.0],
        [0.5, 0.5, 0.3, 0.0],
        [0.6, 0.4, 0.3, 0.0],
    ]

    _ = input("PRESS ENTER TO START WALKING")

    # kin.getXdXddFromOffsets(spider.LEGS_IDS, np.zeros((5, 3), dtype = np.float32), np.zeros((5, 3), dtype = np.float32))

    controller = controllers.VelocityController()
    # controller.distributeForces(spider.LEGS_IDS, 0)
    udpServer = udpServer.UdpServer('192.168.1.25')
    time.sleep(2)


    counter = 0
    while True:
        goalPose = np.array([random.uniform(0.2, 1.0), random.uniform(0.4, 0.8), 0.3, 0.0], dtype = np.float32)
        print(goalPose)
        controller.walk(startPose, goalPose, initBno = counter == 0)
        startPose = goalPose
        counter += 1

