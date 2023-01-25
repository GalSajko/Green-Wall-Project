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
    selectedPins = [pins[22], pins[9], pins[7], pins[31], pins[33]]
    startPose = [0.6, 0.5, 0.3, 0.0]
    poses = [
        [0.75, 0.5, 0.3, 0.0],
        [0.6, 0.6, 0.3, 0.0],
        [0.45, 0.5, 0.3, 0.0],
        [0.6, 0.4, 0.3, 0.0],
        [0.6, 0.5, 0.2, 0.0],
        [0.6, 0.5, 0.4, 0.0]
    ]
    
    print(kin.getXdXddFromOffsets.nopython_signatures)
    _ = input("PRESS ENTER TO START WALKING")

    

    # controller = controllers.VelocityController(False)
    # # controller.distributeForces(spider.LEGS_IDS, 0)
    # # udpServer = udpServer.UdpServer('192.168.1.25')
    # # initSendingThread()
    # time.sleep(10)


    # controller.moveLegsSync(spider.LEGS_IDS, selectedPins, 'g', 3, 'minJerk', startPose)
    # time.sleep(6)
    # controller.pumpsBnoArduino.resetBno()
    # time.sleep(2)
    # # controller.grippersArduino.moveGripper(0, 'o')
    # # time.sleep(1)
    # # controller.moveLegAsync(0, [0.0, 0.0, 0.1], 'l', 1, 'minJerk', isOffset = True)
    # # time.sleep(3)
    # controller.distributeForces(spider.LEGS_IDS, 1)
    # time.sleep(2)



    # while True:
    #     for pose in poses:
    #         controller.moveLegsSync(spider.LEGS_IDS, selectedPins, 'g', 2, 'minJerk', pose)
    #         time.sleep(3.5)
    #         controller.distributeForces(spider.LEGS_IDS, 1)
    #         time.sleep(2)
