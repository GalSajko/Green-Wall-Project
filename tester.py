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
        udpServer.send(controller.currents)
        time.sleep(1.0 / frequency)

def initSendingThread():
    udpSendingThread = threading.Thread(target = forceSending, args = (20, ), name = 'udp_sending_thread', daemon = False)
    udpSendingThread.start()
    print("UDP thread is running.")

if __name__ == "__main__":
    _ = input("PRESS ENTER TO START WALKING")
    time.sleep(4)
    controller = controllers.VelocityController(True)

    # udpServer = udpServer.UdpServer('192.168.1.8')
    # initSendingThread()
    
    startPose = np.array([0.6, 0.4, 0.3, 0.0], dtype = np.float32)

    # controller.moveLegAsync(1, [0.3, 0.0, 0.0], 'l', 1, 'minJerk', isOffset=False)
    controller.moveLegsSync(spider.LEGS_IDS, [[0.3, 0.0, 0.0]] * 5, 'l', 1, 'minJerk', isOffset=False)
    time.sleep(3)
    # controller.moveLegAsync(1, [0.4, 0.0, 0.1], 'l', 3, 'minJerk', isOffset=False)
    # time.sleep(2)
    # controller.moveLegAsync(1, [0.3, 0.0, 0.15], 'l', 3, 'minJerk', isOffset=False)
    # time.sleep(4)

    while True:
        controller.moveLegAsync(1, [0.4, 0.0, 0.0], 'l', 3.5, 'bezier', isOffset=False)
        time.sleep(5.5)
        print(controller.xA[1])
        input("ENTER [0.4, 0.0, 0.0]")
        controller.moveLegAsync(1, [0.3, 0.0, 0.0], 'l', 3.5, 'bezier', isOffset=False)
        time.sleep(5.5)
        print(controller.xA[1])
        input("ENTER [0.3, 0.0, 0.0]")

    # while True:
    #     controller.moveLegsSync(spider.LEGS_IDS, [[0.4, 0.0, 0.1]] * 5, 'l', 1.5, 'minJerk', isOffset=False)
    #     time.sleep(2.5)
    #     print(controller.xA)
    #     input("ENTER [0.4, 0.0, 0.1]")
    #     controller.moveLegsSync(spider.LEGS_IDS, [[0.3, 0.0, 0.0]] * 5, 'l', 1.5, 'minJerk', isOffset=False)
    #     time.sleep(2.5)
    #     print(controller.xA)
    #     input("ENTER [0.3, 0.0, 0.0]")
    #     controller.moveLegsSync(spider.LEGS_IDS, [[0.55, -0.1, 0.15]] * 5, 'l', 2.5, 'minJerk', isOffset=False)
    #     time.sleep(3.5)
    #     print(controller.xA)
    #     input("ENTER [0.55, -0.1, 0.15]")
    #     controller.moveLegsSync(spider.LEGS_IDS, [[0.55, 0.1, 0.0]] * 5, 'l', 1.5, 'minJerk', isOffset=False)
    #     time.sleep(2.5)
    #     print(controller.xA)
    #     input("ENTER [0.55, 0.1, 0.0]")
