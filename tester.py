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
    controller = controllers.VelocityController(False)
    time.sleep(10)
    # udpServer = udpServer.UdpServer('192.168.1.8')
    # initSendingThread()

    t = 1
    controller.moveLegsSync(spider.LEGS_IDS, [[0.3, 0.0, 0.05]] * 5, 'l', 1, 'minJerk', isOffset=False)
    time.sleep(2)
    print(controller.xA[1])
    controller.startForceMode([3], [[0.0, 0.0, 0.0]])
    time.sleep(3)
    while True:
        controller.moveLegsSync([0, 1, 2, 4], [[0.45, 0.0, 0.05]] * 4, 'l', t, 'bezier', isOffset=False)
        time.sleep(t + 1)
        print(controller.xA[[0, 1, 2, 4]])
        controller.moveLegsSync([0, 1, 2, 4], [[0.3, 0.0, 0.05]] * 4, 'l', t, 'bezier', isOffset=False)
        time.sleep(t + 1)
        print(controller.xA[[0, 1, 2, 4]])
