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

    controller.moveLegsSync(spider.LEGS_IDS, [[0.0, 0.0, 0.05]] * 5, 'l', 1, 'minJerk', isOffset=True)
    time.sleep(2)
    controller.grippersArduino.moveGripper(0, 'o')
    controller.startForceMode([3], [np.zeros(3, dtype = np.float32)])
    while True:
        controller.moveLegAsync(1, [0.2, 0.0, 0.0], 'l', 1, 'minJerk', isOffset=True)
        time.sleep(3)
        controller.moveLegAsync(1, [-0.2, 0.0, 0.0], 'l', 1, 'minJerk', isOffset=True)
        time.sleep(3)
