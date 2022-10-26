"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import controllers
from environment import wall
from environment import spider
import udpServer as udpServer

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
    startPins = [pins[22], pins[9], pins[7], pins[31], pins[33]]
    controller = controllers.VelocityController(True)
    udpServer = udpServer.UdpServer('192.168.1.41')
    initSendingThread()

    legId = 2
    controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 3, 'minJerk', [0.6, 0.5, 0.3, 0.0])
    time.sleep(5)
    _ = input("PRESS ENTER TO START FORCE CONTROLL")
    controller.startForceDistribution(np.array([0, 1, 3, 4]), duration = 10)

    # _ = input("PRESS ENTER TO OFFLOAD LEG AGAIN")
    # controller.startForceMode([0], [np.zeros(3)])
    # time.sleep(2)
    # controller.stopForceMode()
    _ = input("PRESS ENTER TO MOVE LEG")
    controller.moveLegAsync(legId, [0.0, 0.0, 0.05], 'g', 2, 'minJerk', [0.6, 0.5, 0.3, 0.0], True)
    time.sleep(2)
    controller.moveLegAsync(legId, [0.0, 0.25, 0.0], 'g', 3, 'minJerk', [0.6, 0.5, 0.3, 0.0], True)
    time.sleep(3)
    controller.moveLegAsync(legId, [0.0, 0.0, -0.05], 'g', 2, 'minJerk', [0.6, 0.5, 0.3, 0.0], True)
    time.sleep(5)
    controller.startForceDistribution(spider.LEGS_IDS, duration = 60)
    