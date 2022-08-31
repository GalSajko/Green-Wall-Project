"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import controllers
import calculations
import environment as env
import planning
import simulaton
import dynamixel

import time
import random
import udpServer as udpServer
import threading
import numpy as np

def forceSending(frequency):
    while True:
        udpServer.send(controller.fA)
        time.sleep(1.0 / frequency)

def initSendingThread():
    udpSendingThread = threading.Thread(target = forceSending, args = (20, ), name = 'udp_sending_thread', daemon = False)
    udpSendingThread.start()
    print("UDP thread is running.")

if __name__ == "__main__":
    # planner = planning.PathPlanner(0.05, 0.1)
    # udpServer = udpServer.UdpServer('192.168.1.8')
    controller = controllers.VelocityController()
    # initSendingThread()

    startPose = [0.6, 0.45, 0.3, 0.0]
    # rightPose = [0.65, 0.4, 0.3, 0.0]
    # bottomPose = [0.6, 0.35, 0.3, 0.0]
    # leftPose = [0.55, 0.4, 0.3, 0.0]
    # topPose = [0.6, 0.45, 0.3, 0.0]
    # path = planner.calculateSpiderBodyPath(startPose, startPose)
    # pathPins = planner.calculateSelectedPins(path)
    pins = env.Wall('squared').createGrid(True)
    # startPins = pathPins[0]
    testPins = np.array([pins[22], pins[9], pins[7], pins[31], pins[33]])

    testLeg = 3
    newPin = pins[30]

    input("PRESS ENTER TO MOVE SPIDER: ")
    controller.moveLegsSync([0, 1, 2, 3, 4], testPins, 'g', 5, 'minJerk', startPose)

    input("PRESS ENTER TO OFFLOAD THE LEG: ")
    controller.offloadSelectedLeg(testLeg, testPins)

    # angles = controller.readLegsPositionsWrapper([3], 'l', returnAngles = True)[1]
    # print(angles)
    # input("PRESS ENTER TO MOVE THE LEG: ")
    # controller.moveGrippersWrapper([testLeg], 'o')
    # controller.moveLegAsync(testLeg, [0.0, 0.0, 0.05], 'g', 2, 'minJerk', startPose, True)
    # time.sleep(2.2)
    # angles = controller.readLegsPositionsWrapper([3], 'l', returnAngles = True)[1]
    # print(angles)
    # controller.moveLegAsync(testLeg, [0.0, -0.25, 0.0], 'g', 5, 'minJerk', startPose, True)
    # time.sleep(5.2)
    # controller.moveLegAsync(testLeg, [0.0, 0.0, -0.05], 'g', 2, 'minJerk', startPose, True)
    # time.sleep(2.2)
    # controller.moveGrippersWrapper([testLeg], 'c')





