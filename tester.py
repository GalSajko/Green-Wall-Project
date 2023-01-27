"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import controllers
from environment import wall
from environment import spider
from planning import pathplanner
from calculations import kinematics as kin
from calculations import transformations as tf
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
        udpServer.send(controller.fD)
        time.sleep(1.0 / frequency)

def initSendingThread():
    udpSendingThread = threading.Thread(target = forceSending, args = (20, ), name = 'udp_sending_thread', daemon = False)
    udpSendingThread.start()
    print("UDP thread is running.")

if __name__ == "__main__":
    pins = wall.createGrid(True)
    selectedPins = [pins[22], pins[15], pins[12], pins[24], pins[27]]
    startPose = [0.6, 0.5, 0.3, 0.0]
    plotter = sim.Plotter()

    _ = input("PRESS ENTER TO START WALKING")

    controller = controllers.VelocityController()

    udpServer = udpServer.UdpServer('192.168.1.25')
    # initSendingThread()
    time.sleep(2)

    counter = 0
    while True:
        goalPose = np.array([random.uniform(0.2, 1.0), random.uniform(0.4, 0.8), 0.3, 0.0], dtype = np.float32)
        # poses, pinsOnPath = pathplanner.createWalkingInstructions(startPose, goalPose)
        # plotter.plotSpiderMovementSteps(poses, pinsOnPath)
        controller.walk(startPose, goalPose, initBno = counter == 0)
        startPose = goalPose
        counter += 1

    # rollRotation = [0.6, 0.62, 0.3, 0.1, 0.0, 0.0]
    # pitchRotation = [0.6, 0.62, 0.3, 0.0, 0.1, 0.0]
    # yawRotation = [0.6, 0.62, 0.3, 0.0, 0.0, 0.1]
    # poses = [rollRotation, pitchRotation, yawRotation]

    # controller.moveLegsSync(spider.LEGS_IDS, selectedPins, 'g', 3, 'minJerk', startPose)
    # time.sleep(4)
    # controller.pumpsBnoArduino.resetBno()
    # time.sleep(1)
    # print(controller.pumpsBnoArduino.getRpy())
    # leg = 0
    # while True:
    #     for pose in poses:
    #         controller.moveLegsSync(spider.LEGS_IDS, selectedPins, 'g', 3, 'minJerk', pose)
    #         time.sleep(4)
    #         rpy = controller.pumpsBnoArduino.getRpy()
    #         print(rpy)

    #         globalToSpiderRotation = tf.xyzRpyToMatrix(np.array([0, 0, 0, rpy[0], rpy[1], rpy[2]]), True)
    #         globalToLegRotation = np.linalg.inv(np.dot(globalToSpiderRotation, spider.T_ANCHORS[leg][:3, :3]))
    #         globalZDirectionInLocal = np.dot(globalToLegRotation, np.array([0.0, 0.0, 1.0], dtype = np.float32))

    #         # pinToPinGlobal = pins[23] - pins[22]
    #         # pinToPinLocal = np.dot(globalToLegRotation, pinToPinGlobal)

    #         controller.grippersArduino.moveGripper(leg, 'o')
    #         time.sleep(1)
    #         controller.moveLegAsync(leg, globalZDirectionInLocal * 0.1, 'l', 3, 'minJerk', isOffset = True)
    #         time.sleep(4)
    #         controller.grippersArduino.moveGripper(leg, 'c')
    #         time.sleep(1)
    #         controller.grippersArduino.moveGripper(leg, 'o')
    #         time.sleep(1)
    #         controller.moveLegAsync(leg, -globalZDirectionInLocal * 0.1, 'l', 3, 'minJerk', isOffset = True)
    #         time.sleep(4)
    #         controller.grippersArduino.moveGripper(leg, 'c')
    #         time.sleep(1)
    #         input()
