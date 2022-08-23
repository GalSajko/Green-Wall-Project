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
    # kinematics = calculations.Kinematics()
    controller = controllers.VelocityController()
    planner = planning.PathPlanner(0.05, 0.1)

    startPose = [0.6, 0.4, 0.3, 0.0]
    rightPose = [0.65, 0.4, 0.3, 0.0]
    bottomPose = [0.6, 0.35, 0.3, 0.0]
    leftPose = [0.55, 0.4, 0.3, 0.0]
    topPose = [0.6, 0.45, 0.3, 0.0]
    path = planner.calculateSpiderBodyPath(startPose, startPose)
    startPins = planner.calculateSelectedPins(path)[0]

    print(startPins)

    _ = input("CONFIRM STARTING PINS AND PRESS ANY KEY:")

    controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 5, 'minJerk', startPose)
    time.sleep(7)

    while True:
        controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 3, 'minJerk', topPose)
        time.sleep(4)
        controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 3, 'minJerk', rightPose)
        time.sleep(4)
        controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 3, 'minJerk', bottomPose)
        time.sleep(4)
        controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 3, 'minJerk', leftPose)
        time.sleep(4)

    # while True:
    #     legIdInput = input("Enter leg ID: ")
    #     while legIdInput not in ['0', '1', '2', '3', '4']:
    #         legIdInput = input("Invalid leg ID, enter again: ")
    #     legIdInput = int(legIdInput)
    #     localGoalPositionInputString = input("Enter XYZ goal position in local origin, separated by comma: ")
    #     localGoalPositionInputStringArray = localGoalPositionInputString.split(',')
    #     localGoalPosition = []
    #     for value in localGoalPositionInputStringArray:
    #         try:
    #             localGoalPosition.append(float(value))
    #         except:
    #             print(f"Cannot convert {value} to float.")
    #     while True:
    #         durationInput = input("Enter movement duration in seconds: ")
    #         try:
    #             durationInput = float(durationInput)
    #             break
    #         except:
    #             print("Cannot convert given duration to float.")
    #     controller.moveLegAsync(legIdInput, localGoalPosition, 'l', durationInput, 'minJerk')
    #     time.sleep(durationInput)




    