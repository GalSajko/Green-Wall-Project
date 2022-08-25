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
    # time.sleep(2)
    planner = planning.PathPlanner(0.05, 0.1)
    udpServer = udpServer.UdpServer('192.168.1.8')
    controller = controllers.VelocityController()
    initSendingThread() 

    startPose = [0.6, 0.4, 0.3, 0.0]
    # rightPose = [0.65, 0.4, 0.3, 0.0]
    # bottomPose = [0.6, 0.35, 0.3, 0.0]
    # leftPose = [0.55, 0.4, 0.3, 0.0]
    # topPose = [0.6, 0.45, 0.3, 0.0]
    # path = planner.calculateSpiderBodyPath(startPose, startPose)
    # pathPins = planner.calculateSelectedPins(path)
    pins = env.Wall('squared').createGrid(True)
    # startPins = pathPins[0]
    testPins = [pins[22], pins[9], pins[7], pins[31], pins[33]]
    
    print("STARTING PINS:\n ", testPins)

    # _ = input("CONFIRM STARTING PINS AND PRESS ENTER:")

    controller.moveLegsSync([0, 1, 2, 3, 4], [[0.35, 0.0, 0.15]] * 5, 'l', 5, 'minJerk')
    _ = input("PRESS TO MEASURE LEGS POSITIONS: ")
    legsPositions = controller.readLegsPositions([0, 1, 2, 3, 4], 'l')
    print(legsPositions)


    # _ = input("PRESS ENTER TO OFFLOAD: ")
    # newPose = controller.offloadSelectedLeg(1, testPins)
    # print("POSE AFTER OFFSET: ", newPose)
    # _ = input("PRESS ENTER TO MOVE THE LEG: ")
    # controller.moveLegAndGripper(1, pins[8], 5, startPose, testPins)


    # _ = input("PRESS ENTER TO START: ")
    # while True:
    #     controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 1.5, 'minJerk', topPose)
    #     time.sleep(2)
    #     controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 1.5, 'minJerk', rightPose)
    #     time.sleep(2)
    #     controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 1.5, 'minJerk', bottomPose)
    #     time.sleep(2)
    #     controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 1.5, 'minJerk', leftPose)
    #     time.sleep(2)

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




    