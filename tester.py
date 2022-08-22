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
    kinematics = calculations.Kinematics()
    controller = controllers.VelocityController()
    print("START")
    goalPosition = [0.065 + 0.3, 0.0, -0.275]

    [controller.moveLegAsync(leg, goalPosition, 'l', 5, 'minJerk') for leg in range(5)]
    time.sleep(6)

    while True:
        legIdInput = input("Enter leg ID: ")
        while legIdInput not in ['0', '1', '2', '3', '4']:
            legIdInput = input("Invalid leg ID, enter again: ")
        legIdInput = int(legIdInput)
        localGoalPositionInputString = input("Enter XYZ goal position in local origin, separated by comma: ")
        localGoalPositionInputStringArray = localGoalPositionInputString.split(',')
        localGoalPosition = []
        for value in localGoalPositionInputStringArray:
            try:
                localGoalPosition.append(float(value))
            except:
                print(f"Cannot convert {value} to float.")
        while True:
            durationInput = input("Enter movement duration in seconds: ")
            try:
                durationInput = float(durationInput)
                break
            except:
                print("Cannot convert given duration to float.")
        controller.moveLegAsync(legIdInput, localGoalPosition, 'l', durationInput, 'minJerk')
        time.sleep(durationInput)




    