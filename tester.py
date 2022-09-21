"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import controllers
import calculations
import environment as env
import planning
import simulaton
import dynamixel
import periphery

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
    controller = controllers.VelocityController()
    wall = env.Wall('squared')
    pins = wall.createGrid(True)
    startPins = [pins[22], pins[9], pins[7], pins[31], pins[33]]

    controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 5, 'minJerk', [0.6, 0.55, 0.3, 0.0])
    time.sleep(5)

    input("PRESS ENTER TO START BNO SENSOR")
    sensor = periphery.BNO055()
    print(sensor.readEulers())
    while True:
        controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 5, 'minJerk', [0.6, 0.55, 0.3, 0.2])
        time.sleep(6)
        print(sensor.readEulers())
        controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 5, 'minJerk', [0.6, 0.55, 0.3, 0.0])
        time.sleep(6)
        print(sensor.readEulers())
        controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 5, 'minJerk', [0.6, 0.55, 0.3, -0.2])
        time.sleep(6)
        print(sensor.readEulers())
        controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 5, 'minJerk', [0.6, 0.55, 0.3, 0.0])
        time.sleep(6)
        print(sensor.readEulers())

    
      






