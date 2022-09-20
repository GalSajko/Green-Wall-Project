"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import controllers
import calculations
import environment as env
import planning
import simulaton
import dynamixel
import bno055

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
    udpServer = udpServer.UdpServer('192.168.1.8')
    initSendingThread()
    time.sleep(2)
    wall = env.Wall('squared')
    pins = wall.createGrid(True)
    startPins = [pins[22], pins[9], pins[7], pins[31], pins[33]]

    controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 5, 'minJerk', [0.6, 0.55, 0.3, 0.0])
    time.sleep(5)
    controller.moveGrippersWrapper([0, 1, 2, 3, 4], 'o')

    input("PRESS ENTER TO START BNO SENSOR")
    sensor = bno055.BNO055()
    while True:
        print(sensor.readEulers())

    
      






