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
        udpServer.send(controller.torques)
        time.sleep(1.0 / frequency)

def initSendingThread():
    udpSendingThread = threading.Thread(target = forceSending, args = (20, ), name = 'udp_sending_thread', daemon = False)
    udpSendingThread.start()
    print("UDP thread is running.")

if __name__ == "__main__":
    controller = controllers.VelocityController()
    udpServer = udpServer.UdpServer('192.168.1.32')
    initSendingThread()
    time.sleep(3)

    # controller.moveLegAsync(4, [0.35, 0.0, 0.2], 'l', 5, 'minJerk')
    controller.startForceMode(1)
    # sensor = periphery.BNO055()
    # while True:
    #     print(sensor.readGravity())
    #     time.sleep(0.1)

    
      







