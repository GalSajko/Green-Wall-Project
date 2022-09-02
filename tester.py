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
        udpServer.send(controller.qCds[4])
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
    for leg in [0, 1, 2, 3]:
        controller.disableEnableLegsWrapper(leg, 'd')

    controller.moveLegAsync(4, [0.4, 0.0, 0.1], 'l', 3, 'minJerk')
    input("PRESS ENTER TO START FORCE REGULATOR:")
    controller.startForceControl()
    
      






