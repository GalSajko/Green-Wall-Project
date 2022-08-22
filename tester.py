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
    controller = controller.VelocityController()
    controller.moveLegAsync(0, [0.065 + 0.3, -0.275], 'l', 5, 'minJerk')
    


    