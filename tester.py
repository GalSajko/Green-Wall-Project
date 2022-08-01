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
        udpServer.send(controller.forces)
        time.sleep(1.0 / frequency)

def initSendingThread():
    udpSendingThread = threading.Thread(target = forceSending, args = (20, ), name = 'udp_sending_thread', daemon = False)
    udpSendingThread.start()
    print("UDP thread is running.")

if __name__ == "__main__":  
    wall = env.Wall('squared')
    pathPlanner = planning.PathPlanner(0.05, 0.1)
    plotter = simulaton.Plotter('squared')
    udpServer = udpServer.UdpServer('192.168.1.3')
    controller = controllers.VelocityController()
    start = [0.6, 0.3, 0.25, 0.0]
    goal = [0.6, 0.3, 0.25, 0.0]
    pins = wall.createGrid(True)
    
    controller.moveLegsSync([0, 1, 2, 3, 4], [pins[21], pins[8], pins[6], pins[30], pins[32]], 'g', 5, 'minJerk', goal)

    time.sleep()

    controller.offloadSelectedLeg(0)
    
    




    