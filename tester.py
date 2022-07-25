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


def forceSending(frequency):
    while True:
        udpServer.send(controller.forces)
        time.sleep(1.0 / frequency)

def initSendingThread():
    udpSendingThread = threading.Thread(target = forceSending, args = (20, ), name = 'udp_sending_thread', daemon = False)
    udpSendingThread.start()
    print("UDP thread is running.")

if __name__ == "__main__":  
    time.sleep(3)  
    controller = controllers.VelocityController()
    dyx = dynamixel.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]], False)
    udpServer = udpServer.UdpServer('192.168.1.3')
    wall = env.Wall('squared')
    pins = wall.createGrid(True)
    time.sleep(1)
    initSendingThread()

    pins = wall.createGrid(True)
    pathPlanner = planning.PathPlanner(0.05, 0.1)
    plotter = simulaton.Plotter('squared')
    start = [0.6, 0.3, 0.3, 0.0]
    goal = [0.6, 0.3, 0.3, 0.0]
    path = pathPlanner.calculateSpiderBodyPath(start, goal)
    pins, rgValues = pathPlanner.calculateIdealLegsPositionsFF(path)
    print(pins)


    controller.moveLegsSync([0, 1, 2, 3, 4], pins[0], 'g', 4, 'minJerk', spiderPose = [0.6, 0.3, 0.3, 0.0]) 

    # time.sleep(5) 

    # newSpiderPose = controller.offloadSelectedLeg(0, [pins[9], pins[6], pins[30], pins[33]], None)
    # print(newSpiderPose)




    