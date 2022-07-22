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

    # pins = wall.createGrid(True)
    # pathPlanner = planning.PathPlanner(0.05, 0.1)
    # plotter = simulaton.Plotter('squared')
    # start = [0.6, 0.3, 0.3, 0.0]
    # goal = [0.6, 0.3, 0.3, 0.0]
    # path = pathPlanner.calculateSpiderBodyPath(start, goal)
    # pins, rgValues = pathPlanner.calculateIdealLegsPositionsFF(path)

    # time.sleep(3)

    # result = controller.moveLegAsync(0, pins[21], 'g', 4, 'minJerk', [0.6, 0.3, 0.3, 0.0])
    # result = controller.moveLegAsync(1, pins[8], 'g', 4, 'minJerk', [0.6, 0.3, 0.3, 0.0])
    # result = controller.moveLegAsync(2, pins[6], 'g', 4, 'minJerk', [0.6, 0.3, 0.3, 0.0])
    # result = controller.moveLegAsync(3, pins[30], 'g', 4, 'minJerk', [0.6, 0.3, 0.3, 0.0])
    # result = controller.moveLegAsync(4, pins[32], 'g', 4, 'minJerk', [0.6, 0.3, 0.3, 0.0])    


    time.sleep(5) 

    controller.offloadSelectedLeg(0, [pins[8], pins[6], pins[30], pins[32]], None)

    # time.sleep(10)

    # controller.moveGripperWrapper(0, 'o')
    # time.sleep(1)
    # controller.moveLegAsync(0, pins[27], 'g', 4, 'bezier', newSpiderPose)
    # time.sleep(5)
    # controller.moveGripperWrapper(0, 'c')


    # kinematics = calculations.Kinematics()
    # wall = env.Wall('squared')
    # pins = wall.createGrid(True)
    # usedPins = [pins[21], pins[8], pins[6], pins[30], pins[32]]
    # controller.movePlatformAsync([0.6, 0.3, 0.3, 0.0], 5, usedPins)
    # while 1:
    #     result = controller.moveLegAsync(0, [0.35, 0.0, 0.0], 'l', 2, 'minJerk')
        # result = controller.moveLegAsync(1, [0.35, 0.0, 0.0], 'l', 2, 'minJerk')
        # result = controller.moveLegAsync(2, [0.35, 0.0, 0.0], 'l', 2, 'minJerk')
        # result = controller.moveLegAsync(3, [0.35, 0.0, 0.0], 'l', 2, 'minJerk')
        # result = controller.moveLegAsync(4, [0.35, 0.0, 0.0], 'l', 2, 'minJerk')
        # time.sleep(2.5)
        # print(kinematics.legForwardKinematics(1, controller.qA[1])[:3][:,3])
        # time.sleep(2)
        
        # result = controller.moveLegAsync(0, [0.45, 0.0, 0.25], 'l', 1, 'minJerk')
        # result = controller.moveLegAsync(1, [0.45, 0.0, 0.25], 'l', 1, 'minJerk')
        # result = controller.moveLegAsync(2, [0.45, 0.0, 0.25], 'l', 1, 'minJerk')
        # result = controller.moveLegAsync(3, [0.45, 0.0, 0.25], 'l', 1, 'minJerk')
        # result = controller.moveLegAsync(4, [0.45, 0.0, 0.25], 'l', 1, 'minJerk')
        # time.sleep(1.1)
        # print(kinematics.legForwardKinematics(1, controller.qA[1])[:3][:,3])
        # time.sleep(2)
    # controller.endControllerThread()



    