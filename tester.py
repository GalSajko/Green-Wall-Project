"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import controllers

import time
import udpServer as udpServer
import threading

def forceSending(frequency):
    while True:
        udpServer.send(controller.fA)
        time.sleep(1.0 / frequency)

def initSendingThread():
    udpSendingThread = threading.Thread(target = forceSending, args = (20, ), name = 'udp_sending_thread', daemon = False)
    udpSendingThread.start()
    print("UDP thread is running.")

if __name__ == "__main__":
    controller = controllers.VelocityController(True)
    # controller.moveLegsSync([0, 1, 2, 3, 4], [[0.35, 0.0, -0.3]] * 5, 'l', 3, 'minJerk')
    
    controller.startForceMode([2], [[0.0, 0.0, 0.0]]*1)
    while True:
        controller.moveLegAsync(0, [0.35, 0.0, 0.3], 'l', 2, 'minJerk')
        time.sleep(0.5)
        controller.moveLegAsync(1, [0.35, 0.0, 0.3], 'l', 2, 'minJerk')
        time.sleep(0.5)
        controller.moveLegAsync(3, [0.35, 0.0, 0.3], 'l', 2, 'minJerk')
        time.sleep(0.5)
        controller.moveLegAsync(4, [0.35, 0.0, 0.3], 'l', 2, 'minJerk')
        time.sleep(0.5)

        controller.moveLegAsync(0, [0.35, 0.0, 0.0], 'l', 2, 'minJerk')
        time.sleep(0.5)
        controller.moveLegAsync(1, [0.35, 0.0, 0.0], 'l', 2, 'minJerk')
        time.sleep(0.5)
        controller.moveLegAsync(3, [0.35, 0.0, 0.0], 'l', 2, 'minJerk')
        time.sleep(0.5)
        controller.moveLegAsync(4, [0.35, 0.0, 0.0], 'l', 2, 'minJerk')
        time.sleep(0.5)
