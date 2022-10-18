"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import controllers
from environment import wall
import udpServer as udpServer

import time
import threading


def forceSending(frequency):
    while True:
        udpServer.send(controller.tauAMean)
        time.sleep(1.0 / frequency)

def initSendingThread():
    udpSendingThread = threading.Thread(target = forceSending, args = (20, ), name = 'udp_sending_thread', daemon = False)
    udpSendingThread.start()
    print("UDP thread is running.")

if __name__ == "__main__":
    pins = wall.createGrid(True)
    startPins = [pins[22], pins[9], pins[7], pins[31], pins[33]]
    controller = controllers.VelocityController(True)
    udpServer = udpServer.UdpServer('192.168.1.38')
    initSendingThread()

    
    leg = 4
    controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 3, 'minJerk', [0.6, 0.5, 0.3, 0.0])
    time.sleep(5)
    _ = input("PRESS ENTER TO START FORCE CONTROLL")

    controller.forceDistribution()
    
    