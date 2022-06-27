"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import numpy as np
import matplotlib.pyplot as plt
import serial
import threading
import itertools as itt
import time

import calculations
import dynamixel
import planning
import controllers
import environment as env
import simulaton as sim

if __name__ == "__main__":
    # controller = controllers.VelocityController()
    dyx = dynamixel.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]], False)
    while True:
        start = time.time()
        dyx.syncReadMotorsPositionsInLegs([0, 1, 2, 3, 4])
        t = (time.time() - start) * 1000.0
        t = "{:.2f}".format(t)
        print(t)
    
    # time.sleep(3)
    # result = controller.moveLegAsync(0, [0.35, 0.0, 0.15], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(1, [0.35, 0.0, 0.15], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(2, [0.35, 0.0, 0.15], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(3, [0.35, 0.0, 0.15], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(4, [0.35, 0.0, 0.15], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(0, [0.35, 0.0, 0.0], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(1, [0.35, 0.0, 0.0], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(2, [0.35, 0.0, 0.0], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(3, [0.35, 0.0, 0.0], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(4, [0.35, 0.0, 0.0], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(0, [0.35, 0.0, 0.15], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(1, [0.35, 0.0, 0.15], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(2, [0.35, 0.0, 0.15], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(3, [0.35, 0.0, 0.15], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(4, [0.35, 0.0, 0.15], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(0, [0.35, 0.0, 0.0], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(1, [0.35, 0.0, 0.0], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(2, [0.35, 0.0, 0.0], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(3, [0.35, 0.0, 0.0], 'l', 5, 'minJerk')
    # time.sleep(1)
    # result = controller.moveLegAsync(4, [0.35, 0.0, 0.0], 'l', 5, 'minJerk')
