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
    controller = controllers.VelocityController()
    
    time.sleep(3)
    result = controller.moveLegAsync(0, [0.35, 0.0, 0.15], 'l', 5, 'minJerk')
    time.sleep(1)
    result = controller.moveLegAsync(1, [0.35, 0.0, 0.15], 'l', 5, 'minJerk')
    time.sleep(1)
    result = controller.moveLegAsync(2, [0.35, 0.0, 0.15], 'l', 5, 'minJerk')
    time.sleep(1)
    result = controller.moveLegAsync(3, [0.35, 0.0, 0.15], 'l', 5, 'minJerk')
    time.sleep(1)
    result = controller.moveLegAsync(4, [0.35, 0.0, 0.15], 'l', 5, 'minJerk')
    time.sleep(1)
    result = controller.moveLegAsync(0, [0.35, 0.0, 0.0], 'l', 5, 'minJerk')
    time.sleep(1)
    result = controller.moveLegAsync(1, [0.35, 0.0, 0.0], 'l', 5, 'minJerk')
    time.sleep(1)
    result = controller.moveLegAsync(2, [0.35, 0.0, 0.0], 'l', 5, 'minJerk')
    time.sleep(1)
    result = controller.moveLegAsync(3, [0.35, 0.0, 0.0], 'l', 5, 'minJerk')
    time.sleep(1)
    result = controller.moveLegAsync(4, [0.35, 0.0, 0.0], 'l', 5, 'minJerk')
