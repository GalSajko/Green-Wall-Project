"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import controllers
import numbafunctions as nf
from calculations import Dynamics
import environment as env

import time
import numpy as np

if __name__ == "__main__":
    nf.initNumbaFunctions()

    wall = env.Wall('squared')
    pins = wall.createGrid(True)
    startPins = [pins[22], pins[9], pins[7], pins[31], pins[33]]
    controller = controllers.VelocityController(True)

    
    leg = 4
    controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 3, 'minJerk', [0.6, 0.5, 0.3, 0.0])
    time.sleep(5)
    _ = input("PRESS ENTER TO START FORCE CONTROLL")
    # while True:
    print("FORCE MODE")

    controller.startForceMode([leg], [[0.0, 0.0, 0.0]])
    time.sleep(1)
    st = time.perf_counter()
    # fDist = Dynamics().distributeForces(controller.fAMean, 3, startPins)
    while True:
        fDist = Dynamics().distributeForces(controller.fAMean, leg, startPins)
        controller.startForceMode([0, 1, 2, 3, 4], fDist)
        time.sleep(0.2)
        et = time.perf_counter() - st
        if et > 1.5:
            break
        # print("POSITION MODE")
        # controller.stopForceMode()
        # time.sleep(15)
    
    
    controller.stopForceMode()
    print("STOP FORCE MODE")
    time.sleep(5)
    controller.moveLegAsync(leg, np.array([0.0, 0.0, 0.1], dtype = np.float32), 'g', 1, 'minJerk', [0.6, 0.5, 0.3, 0.0], True)
    time.sleep(1)
    controller.moveLegAsync(leg, np.array([-0.2, 0.25, 0.0], dtype = np.float32), 'g', 2, 'minJerk', [0.6, 0.5, 0.3, 0.0], True)
    time.sleep(2)
    controller.moveLegAsync(leg, np.array([0.0, 0.0, -0.1], dtype = np.float32), 'g', 1, 'minJerk', [0.6, 0.5, 0.3, 0.0], True)
    
    