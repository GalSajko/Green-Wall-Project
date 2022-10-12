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

    
    controller.moveLegsSync([0, 1, 2, 3, 4], startPins, 'g', 3, 'minJerk', [0.6, 0.5, 0.3, 0.0])
    time.sleep(5)
    _ = input("PRESS ENTER TO START FORCE CONTROLL")
    
    while True:
        print("FORCE MODE")
        st = time.perf_counter()
        while True:
            fgSum, fDist = Dynamics().distributeForces([0, 1, 2, 3, 4], controller.fAMean)
            controller.startForceMode([0, 1, 2, 3, 4], fDist)
            time.sleep(0.2)
            et = time.perf_counter() - st
            if et > 15:
                break
        print("POSITION MODE")
        controller.stopForceMode()
        time.sleep(15)


    
    