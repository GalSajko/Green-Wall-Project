"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import app
import time
import numpy as np
from planning import pathplanner

import config
from environment import wall, spider


if __name__ == "__main__":
    spiderApp = app.App()
    time.sleep(1)
    # input("ENTER")
    # spider.motorsVelocityController.startForceMode([2, 3], [np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])])
    # spider.motorsVelocityController.startImpedanceMode(0, np.array([1.0, 0.0, 0.0]))
    # spider.spiderStatesManager(config.WORKING_STATE, (np.array([0.6, 0.4, 0.3, 0.0]), ))
    while True:
        spiderApp.pumpsBnoArduino.pumpControll('0', 1)
        time.sleep(0.05)
    # pins = wall.createGrid(True)
    # usedPins = [pins[22], pins[9], pins[12], pins[24], pins[33]]
    # startPose = [0.4, 0.5, 0.3, 0.0]

    # xA = spiderApp.xA
    # spiderApp.motorsVelocityController.moveLegsSync(spider.LEGS_IDS, xA, usedPins, config.GLOBAL_ORIGIN, 5, config.MINJERK_TRAJECTORY, startPose)
    # time.sleep(6)
    # spiderApp.pumpsBnoArduino.resetBno()
    # time.sleep(1)

    # poses = [
    #     [0.4, 0.5, 0.3, 0.1, 0.1, 0.0],
    #     [0.4, 0.5, 0.3, -0.1, 0.1, -0.0],
    #     [0.4, 0.5, 0.3, 0.1, -0.1, 0.0],
    #     [0.4, 0.5, 0.3, -0.1, -0.1, -0.0],
    # ]


    # while True:
    #     for pose in poses:
    #         spiderApp.motorsVelocityController.moveLegsSync(spider.LEGS_IDS, spiderApp.xA, usedPins, config.GLOBAL_ORIGIN, 3, config.MINJERK_TRAJECTORY, pose)
    #         time.sleep(4)
    #         print("RPY BEFORE OFFLOADING: ", spiderApp.pumpsBnoArduino.getRpy())



    #         spiderApp.pinToPinMovement(1, usedPins[1], pins[3])
    #         time.sleep(3)
    #         spiderApp.pinToPinMovement(1, pins[3], usedPins[1])
    #         time.sleep(3)
    

