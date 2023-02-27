"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import app
import time
import numpy as np
from planning import pathplanner

import config


if __name__ == "__main__":
    spider = app.App()
    # time.sleep(1)
    # input("ENTER")
    # spider.motorsVelocityController.startForceMode([2, 3], [np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])])
    # spider.motorsVelocityController.startImpedanceMode(0, np.array([1.0, 0.0, 0.0]))
    # spider.spiderStatesManager(config.WORKING_STATE, (np.array([0.6, 0.4, 0.3, 0.0]), ))
    input("ENTER")
    spider.pumpsBnoArduino.resetBno()
    while True:
        rpy = spider.pumpsBnoArduino.getGravityVector()
        print(rpy)
        time.sleep(0.1)
    # for leg in [0, 1, 2, 3, 4]:
    #     spider.motorsVelocityController.moveLegAsync(leg, spider.xA[leg], [0.3, 0.0, 0.1], config.LEG_ORIGIN, 3, 'minJerk')
    #     time.sleep(3)

