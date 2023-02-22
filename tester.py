"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import app
import time
import numpy as np
from planning import pathplanner

import config


if __name__ == "__main__":
    # spider = app.App()
    # time.sleep(1)
    # input("ENTER")
    # spider.motorsVelocityController.startForceMode([2, 3], [np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])])
    # spider.motorsVelocityController.startImpedanceMode(0, np.array([1.0, 0.0, 0.0]))
    # spider.spiderStatesManager(config.WORKING_STATE, (np.array([0.6, 0.4, 0.3, 0.0]), ))
    pathplanner.createWalkingInstructions([0.48, 0.75, 0.3, 0.0], [0.36, 0.65, 0.3, 0.0])

