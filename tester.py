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
    for leg in [1, 2, 3, 4]:
        spider.motorsVelocityController.grippersArduino.moveGripper(leg, 'c')
        time.sleep(1)
    while True:
        spider.motorsVelocityController.grippersArduino.openGrippersAndWait([1, 2, 3, 4])
        time.sleep(1)
        for leg in [1, 2, 3, 4]:
            spider.motorsVelocityController.grippersArduino.moveGripper(leg, 'c')
            time.sleep(1)

