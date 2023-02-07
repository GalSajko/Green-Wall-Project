"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import app
import time
import numpy as np

import config


if __name__ == "__main__":
    spider = app.App()
    time.sleep(1)
    spider.spiderStatesManager(config.WORKING_STATE, (np.array([0.6, 0.4, 0.3, 0.0]), ))
    # input("ENTER")
    # with spider.statesObjectsLocker:
    #     spider.hwErrors[2, 2] = 1.0

