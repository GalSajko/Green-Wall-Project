"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import controllers
import time
import random


if __name__ == "__main__":    
    controller = controllers.VelocityController()
    # result = controller.moveLegAsync(4, [0.35, 0.0, 0.25], 'l', 3, 'minJerk')
    while 1:
        result = controller.moveLegAsync(4, [0.35, 0.0, 0.0], 'l', 2, 'minJerk')
        # result = controller.moveLegAsync(3, [0.35, 0.0, 0.0], 'l', 3, 'minJerk')
        # result = controller.moveLegAsync(2, [0.35, 0.0, 0.0], 'l', 3, 'minJerk')
        # result = controller.moveLegAsync(1, [0.35, 0.0, 0.0], 'l', 3, 'minJerk')
        # result = controller.moveLegAsync(0, [0.45, 0.0, 0.0], 'l', 2, 'minJerk')
        time.sleep(1)
        result = controller.moveLegAsync(4, [0.45, 0.0, 0.25], 'l', 2, 'minJerk')
        # result = controller.moveLegAsync(3, [0.45, 0.0, 0.25], 'l', 2, 'minJerk')
        # result = controller.moveLegAsync(2, [0.45, 0.0, 0.25], 'l', 2, 'minJerk')
        # result = controller.moveLegAsync(1, [0.45, 0.0, 0.25], 'l', 2, 'minJerk')
        # result = controller.moveLegAsync(0, [0.45, 0.0, 0.25], 'l', 2, 'minJerk')
        time.sleep(1)

    # controller.endControllerThread()

