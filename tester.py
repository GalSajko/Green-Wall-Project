"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import controllers
import calculations
import environment as env
import time
import random


if __name__ == "__main__":    
    controller = controllers.VelocityController()
    kinematics = calculations.Kinematics()
    wall = env.Wall('squared')
    pins = wall.createGrid(True)
    usedPins = [pins[21], pins[8], pins[6], pins[30], pins[32]]
    controller.movePlatformAsync([0.6, 0.3, 0.3, 0.0], 5, usedPins)
    # while 1:
    #     result = controller.moveLegAsync(0, [0.35, 0.0, 0.0], 'l', 2, 'minJerk')
        # result = controller.moveLegAsync(1, [0.35, 0.0, 0.0], 'l', 2, 'minJerk')
        # result = controller.moveLegAsync(2, [0.35, 0.0, 0.0], 'l', 2, 'minJerk')
        # result = controller.moveLegAsync(3, [0.35, 0.0, 0.0], 'l', 2, 'minJerk')
        # result = controller.moveLegAsync(4, [0.35, 0.0, 0.0], 'l', 2, 'minJerk')
        # time.sleep(2.5)
        # print(kinematics.legForwardKinematics(1, controller.qA[1])[:3][:,3])
        # time.sleep(2)
        
        # result = controller.moveLegAsync(0, [0.45, 0.0, 0.25], 'l', 1, 'minJerk')
        # result = controller.moveLegAsync(1, [0.45, 0.0, 0.25], 'l', 1, 'minJerk')
        # result = controller.moveLegAsync(2, [0.45, 0.0, 0.25], 'l', 1, 'minJerk')
        # result = controller.moveLegAsync(3, [0.45, 0.0, 0.25], 'l', 1, 'minJerk')
        # result = controller.moveLegAsync(4, [0.45, 0.0, 0.25], 'l', 1, 'minJerk')
        # time.sleep(1.1)
        # print(kinematics.legForwardKinematics(1, controller.qA[1])[:3][:,3])
        # time.sleep(2)
    # controller.endControllerThread()

