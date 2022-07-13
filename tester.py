"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import controllers
import calculations
import time
import random


if __name__ == "__main__":    
    controller = controllers.VelocityController()
    kinematics = calculations.Kinematics()
    # motors = dynamixel.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]], False)
    result = controller.moveLegAsync(1, [0.3, 0.0, 0.0], 'l', 3, 'minJerk')
    result = controller.moveLegAsync(2, [0.365, 0.0, 0.3], 'l', 3, 'minJerk')
    result = controller.moveLegAsync(3, [0.365, 0.0, 0.3], 'l', 3, 'minJerk')
    result = controller.moveLegAsync(4, [0.365, 0.0, 0.3], 'l', 3, 'minJerk')

    leg = 3
    # while 1:
        # result = controller.moveLegAsync(1, [0.35, 0.0, 0.0], 'l', 1, 'minJerk')
        # result = controller.moveLegAsync(2, [0.35, 0.0, 0.0], 'l', 1, 'minJerk')
        # result = controller.moveLegAsync(3, [0.35, 0.0, 0.0], 'l', 1, 'minJerk')
        # result = controller.moveLegAsync(4, [0.35, 0.0, 0.0], 'l', 1, 'minJerk')
        # result = controller.moveLegAsync(0, [0.45, 0.0, 0.0], 'l', 2, 'minJerk')
        # time.sleep(2)
        # print(kinematics.legForwardKinematics(leg, controller.qA[1])[:3][:,3])
        # print(kinematics.legForwardKinematics(leg, controller.qA[2])[:3][:,3])
        # time.sleep(2)
        
    # result = controller.moveLegAsync(1, [0.45, 0.0, 0.25], 'l', 1, 'minJerk')
    # result = controller.moveLegAsync(2, [0.45, 0.0, 0.25], 'l', 1, 'minJerk')
    # result = controller.moveLegAsync(3, [0.45, 0.0, 0.25], 'l', 1, 'minJerk')
    # result = controller.moveLegAsync(4, [0.45, 0.0, 0.25], 'l', 1, 'minJerk')
        # result = controller.moveLegAsync(0, [0.45, 0.0, 0.25], 'l', 2, 'minJerk')
        # time.sleep(2)
        # print(kinematics.legForwardKinematics(leg, controller.qA[1])[:3][:,3])
        # print(kinematics.legForwardKinematics(leg, controller.qA[2])[:3][:,3])
        # time.sleep(2)

    # controller.endControllerThread()

