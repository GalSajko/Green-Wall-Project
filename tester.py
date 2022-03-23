"""
This module is meant as a testing sandbox for other modules, during implementation.
"""

import numpy as np

import environment
import simulaton
import calculations 

if __name__ == "__main__":

    pathPlanner = calculations.PathPlanner()
    path = pathPlanner.calculateSpiderBodyPath([0.5, 0.5], [5.5, 3.5], 0.05)
    bestParams = [0.0323541 , 0.38081064, 0.58683527]
    selectedPins = pathPlanner.calculateSpiderLegsPositionsFF(path, bestParams)

    kinematics = calculations.Kinematics()
    joints = kinematics.legInverseKinematics(0, [0.5, 0, 0])
    print(joints)

    print(np.degrees(joints))

    # plotter = simulaton.Plotter()
    # plotter.plotSpiderMovement(path, selectedPins)