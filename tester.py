"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import time
import numpy as np
import matplotlib.pyplot as plt

import calculations
import dynamixel
import planning
import controllers
import environment as env
import simulaton as sim

if __name__ == "__main__":
    wall = env.Wall()
    plotter = sim.Plotter()
    planner = planning.PathPlanner()

    path = planner.calculateSpiderBodyPath([0.4, 0.25], [0.4, 1.1], 0.05)
    bestParams = [0.2 , 0.4, 0.4]
    selectedPins = planner.calculateSpiderLegsPositionsFF(path, bestParams)
    plotter.plotSpiderMovement(path, selectedPins)

    





    

