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
    matrixCalculator = calculations.MatrixCalculator()

    appPoint = matrixCalculator.getLegApproachPositionInGlobal(0, [1, 1, 0.2, 0, 0, 0], [1, 1.5, 0])

    print(np.round(appPoint, 3))



 






    





    

