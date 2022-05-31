"""
This module is meant as a testing sandbox for other modules, during implementation.
"""
import time
import numpy as np
import matplotlib.pyplot as plt
import serial
import threading

import calculations
import dynamixel
import planning
import controllers
import environment as env
import simulaton as sim

if __name__ == "__main__":
    wall = env.Wall('squared')
    grid = wall.createGrid()
    plotter = sim.Plotter('squared')
    plotter.plotWallGrid()

    

    
    






    





    

