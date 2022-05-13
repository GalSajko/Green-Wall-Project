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
    spider = env.Spider()
    wall = env.Wall('squared')
    plotter = sim.Plotter('squared')
    velocityController = controllers.VelocityController()
    pathPlanner = planning.PathPlanner(0.05, 0.2, 'squared')
    trajPlanner = planning.TrajectoryPlanner()
    kinematics = calculations.Kinematics()


    spiderStartPose = [0.4, 0.33, spider.LYING_HEIGHT, 0]
    spiderGoalPose = [0.4, 1, spider.WALKING_HEIGHT, 0]

    velocityController.walk(spiderStartPose, spiderGoalPose)





    





    

